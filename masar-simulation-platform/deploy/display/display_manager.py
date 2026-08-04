#!/usr/bin/env python3
"""Own the three fixed X11/noVNC display stacks without global process cleanup."""

from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import json
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import threading
import time
from types import MappingProxyType
from typing import Callable, Mapping, Sequence


_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,127}$")


class DisplayLifecycleError(RuntimeError):
    pass


class DisplayConflict(DisplayLifecycleError):
    pass


class DisplayOwnershipError(DisplayLifecycleError):
    pass


@dataclass(frozen=True, slots=True)
class DisplaySpec:
    name: str
    display: str
    vnc_port: int
    novnc_port: int

    @property
    def display_number(self) -> int:
        return int(self.display.removeprefix(":"))


DISPLAY_SPECS: Mapping[str, DisplaySpec] = MappingProxyType(
    {
        "gazebo": DisplaySpec("gazebo", ":101", 19131, 18131),
        "rviz": DisplaySpec("rviz", ":102", 19132, 18132),
        "webots": DisplaySpec("webots", ":103", 19133, 18133),
    }
)


@dataclass(frozen=True, slots=True)
class ComponentCommand:
    name: str
    argv: tuple[str, ...]
    environment: Mapping[str, str] | None = None

    def __post_init__(self) -> None:
        if not self.name or not self.argv or any(not item or "\0" in item for item in self.argv):
            raise ValueError("component command must contain a safe name and argv")


@dataclass(frozen=True, slots=True)
class ComponentRecord:
    name: str
    argv: tuple[str, ...]
    pid: int
    pgid: int
    start_ticks: int
    status: str
    signals_sent: tuple[str, ...] = ()

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "argv": list(self.argv),
            "pid": self.pid,
            "pgid": self.pgid,
            "start_ticks": self.start_ticks,
            "status": self.status,
            "signals_sent": list(self.signals_sent),
        }


@dataclass(frozen=True, slots=True)
class DisplayRecord:
    name: str
    display: str
    vnc_port: int
    novnc_port: int
    status: str
    components: tuple[ComponentRecord, ...]

    def as_dict(self) -> dict:
        return {
            "name": self.name,
            "display": self.display,
            "vnc_port": self.vnc_port,
            "novnc_port": self.novnc_port,
            "status": self.status,
            "components": [component.as_dict() for component in self.components],
        }


@dataclass(frozen=True, slots=True)
class LifecycleState:
    run_id: str
    status: str
    displays: Mapping[str, DisplayRecord]

    def as_dict(self) -> dict:
        return {
            "schema_version": 1,
            "run_id": self.run_id,
            "status": self.status,
            "displays": {name: display.as_dict() for name, display in self.displays.items()},
        }


@dataclass(frozen=True, slots=True)
class ShutdownGrace:
    sigint: float = 2.0
    sigterm: float = 2.0
    sigkill: float = 1.0

    def __post_init__(self) -> None:
        if any(value < 0 or value > 30 for value in (self.sigint, self.sigterm, self.sigkill)):
            raise ValueError("shutdown grace values must be between 0 and 30 seconds")


CommandFactory = Callable[[DisplaySpec], Sequence[ComponentCommand]]
ReadinessCheck = Callable[[DisplaySpec], bool]


class DisplayLifecycleManager:
    def __init__(
        self,
        state_dir: str | Path = "/run/masar-platform-displays",
        x11_dir: str | Path = "/tmp/.X11-unix",
        *,
        command_factory: CommandFactory | None = None,
        readiness_check: ReadinessCheck | None = None,
        startup_timeout: float = 8.0,
    ) -> None:
        self.state_dir = Path(state_dir)
        self.x11_dir = Path(x11_dir)
        self.command_factory = command_factory or production_commands
        self.readiness_check = readiness_check or self._x_socket_ready
        self.startup_timeout = startup_timeout
        self._state: LifecycleState | None = None
        self._children: dict[int, subprocess.Popen] = {}

    @property
    def state_path(self) -> Path:
        return self.state_dir / "state.json"

    def start(self, run_id: str, names: Sequence[str] = ("gazebo", "rviz", "webots")) -> LifecycleState:
        _validate_identifier(run_id)
        requested = tuple(names)
        if not requested or len(requested) != len(set(requested)):
            raise ValueError("at least one unique display name is required")
        unknown = set(requested) - set(DISPLAY_SPECS)
        if unknown:
            raise ValueError(f"unknown display(s): {sorted(unknown)}")
        if self.state_path.exists():
            raise DisplayConflict("a display lifecycle state already exists")

        self.x11_dir.mkdir(parents=True, exist_ok=True)
        self.state_dir.mkdir(parents=True, exist_ok=True)
        for name in requested:
            self._assert_display_available(DISPLAY_SPECS[name])

        displays: dict[str, DisplayRecord] = {}
        self._state = LifecycleState(run_id, "STARTING", MappingProxyType(displays))
        self._write_state(self._state)
        try:
            for name in requested:
                spec = DISPLAY_SPECS[name]
                components: list[ComponentRecord] = []
                commands = tuple(self.command_factory(spec))
                if tuple(command.name for command in commands) != (
                    "xvfb",
                    "desktop",
                    "x11vnc",
                    "websockify",
                ):
                    raise DisplayLifecycleError("display stack must define the four fixed components")
                for index, command in enumerate(commands):
                    components.append(self._spawn(command))
                    displays[name] = DisplayRecord(
                        name=name,
                        display=spec.display,
                        vnc_port=spec.vnc_port,
                        novnc_port=spec.novnc_port,
                        status="STARTING",
                        components=tuple(components),
                    )
                    self._state = LifecycleState(run_id, "STARTING", MappingProxyType(dict(displays)))
                    self._write_state(self._state)
                    if index == 0:
                        self._wait_ready(spec, components[0])
                displays[name] = DisplayRecord(
                    name=name,
                    display=spec.display,
                    vnc_port=spec.vnc_port,
                    novnc_port=spec.novnc_port,
                    status="RUNNING",
                    components=tuple(components),
                )
                self._state = LifecycleState(run_id, "STARTING", MappingProxyType(dict(displays)))
                self._write_state(self._state)
            self._state = LifecycleState(run_id, "RUNNING", MappingProxyType(dict(displays)))
            self._write_state(self._state)
            return self._state
        except BaseException:
            self._state = LifecycleState(run_id, "ERROR", MappingProxyType(dict(displays)))
            self._stop_records(ShutdownGrace(0.1, 0.1, 0.2))
            try:
                self.state_path.unlink()
            except FileNotFoundError:
                pass
            self._state = None
            raise

    def poll(self) -> LifecycleState:
        state = self.inspect()
        self._write_state(state)
        return state

    def inspect(self) -> LifecycleState:
        state = self._state or self.load()
        displays: dict[str, DisplayRecord] = {}
        has_error = False
        for name, display in state.displays.items():
            components: list[ComponentRecord] = []
            for component in display.components:
                status = component.status
                if status == "RUNNING" and not self._component_active(component):
                    status = "EXITED"
                components.append(replace(component, status=status))
            display_status = display.status
            if display_status == "RUNNING" and any(item.status != "RUNNING" for item in components):
                display_status = "ERROR"
            has_error = has_error or display_status == "ERROR"
            displays[name] = replace(display, status=display_status, components=tuple(components))
        status = "ERROR" if has_error else state.status
        self._state = LifecycleState(state.run_id, status, MappingProxyType(displays))
        return self._state

    def start_or_recover(
        self,
        run_id: str,
        names: Sequence[str] = ("gazebo", "rviz", "webots"),
    ) -> LifecycleState:
        if not self.state_path.exists():
            return self.start(run_id, names)

        state = self.load()
        if state.run_id != run_id:
            raise DisplayConflict("display state belongs to another run")
        if tuple(state.displays) != tuple(names):
            raise DisplayConflict("display state does not match the requested stack")

        active: dict[int, bool] = {}
        for display in state.displays.values():
            for component in display.components:
                active[component.pid] = self._component_active(component)

        all_running = (
            state.status == "RUNNING"
            and all(display.status == "RUNNING" for display in state.displays.values())
            and all(active.values())
        )
        if all_running:
            self._state = state
            return state

        if any(active.values()):
            cleanup_displays: dict[str, DisplayRecord] = {}
            for name, display in state.displays.items():
                components = tuple(
                    replace(component, status="RUNNING" if active[component.pid] else "EXITED")
                    for component in display.components
                )
                cleanup_displays[name] = replace(display, status="ERROR", components=components)
            self._state = LifecycleState(run_id, "ERROR", MappingProxyType(cleanup_displays))
            self._stop_records(ShutdownGrace(0.1, 0.1, 0.2))

        self.state_path.unlink(missing_ok=True)
        self._state = None
        return self.start(run_id, names)

    def stop(
        self,
        run_id: str,
        grace: ShutdownGrace | None = None,
        *,
        missing_ok: bool = False,
    ) -> LifecycleState | None:
        _validate_identifier(run_id)
        try:
            state = self._state or self.load()
        except FileNotFoundError:
            if missing_ok:
                return None
            raise
        if state.run_id != run_id:
            raise DisplayOwnershipError("run_id does not own the display lifecycle")
        self._state = state
        return self._stop_records(grace or ShutdownGrace())

    def load(self) -> LifecycleState:
        payload = json.loads(self.state_path.read_text(encoding="utf-8"))
        if payload.get("schema_version") != 1:
            raise DisplayLifecycleError("unsupported display state schema")
        displays = {
            name: DisplayRecord(
                name=value["name"],
                display=value["display"],
                vnc_port=int(value["vnc_port"]),
                novnc_port=int(value["novnc_port"]),
                status=value["status"],
                components=tuple(
                    ComponentRecord(
                        name=item["name"],
                        argv=tuple(item["argv"]),
                        pid=int(item["pid"]),
                        pgid=int(item["pgid"]),
                        start_ticks=int(item["start_ticks"]),
                        status=item["status"],
                        signals_sent=tuple(item.get("signals_sent", ())),
                    )
                    for item in value["components"]
                ),
            )
            for name, value in payload["displays"].items()
        }
        return LifecycleState(payload["run_id"], payload["status"], MappingProxyType(displays))

    def _spawn(self, command: ComponentCommand) -> ComponentRecord:
        environment = os.environ.copy()
        environment.update(dict(command.environment or {}))
        child = subprocess.Popen(
            list(command.argv),
            env=environment,
            stdin=subprocess.DEVNULL,
            stdout=None,
            stderr=None,
            start_new_session=True,
            shell=False,
        )
        self._children[child.pid] = child
        try:
            return ComponentRecord(
                name=command.name,
                argv=command.argv,
                pid=child.pid,
                pgid=os.getpgid(child.pid),
                start_ticks=_proc_start_ticks(child.pid),
                status="RUNNING",
            )
        except BaseException:
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            child.wait(timeout=2)
            raise

    def _wait_ready(self, spec: DisplaySpec, xvfb: ComponentRecord) -> None:
        deadline = time.monotonic() + self.startup_timeout
        while time.monotonic() < deadline:
            if not self._component_active(xvfb):
                raise DisplayLifecycleError(f"Xvfb for {spec.name} exited during startup")
            if self.readiness_check(spec):
                return
            time.sleep(0.05)
        raise DisplayLifecycleError(f"X display {spec.display} did not become ready")

    def _assert_display_available(self, spec: DisplaySpec) -> None:
        socket_path = self.x11_dir / f"X{spec.display_number}"
        lock_path = self.x11_dir.parent / f".X{spec.display_number}-lock"
        if socket_path.exists() or socket_path.is_socket() or lock_path.exists():
            raise DisplayConflict(f"X display {spec.display} is already owned outside this lifecycle")

    def _x_socket_ready(self, spec: DisplaySpec) -> bool:
        return (self.x11_dir / f"X{spec.display_number}").is_socket()

    def _component_active(self, record: ComponentRecord) -> bool:
        child = self._children.get(record.pid)
        if child is not None and child.poll() is not None:
            return False
        try:
            pgid = os.getpgid(record.pid)
            start_ticks = _proc_start_ticks(record.pid)
        except (ProcessLookupError, FileNotFoundError):
            return False
        if pgid != record.pgid or start_ticks != record.start_ticks:
            raise DisplayOwnershipError(
                f"cannot prove ownership of {record.name} pid {record.pid}"
            )
        return True

    def _stop_records(self, grace: ShutdownGrace) -> LifecycleState:
        assert self._state is not None
        state = self._state
        displays: dict[str, DisplayRecord] = {}
        for name in reversed(tuple(state.displays)):
            display = state.displays[name]
            stopped: list[ComponentRecord] = []
            for component in reversed(display.components):
                stopped.append(self._stop_component(component, grace))
            displays[name] = replace(display, status="STOPPED", components=tuple(reversed(stopped)))
        ordered = {name: displays[name] for name in state.displays}
        self._state = LifecycleState(state.run_id, "STOPPED", MappingProxyType(ordered))
        self._write_state(self._state)
        return self._state

    def _stop_component(self, record: ComponentRecord, grace: ShutdownGrace) -> ComponentRecord:
        signals_sent = list(record.signals_sent)
        if record.status not in {"STOPPED", "EXITED"}:
            for signal_value, signal_name, wait_seconds in (
                (signal.SIGINT, "SIGINT", grace.sigint),
                (signal.SIGTERM, "SIGTERM", grace.sigterm),
                (signal.SIGKILL, "SIGKILL", grace.sigkill),
            ):
                if not self._component_active(record):
                    break
                try:
                    os.killpg(record.pgid, signal_value)
                except ProcessLookupError:
                    break
                signals_sent.append(signal_name)
                if self._wait_inactive(record, wait_seconds):
                    break
        child = self._children.get(record.pid)
        if child is not None:
            try:
                child.wait(timeout=0)
            except subprocess.TimeoutExpired:
                pass
        status = "STOPPED" if record.status != "EXITED" else "EXITED"
        return replace(record, status=status, signals_sent=tuple(signals_sent))

    def _wait_inactive(self, record: ComponentRecord, wait_seconds: float) -> bool:
        deadline = time.monotonic() + wait_seconds
        while True:
            if not self._component_active(record):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(min(0.02, max(0.001, wait_seconds / 10 if wait_seconds else 0.001)))

    def _write_state(self, state: LifecycleState) -> None:
        self.state_dir.mkdir(parents=True, exist_ok=True)
        temporary = self.state_path.with_suffix(".json.tmp")
        temporary.write_text(
            json.dumps(state.as_dict(), indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.chmod(0o600)
        os.replace(temporary, self.state_path)


def production_commands(spec: DisplaySpec) -> tuple[ComponentCommand, ...]:
    environment = {"DISPLAY": spec.display, "NO_PROXY": "127.0.0.1,localhost,::1", "no_proxy": "127.0.0.1,localhost,::1"}
    startup = f"/usr/local/bin/xstartup-{spec.name}"
    return (
        ComponentCommand(
            "xvfb",
            ("Xvfb", spec.display, "-screen", "0", "1280x720x24", "-nolisten", "tcp", "-noreset"),
        ),
        ComponentCommand("desktop", (startup,), environment),
        ComponentCommand(
            "x11vnc",
            (
                "x11vnc",
                "-display",
                spec.display,
                "-rfbport",
                str(spec.vnc_port),
                "-listen",
                "127.0.0.1",
                "-6",
                "-rfbportv6",
                str(spec.vnc_port),
                "-listenv6",
                "::1",
                "-forever",
                "-shared",
                "-nopw",
                "-noxdamage",
            ),
            environment,
        ),
        ComponentCommand(
            "websockify",
            (
                "websockify",
                "--web",
                "/opt/noVNC",
                "--heartbeat",
                "30",
                f"127.0.0.1:{spec.novnc_port}",
                f"127.0.0.1:{spec.vnc_port}",
            ),
            environment,
        ),
    )


def _proc_start_ticks(pid: int) -> int:
    stat_value = Path(f"/proc/{pid}/stat").read_text(encoding="utf-8")
    fields = stat_value[stat_value.rfind(")") + 2 :].split()
    return int(fields[19])


def _validate_identifier(value: str) -> None:
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise ValueError("run_id is not a safe identifier")


def _tcp_ready(port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=0.5):
            return True
    except OSError:
        return False


def _serve(manager: DisplayLifecycleManager, run_id: str) -> int:
    stop_event = threading.Event()

    def request_stop(_signal: int, _frame: object) -> None:
        stop_event.set()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    manager.start_or_recover(run_id)
    try:
        while not stop_event.wait(0.5):
            manager.poll()
    finally:
        manager.stop(run_id, missing_ok=True)
    return 0


def _health(manager: DisplayLifecycleManager) -> int:
    try:
        state = manager.inspect()
    except (OSError, ValueError, DisplayLifecycleError):
        return 1
    if state.status != "RUNNING" or any(display.status != "RUNNING" for display in state.displays.values()):
        return 1
    return 0 if all(_tcp_ready(display.novnc_port) for display in state.displays.values()) else 1


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-dir", default="/run/masar-platform-displays")
    parser.add_argument("--x11-dir", default="/tmp/.X11-unix")
    subparsers = parser.add_subparsers(dest="command", required=True)
    serve = subparsers.add_parser("serve")
    serve.add_argument("--run-id", default="platform-display-stack")
    subparsers.add_parser("health")
    args = parser.parse_args(argv)
    manager = DisplayLifecycleManager(args.state_dir, args.x11_dir)
    if args.command == "serve":
        return _serve(manager, args.run_id)
    return _health(manager)


if __name__ == "__main__":
    raise SystemExit(main())
