"""Run-owned process groups, records, bounded shutdown, and log cursors."""

from __future__ import annotations

import json
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
from dataclasses import dataclass, replace
from datetime import datetime, timezone
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple


_IDENTIFIER = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]{0,127}$")
_ENVIRONMENT_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")
_LOCAL_NO_PROXY_ENTRIES = ("127.0.0.1", "localhost", "::1")


class ProcessManagerError(RuntimeError):
    pass


class CommandNotAllowed(ProcessManagerError):
    pass


class ProcessConflict(ProcessManagerError):
    pass


class ProcessNotFound(ProcessManagerError):
    pass


class ProcessOwnershipError(ProcessManagerError):
    pass


@dataclass(frozen=True)
class CommandPolicy:
    agent: str
    executable: str


@dataclass(frozen=True)
class GracePeriods:
    sigint: float = 10.0
    sigterm: float = 5.0
    sigkill: float = 5.0

    def __post_init__(self) -> None:
        for value in (self.sigint, self.sigterm, self.sigkill):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or value < 0:
                raise ValueError("grace periods must be non-negative numbers")


@dataclass(frozen=True)
class ProcessRecord:
    run_id: str
    process_id: str
    command_ref: str
    argv: Tuple[str, ...]
    pid: int
    pgid: int
    status: str
    return_code: Optional[int]
    started_at: str
    ended_at: Optional[str]
    signals_sent: Tuple[str, ...]
    record_path: str
    stdout_path: str
    stderr_path: str

    def as_dict(self) -> Dict[str, Any]:
        return {
            "run_id": self.run_id,
            "process_id": self.process_id,
            "command_ref": self.command_ref,
            "argv": list(self.argv),
            "pid": self.pid,
            "pgid": self.pgid,
            "status": self.status,
            "return_code": self.return_code,
            "started_at": self.started_at,
            "ended_at": self.ended_at,
            "signals_sent": list(self.signals_sent),
            "record_path": self.record_path,
            "stdout_path": self.stdout_path,
            "stderr_path": self.stderr_path,
        }


@dataclass(frozen=True)
class LogChunk:
    data: str
    cursor: int
    eof: bool

    def as_dict(self) -> Dict[str, Any]:
        return {"data": self.data, "cursor": self.cursor, "eof": self.eof}


DEFAULT_COMMAND_POLICIES: Mapping[str, CommandPolicy] = {
    "ros2.swarm": CommandPolicy("ros2", "ros2"),
    "ros1.response_deform": CommandPolicy("ros1", "./run_response_deform_demo.sh"),
    "ros1.response_controller": CommandPolicy("ros1", "./run_response_controller_demo.sh"),
    "ros1.exploration_gazebo": CommandPolicy("ros1", "./launch_robosar_g.sh"),
    "ros2.exploration_webots": CommandPolicy("ros2", "./start_robosar_webots.sh"),
    "ros1.exploration_webots": CommandPolicy("ros1", "./launch_robosar_w.sh"),
    "ros1.tepper_response": CommandPolicy("ros1", "./run_tepper_response_demo.sh"),
}


class ProcessManager:
    def __init__(
        self,
        agent_kind: str,
        workspace: Path,
        runs_dir: Path,
        command_policies: Optional[Mapping[str, CommandPolicy]] = None,
    ) -> None:
        self.agent_kind = agent_kind
        self.workspace = Path(workspace).resolve()
        self.runs_dir = Path(runs_dir).resolve()
        self.command_policies = dict(command_policies or DEFAULT_COMMAND_POLICIES)
        self._children: Dict[Tuple[str, str], subprocess.Popen] = {}
        self._lock = threading.RLock()

    def start(
        self,
        run_id: str,
        process_id: str,
        command_ref: str,
        argv: Sequence[str],
        environment: Optional[Mapping[str, str]] = None,
    ) -> ProcessRecord:
        _validate_identifier("run_id", run_id)
        _validate_identifier("process_id", process_id)
        normalized_argv = _validate_command(self.agent_kind, command_ref, argv, self.command_policies)
        normalized_environment = _validate_environment(environment or {})
        if not self.workspace.is_dir():
            raise ProcessManagerError("configured workspace does not exist")

        with self._lock:
            record_path = self._record_path(run_id, process_id)
            if record_path.exists():
                existing = self.get(run_id, process_id)
                if existing.status == "RUNNING":
                    raise ProcessConflict("process_id is already running for this run")
                raise ProcessConflict("process_id already has an ownership record")

            process_dir = record_path.parent.parent / process_id
            process_dir.mkdir(parents=True, exist_ok=True)
            stdout_path = process_dir / "stdout.log"
            stderr_path = process_dir / "stderr.log"
            child_environment = os.environ.copy()
            child_environment.update(normalized_environment)
            child_environment["MASAR_PLATFORM_MODE"] = "1"
            child_environment["NO_PROXY"] = _merge_no_proxy(child_environment.get("NO_PROXY", ""))
            child_environment["no_proxy"] = _merge_no_proxy(child_environment.get("no_proxy", ""))
            with stdout_path.open("ab", buffering=0) as stdout_stream:
                with stderr_path.open("ab", buffering=0) as stderr_stream:
                    child = subprocess.Popen(
                        list(normalized_argv),
                        cwd=str(self.workspace),
                        env=child_environment,
                        stdin=subprocess.DEVNULL,
                        stdout=stdout_stream,
                        stderr=stderr_stream,
                        start_new_session=True,
                        shell=False,
                    )
            try:
                pgid = os.getpgid(child.pid)
                record = ProcessRecord(
                    run_id=run_id,
                    process_id=process_id,
                    command_ref=command_ref,
                    argv=normalized_argv,
                    pid=child.pid,
                    pgid=pgid,
                    status="RUNNING",
                    return_code=None,
                    started_at=_utc_now(),
                    ended_at=None,
                    signals_sent=(),
                    record_path=str(record_path),
                    stdout_path=str(stdout_path),
                    stderr_path=str(stderr_path),
                )
                self._children[(run_id, process_id)] = child
                self._write_record(record)
                return record
            except BaseException:
                self._children.pop((run_id, process_id), None)
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                child.wait(timeout=5)
                raise

    def get(self, run_id: str, process_id: str) -> ProcessRecord:
        _validate_identifier("run_id", run_id)
        _validate_identifier("process_id", process_id)
        path = self._record_path(run_id, process_id)
        if not path.is_file():
            raise ProcessNotFound("process ownership record was not found")
        with self._lock:
            record = _record_from_dict(json.loads(path.read_text(encoding="utf-8")))
            return self._refresh(record)

    def probe(self, run_id: str) -> List[ProcessRecord]:
        _validate_identifier("run_id", run_id)
        process_root = self.runs_dir / run_id / "processes"
        if not process_root.is_dir():
            return []
        records = []
        for path in sorted(process_root.glob("*.json")):
            records.append(self.get(run_id, path.stem))
        return records

    def stop(
        self,
        run_id: str,
        process_id: str,
        grace: Optional[GracePeriods] = None,
    ) -> ProcessRecord:
        grace = grace or GracePeriods()
        with self._lock:
            record = self.get(run_id, process_id)
            if record.status in {"STOPPED", "EXITED"}:
                return record
            self._assert_owned(record)
            signals = list(record.signals_sent)
            for signal_value, signal_name, wait_seconds in (
                (signal.SIGINT, "SIGINT", grace.sigint),
                (signal.SIGTERM, "SIGTERM", grace.sigterm),
                (signal.SIGKILL, "SIGKILL", grace.sigkill),
            ):
                if not _pid_active(record.pid):
                    break
                self._assert_owned(record)
                os.killpg(record.pgid, signal_value)
                signals.append(signal_name)
                record = replace(record, signals_sent=tuple(signals))
                self._write_record(record)
                if self._wait_inactive(record, wait_seconds):
                    break

            if _pid_active(record.pid):
                raise ProcessManagerError("owned process group did not stop within the grace periods")
            child = self._children.get((run_id, process_id))
            return_code = child.poll() if child is not None else record.return_code
            stopped = replace(
                record,
                status="STOPPED",
                return_code=return_code,
                ended_at=_utc_now(),
                signals_sent=tuple(signals),
            )
            self._write_record(stopped)
            return stopped

    def stop_all(self, run_id: str, grace: Optional[GracePeriods] = None) -> List[ProcessRecord]:
        return [self.stop(run_id, record.process_id, grace) for record in self.probe(run_id)]

    def read_logs(
        self,
        run_id: str,
        process_id: str,
        stream: str,
        after: int,
        limit: int = 65536,
    ) -> LogChunk:
        if stream not in {"stdout", "stderr"}:
            raise ValueError("stream must be stdout or stderr")
        if isinstance(after, bool) or not isinstance(after, int) or after < 0:
            raise ValueError("after must be a non-negative integer")
        if isinstance(limit, bool) or not isinstance(limit, int) or limit <= 0 or limit > 1048576:
            raise ValueError("limit must be between 1 and 1048576")
        record = self.get(run_id, process_id)
        path = Path(record.stdout_path if stream == "stdout" else record.stderr_path)
        size = path.stat().st_size if path.exists() else 0
        cursor = min(after, size)
        with path.open("rb") as log_stream:
            log_stream.seek(cursor)
            data = log_stream.read(limit)
        next_cursor = cursor + len(data)
        return LogChunk(
            data=data.decode("utf-8", errors="replace"),
            cursor=next_cursor,
            eof=next_cursor >= size and record.status != "RUNNING",
        )

    def _record_path(self, run_id: str, process_id: str) -> Path:
        return self.runs_dir / run_id / "processes" / (process_id + ".json")

    def _write_record(self, record: ProcessRecord) -> None:
        path = Path(record.record_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(".json.tmp")
        temporary.write_text(json.dumps(record.as_dict(), separators=(",", ":"), sort_keys=True), encoding="utf-8")
        temporary.chmod(0o600)
        os.replace(str(temporary), str(path))

    def _refresh(self, record: ProcessRecord) -> ProcessRecord:
        if record.status != "RUNNING":
            return record
        child = self._children.get((record.run_id, record.process_id))
        return_code = child.poll() if child is not None else None
        if return_code is None and _pid_active(record.pid):
            return record
        refreshed = replace(record, status="EXITED", return_code=return_code, ended_at=_utc_now())
        self._write_record(refreshed)
        return refreshed

    def _assert_owned(self, record: ProcessRecord) -> None:
        try:
            actual_pgid = os.getpgid(record.pid)
        except ProcessLookupError as exc:
            raise ProcessOwnershipError("recorded process no longer exists") from exc
        if actual_pgid != record.pgid or record.pid == os.getpid() or record.pgid == os.getpgrp():
            raise ProcessOwnershipError("recorded pid/pgid ownership cannot be proven")

    def _wait_inactive(self, record: ProcessRecord, wait_seconds: float) -> bool:
        deadline = time.monotonic() + wait_seconds
        child = self._children.get((record.run_id, record.process_id))
        while True:
            if child is not None:
                return_code = child.poll()
                if return_code is not None:
                    return True
            if not _pid_active(record.pid):
                return True
            if time.monotonic() >= deadline:
                return False
            time.sleep(min(0.02, max(0.001, wait_seconds / 10.0)))


def _merge_no_proxy(value: str) -> str:
    entries = [entry.strip() for entry in value.split(",") if entry.strip()]
    for required in _LOCAL_NO_PROXY_ENTRIES:
        if required not in entries:
            entries.append(required)
    return ",".join(entries)


def _validate_identifier(name: str, value: str) -> None:
    if not isinstance(value, str) or not _IDENTIFIER.fullmatch(value):
        raise ValueError("%s is not a safe identifier" % name)


def _validate_command(
    agent_kind: str,
    command_ref: str,
    argv: Sequence[str],
    policies: Mapping[str, CommandPolicy],
) -> Tuple[str, ...]:
    policy = policies.get(command_ref)
    if policy is None or policy.agent != agent_kind:
        raise CommandNotAllowed("command_ref is not allowed for this Agent")
    if isinstance(argv, (str, bytes)) or not argv:
        raise CommandNotAllowed("argv must be a non-empty array")
    normalized = tuple(argv)
    if any(not isinstance(value, str) or not value or "\0" in value for value in normalized):
        raise CommandNotAllowed("argv contains an invalid value")
    if normalized[0] != policy.executable:
        raise CommandNotAllowed("argv executable does not match command_ref")
    return normalized


def _validate_environment(environment: Mapping[str, str]) -> Dict[str, str]:
    normalized = {}
    for name, value in environment.items():
        if not isinstance(name, str) or not _ENVIRONMENT_NAME.fullmatch(name):
            raise ValueError("environment name is invalid")
        if not isinstance(value, str) or "\0" in value:
            raise ValueError("environment value is invalid")
        normalized[name] = value
    return normalized


def _record_from_dict(value: Mapping[str, Any]) -> ProcessRecord:
    return ProcessRecord(
        run_id=value["run_id"],
        process_id=value["process_id"],
        command_ref=value["command_ref"],
        argv=tuple(value["argv"]),
        pid=int(value["pid"]),
        pgid=int(value["pgid"]),
        status=value["status"],
        return_code=value.get("return_code"),
        started_at=value["started_at"],
        ended_at=value.get("ended_at"),
        signals_sent=tuple(value.get("signals_sent", [])),
        record_path=value["record_path"],
        stdout_path=value["stdout_path"],
        stderr_path=value["stderr_path"],
    )


def _pid_active(pid: int) -> bool:
    stat_path = Path("/proc") / str(pid) / "stat"
    try:
        state = stat_path.read_text(encoding="utf-8").split()[2]
        return state != "Z"
    except (FileNotFoundError, IndexError, PermissionError):
        return False


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


__all__ = [
    "CommandNotAllowed",
    "CommandPolicy",
    "DEFAULT_COMMAND_POLICIES",
    "GracePeriods",
    "LogChunk",
    "ProcessConflict",
    "ProcessManager",
    "ProcessManagerError",
    "ProcessNotFound",
    "ProcessOwnershipError",
    "ProcessRecord",
]
