"""Single-run orchestration across the signed ROS1 and ROS2 Runtime Agents."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
import os
from pathlib import Path
import shutil
import socket
from threading import Event as ThreadEvent
from threading import Lock, Thread
import time
from typing import Any, Protocol
import uuid

from .agents import AgentHTTPError, AgentUnavailable
from .artifacts import ArtifactStore
from .catalog import CatalogError, ScenarioCatalog
from .commands import CommandInvocation, render_invocation
from .contracts import ScenarioManifest, ScenarioStep
from .displays import display_plan
from .events import EventStore
from .leases import HEARTBEAT_INTERVAL_SECONDS, LeaseConflict, LeaseError, LeaseNotFound, LeaseStore
from .state_machine import ReasonCode, RunNotFound, RunRecord, RunState, RunStateMachine


READINESS_POLL_SECONDS = 0.25


class RuntimeAgent(Protocol):
    def health(self) -> dict[str, Any]: ...

    def start(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]: ...

    def stop(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]: ...

    def probe(self, run_id: str) -> dict[str, Any]: ...

    def probe_readiness(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]: ...

    def logs(
        self,
        run_id: str,
        process_id: str,
        *,
        stream: str = "stdout",
        after: int = 0,
        limit: int = 65536,
    ) -> dict[str, Any]: ...

    def command(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]: ...


class OrchestratorError(RuntimeError):
    pass


class PreflightFailed(OrchestratorError):
    def __init__(self, code: str, message: str, details: Mapping[str, Any], *, status_code: int = 422) -> None:
        self.code = code
        self.details = dict(details)
        self.status_code = status_code
        super().__init__(message)


class RunConflict(OrchestratorError):
    def __init__(self, run_id: str) -> None:
        self.run_id = run_id
        super().__init__("an active run already exists")


class ReadinessTimeout(OrchestratorError):
    pass


class UnexpectedProcessExit(OrchestratorError):
    pass


class CommandNotSupported(OrchestratorError):
    pass


class RunNotActive(OrchestratorError):
    pass


@dataclass(frozen=True, slots=True)
class PreflightCheck:
    name: str
    status: str
    detail: str

    def as_dict(self) -> dict[str, str]:
        return {"name": self.name, "status": self.status, "detail": self.detail}


class PreflightChecker:
    """Read-only checks that must pass before a lease or business process exists."""

    def __init__(
        self,
        data_dir: str | Path,
        agents: Mapping[str, RuntimeAgent],
        *,
        display_check: Callable[[ScenarioManifest], tuple[bool, str]] | None = None,
    ) -> None:
        self.data_dir = Path(data_dir)
        self.agents = dict(agents)
        self.display_check = display_check or (lambda _manifest: (True, "display transport checked by Tasks 5-7"))

    def check(self, manifest: ScenarioManifest) -> tuple[PreflightCheck, ...]:
        checks: list[PreflightCheck] = []
        self.data_dir.mkdir(parents=True, exist_ok=True)
        if not os.access(self.data_dir, os.W_OK):
            raise PreflightFailed(
                "PREFLIGHT_FAILED",
                "run data directory is not writable",
                {"check": "data_directory", "path": str(self.data_dir)},
            )
        checks.append(PreflightCheck("data_directory", "PASS", "run data directory is writable"))

        available = shutil.disk_usage(self.data_dir).free
        if available < manifest.minimum_free_bytes:
            raise PreflightFailed(
                "PREFLIGHT_FAILED",
                "insufficient free disk space",
                {
                    "check": "disk_space",
                    "available_bytes": available,
                    "required_bytes": manifest.minimum_free_bytes,
                },
            )
        checks.append(PreflightCheck("disk_space", "PASS", f"{available} bytes free"))

        agent_kinds = tuple(dict.fromkeys(step.agent for step in manifest.steps))
        for agent_kind in agent_kinds:
            client = self.agents.get(agent_kind)
            if client is None:
                raise PreflightFailed(
                    "AGENT_UNAVAILABLE",
                    f"{agent_kind} Runtime Agent is not configured",
                    {"check": "agent_health", "agent": agent_kind},
                    status_code=503,
                )
            try:
                health = client.health()
            except (AgentUnavailable, AgentHTTPError, OSError) as exc:
                raise PreflightFailed(
                    "AGENT_UNAVAILABLE",
                    f"{agent_kind} Runtime Agent is unavailable",
                    {"check": "agent_health", "agent": agent_kind},
                    status_code=503,
                ) from exc
            if health.get("status") != "ok" or health.get("agent") != agent_kind:
                raise PreflightFailed(
                    "AGENT_UNAVAILABLE",
                    f"{agent_kind} Runtime Agent health contract does not match",
                    {"check": "agent_health", "agent": agent_kind},
                    status_code=503,
                )
            checks.append(PreflightCheck(f"agent_{agent_kind}", "PASS", "signed Runtime Agent is healthy"))

        for step in manifest.steps:
            for probe in step.readiness.all:
                if probe.type != "tcp":
                    continue
                host, port = _host_port(probe.name)
                if not _port_is_free(host, port):
                    raise PreflightFailed(
                        "PREFLIGHT_FAILED",
                        "required runtime port is already occupied",
                        {"check": "port", "address": probe.name},
                        status_code=409,
                    )
                checks.append(PreflightCheck(f"port_{port}", "PASS", "runtime port is available"))

        display_ready, display_detail = self.display_check(manifest)
        if not display_ready:
            raise PreflightFailed(
                "PREFLIGHT_FAILED",
                "display prerequisites are unavailable",
                {"check": "display", "detail": display_detail},
            )
        checks.append(PreflightCheck("display", "PASS", display_detail))
        return tuple(checks)


@dataclass(frozen=True, slots=True)
class RunContext:
    manifest: ScenarioManifest
    parameters: dict[str, Any]
    invocations: Mapping[str, CommandInvocation]


_SCENARIO_COMMANDS: Mapping[str, Mapping[str, tuple[str, str]]] = {
    "swarm.webots.ros2swarm": {
        "mission.start": ("ros2", "ros2.swarm.start"),
        "mission.stop": ("ros2", "ros2.swarm.stop"),
    },
    "control.gazebo.response-deform": {
        "mission.start": ("ros1", "ros1.mission.start"),
        "mission.stop": ("ros1", "ros1.mission.stop"),
        "response.return_home": ("ros1", "ros1.response.return_home"),
    },
    "control.gazebo.response-controller": {},
    "exploration.gazebo.tepper": {
        "mission.start": ("ros1", "ros1.mission.start"),
        "mission.stop": ("ros1", "ros1.mission.stop"),
        "mission.home": ("ros1", "ros1.mission.home"),
    },
    "exploration.webots.robosar": {
        "mission.start": ("ros1", "ros1.mission.start"),
        "mission.stop": ("ros1", "ros1.mission.stop"),
        "mission.home": ("ros1", "ros1.mission.home"),
    },
    "rescue.gazebo.tepper-response": {
        "mission.start": ("ros1", "ros1.mission.start"),
        "mission.stop": ("ros1", "ros1.mission.stop"),
        "mission.home": ("ros1", "ros1.mission.home"),
        "response.return_home": ("ros1", "ros1.response.return_home"),
    },
}


class Orchestrator:
    def __init__(
        self,
        catalog: ScenarioCatalog,
        states: RunStateMachine,
        leases: LeaseStore,
        events: EventStore,
        artifacts: ArtifactStore,
        agents: Mapping[str, RuntimeAgent],
        preflight: PreflightChecker,
        *,
        owner_id: str | None = None,
        readiness_poll_seconds: float = READINESS_POLL_SECONDS,
        max_readiness_attempts: int | None = None,
        sleep: Callable[[float], None] = time.sleep,
        monotonic: Callable[[], float] = time.monotonic,
    ) -> None:
        self.catalog = catalog
        self.states = states
        self.leases = leases
        self.events = events
        self.artifacts = artifacts
        self.agents = dict(agents)
        self.preflight = preflight
        self.owner_id = owner_id or "api-" + uuid.uuid4().hex
        self.readiness_poll_seconds = readiness_poll_seconds
        self.max_readiness_attempts = max_readiness_attempts
        self.sleep = sleep
        self.monotonic = monotonic
        self._threads: dict[str, Thread] = {}
        self._stop_events: dict[str, ThreadEvent] = {}
        self._locks: dict[str, Lock] = {}
        self._registry_lock = Lock()
        self._display_failures: set[tuple[str, str]] = set()

    def create_run(
        self,
        scenario_id: str,
        parameters: Mapping[str, Any] | None,
        recording: Mapping[str, Any] | None = None,
    ) -> RunRecord:
        try:
            manifest = self.catalog.get(scenario_id)
            resolved_parameters = self.catalog.validate_parameters(scenario_id, parameters)
        except CatalogError as exc:
            raise PreflightFailed("INVALID_PARAMETERS", str(exc), {"scenario_id": scenario_id}) from exc
        checks = self.preflight.check(manifest)
        invocations = {
            step.id: render_invocation(step.command_ref, resolved_parameters, manifest=manifest)
            for step in manifest.steps
        }
        request = {
            "scenario_id": scenario_id,
            "parameters": dict(parameters or {}),
            "recording": dict(recording or {"enabled": False, "profile": "default"}),
        }
        resolved = {
            "schema_version": 1,
            "manifest": manifest.model_dump(mode="json"),
            "parameters": resolved_parameters,
            "steps": [
                {
                    "step_id": step.id,
                    "argv": list(invocations[step.id].argv),
                    "environment": dict(invocations[step.id].environment),
                }
                for step in manifest.steps
            ],
            "preflight": [check.as_dict() for check in checks],
            "display_plan": display_plan(manifest).as_dict(),
        }
        run_id = "run-" + uuid.uuid4().hex
        try:
            self.leases.acquire(
                run_id,
                scenario_id,
                self.owner_id,
                request=request,
                resolved_manifest=resolved,
                step_ids=tuple(step.id for step in manifest.steps),
            )
        except LeaseConflict as exc:
            raise RunConflict(exc.active_run_id) from exc
        try:
            self.artifacts.write_json(run_id, "request", "request.json", request)
            self.artifacts.write_json(run_id, "resolved_scenario", "resolved-scenario.json", resolved)
        except BaseException:
            self.states.transition(run_id, RunState.FAILED, reason_code=ReasonCode.PREFLIGHT_FAILED)
            self._release_lease(run_id)
            raise
        return self.states.get(run_id)

    def start_async(self, run_id: str) -> None:
        with self._registry_lock:
            existing = self._threads.get(run_id)
            if existing is not None and existing.is_alive():
                return
            stop_event = self._stop_events.setdefault(run_id, ThreadEvent())
            thread = Thread(target=self._run_worker, args=(run_id, stop_event), daemon=True, name=f"masar-{run_id}")
            self._threads[run_id] = thread
            thread.start()

    def start_run(self, run_id: str) -> RunRecord:
        lock = self._lock_for(run_id)
        with lock:
            current = self.states.get(run_id)
            if current.state is not RunState.PREFLIGHT:
                return current
            context = self._context(current)
            self.states.transition(run_id, RunState.STARTING)
            started: list[str] = []
            try:
                for step in _dependency_order(context.manifest.steps):
                    self._heartbeat(run_id)
                    self._update_step(run_id, step.id, "STARTING", started=True)
                    invocation = context.invocations[step.id]
                    self.agents[step.agent].start(
                        run_id,
                        {
                            "process_id": step.id,
                            "command_ref": step.command_ref,
                            "argv": list(invocation.argv),
                            "environment": dict(invocation.environment),
                        },
                    )
                    started.append(step.id)
                    self._append_event(
                        run_id,
                        "STEP_STARTED",
                        {"step_id": step.id, "agent": step.agent, "command_ref": step.command_ref},
                    )
                    self._wait_ready(run_id, context.manifest, step)
                    self._update_step(run_id, step.id, "READY")
                    self._append_event(run_id, "STEP_READY", {"step_id": step.id, "agent": step.agent})
                self._start_optional_displays(run_id, context.manifest)
            except ReadinessTimeout:
                self._rollback_started(run_id, context, started)
                return self._fail(run_id, ReasonCode.READINESS_TIMEOUT)
            except UnexpectedProcessExit:
                self._rollback_started(run_id, context, started)
                return self._fail(run_id, ReasonCode.UNEXPECTED_PROCESS_EXIT)
            except LeaseError:
                self._rollback_started(run_id, context, started)
                return self._fail(run_id, ReasonCode.NEEDS_INSPECTION)
            except (AgentUnavailable, AgentHTTPError, KeyError, OSError):
                self._rollback_started(run_id, context, started)
                return self._fail(run_id, ReasonCode.PREFLIGHT_FAILED)
            return self.states.transition(run_id, RunState.RUNNING)

    def stop_run(self, run_id: str) -> RunRecord:
        lock = self._lock_for(run_id)
        with lock:
            current = self.states.get(run_id)
            if current.state in {RunState.COMPLETED, RunState.FAILED, RunState.STOPPING}:
                return current
            self._stop_events.setdefault(run_id, ThreadEvent()).set()
            self.states.transition(run_id, RunState.STOPPING, reason_code=ReasonCode.USER_STOP)
            context = self._context(current)
            errors = self._stop_optional_displays(run_id, context.manifest)
            errors.extend(self._stop_steps(run_id, context))
            if errors:
                return self._fail(run_id, ReasonCode.NEEDS_INSPECTION)
            completed = self.states.transition(run_id, RunState.COMPLETED, reason_code=ReasonCode.USER_STOP)
            self._release_lease(run_id)
            return completed

    def issue_command(self, run_id: str, command: str) -> dict[str, Any]:
        run = self.states.get(run_id)
        if run.state is not RunState.RUNNING:
            raise RunNotActive("mission commands require a RUNNING run")
        mapping = _SCENARIO_COMMANDS.get(run.scenario_id, {})
        definition = mapping.get(command)
        if definition is None:
            raise CommandNotSupported(f"{command} is not supported by {run.scenario_id}")
        agent_kind, action = definition
        context = self._context(run)
        result = self.agents[agent_kind].command(
            run_id,
            {"action": action, "runtime": context.manifest.runtime.model_dump(mode="json")},
        )
        self._append_event(
            run_id,
            "COMMAND_PUBLISHED",
            {"command": command, "agent": agent_kind, "action": action},
        )
        return result

    def logs(
        self,
        run_id: str,
        process_id: str,
        *,
        stream: str = "stdout",
        after: int = 0,
        limit: int = 65536,
    ) -> dict[str, Any]:
        run = self.states.get(run_id)
        context = self._context(run)
        step = next((item for item in context.manifest.steps if item.id == process_id), None)
        agent_kind = step.agent if step is not None else None
        rviz = display_plan(context.manifest).rviz
        if agent_kind is None and rviz is not None and process_id == rviz.process_id:
            agent_kind = rviz.agent
        if agent_kind is None:
            raise RunNotFound(process_id)
        return self.agents[agent_kind].logs(
            run_id,
            process_id,
            stream=stream,
            after=after,
            limit=limit,
        )

    def recover_active_runs(self) -> tuple[RunRecord, ...]:
        recovered: list[RunRecord] = []
        for run in self.states.list_active():
            context = self._context(run)
            ownership_proven = run.state is RunState.RUNNING
            for agent_kind in tuple(dict.fromkeys(step.agent for step in context.manifest.steps)):
                try:
                    response = self.agents[agent_kind].probe(run.run_id)
                except (AgentUnavailable, AgentHTTPError, OSError):
                    ownership_proven = False
                    break
                expected = {step.id for step in context.manifest.steps if step.agent == agent_kind}
                running = {
                    record.get("process_id")
                    for record in response.get("processes", [])
                    if record.get("run_id") == run.run_id
                    and record.get("status") == "RUNNING"
                    and isinstance(record.get("pgid"), int)
                }
                if not expected <= running:
                    ownership_proven = False
                    break
            if ownership_proven:
                self.leases.adopt(run.run_id, self.owner_id)
                self.start_monitor(run.run_id)
                recovered.append(self.states.get(run.run_id))
                continue
            failed = self.states.transition(run.run_id, RunState.FAILED, reason_code=ReasonCode.NEEDS_INSPECTION)
            try:
                lease = self.leases.get(run.run_id)
                self.leases.release(run.run_id, lease.owner_id)
            except LeaseNotFound:
                pass
            recovered.append(failed)
        return tuple(recovered)

    def start_monitor(self, run_id: str) -> None:
        with self._registry_lock:
            thread = self._threads.get(run_id)
            if thread is not None and thread.is_alive():
                return
            stop_event = self._stop_events.setdefault(run_id, ThreadEvent())
            thread = Thread(target=self._monitor, args=(run_id, stop_event), daemon=True, name=f"monitor-{run_id}")
            self._threads[run_id] = thread
            thread.start()

    def _run_worker(self, run_id: str, stop_event: ThreadEvent) -> None:
        result = self.start_run(run_id)
        if result.state is RunState.RUNNING and not stop_event.is_set():
            self._monitor(run_id, stop_event)

    def _monitor(self, run_id: str, stop_event: ThreadEvent) -> None:
        while not stop_event.wait(HEARTBEAT_INTERVAL_SECONDS):
            try:
                run = self.states.get(run_id)
                if run.state is not RunState.RUNNING:
                    return
                self._heartbeat(run_id)
                context = self._context(run)
                for agent_kind in tuple(dict.fromkeys(step.agent for step in context.manifest.steps)):
                    response = self.agents[agent_kind].probe(run_id)
                    records = {
                        record.get("process_id"): record
                        for record in response.get("processes", [])
                        if isinstance(record.get("process_id"), str)
                    }
                    for step in context.manifest.steps:
                        if step.agent != agent_kind:
                            continue
                        record = records.get(step.id)
                        if record is None or record.get("status") != "RUNNING":
                            raise UnexpectedProcessExit(step.id)
                    rviz = display_plan(context.manifest).rviz
                    if rviz is not None and rviz.agent == agent_kind:
                        record = records.get(rviz.process_id)
                        if record is None or record.get("status") != "RUNNING":
                            self._record_display_error(run_id, rviz.process_id, record)
            except UnexpectedProcessExit:
                lock = self._lock_for(run_id)
                with lock:
                    current = self.states.get(run_id)
                    if current.state is RunState.RUNNING:
                        context = self._context(current)
                        self._stop_optional_displays(run_id, context.manifest)
                        self._stop_steps(run_id, context)
                        self._fail(run_id, ReasonCode.UNEXPECTED_PROCESS_EXIT)
                return
            except (AgentUnavailable, AgentHTTPError, LeaseError, OSError):
                lock = self._lock_for(run_id)
                with lock:
                    current = self.states.get(run_id)
                    if current.state is RunState.RUNNING:
                        self._fail(run_id, ReasonCode.NEEDS_INSPECTION)
                return

    def _wait_ready(self, run_id: str, manifest: ScenarioManifest, step: ScenarioStep) -> None:
        deadline = self.monotonic() + step.start_timeout_seconds
        attempts = 0
        next_heartbeat = self.monotonic() + HEARTBEAT_INTERVAL_SECONDS
        payload = {
            "runtime": manifest.runtime.model_dump(mode="json"),
            "probes": [probe.model_dump(mode="json") for probe in step.readiness.all],
        }
        while True:
            response = self.agents[step.agent].probe_readiness(run_id, payload)
            process = next(
                (record for record in response.get("processes", []) if record.get("process_id") == step.id),
                None,
            )
            if process is not None and process.get("status") != "RUNNING":
                raise UnexpectedProcessExit(step.id)
            required = [probe for probe in response.get("probes", []) if probe.get("required", True)]
            if all(probe.get("ready") is True for probe in required):
                return
            attempts += 1
            now = self.monotonic()
            if (self.max_readiness_attempts is not None and attempts >= self.max_readiness_attempts) or now >= deadline:
                raise ReadinessTimeout(step.id)
            if now >= next_heartbeat:
                self._heartbeat(run_id)
                next_heartbeat = now + HEARTBEAT_INTERVAL_SECONDS
            self.sleep(self.readiness_poll_seconds)

    def _rollback_started(self, run_id: str, context: RunContext, started: Sequence[str]) -> None:
        started_set = set(started)
        for step_id in context.manifest.stop_order:
            if step_id not in started_set:
                continue
            step = next(item for item in context.manifest.steps if item.id == step_id)
            try:
                self._stop_step(run_id, step)
            except (AgentUnavailable, AgentHTTPError, OSError):
                self._update_step(run_id, step.id, "STOP_FAILED", detail="rollback stop failed", ended=True)

    def _start_optional_displays(self, run_id: str, manifest: ScenarioManifest) -> None:
        rviz = display_plan(manifest).rviz
        if rviz is None:
            return
        try:
            self.agents[rviz.agent].start(
                run_id,
                {
                    "process_id": rviz.process_id,
                    "command_ref": rviz.command_ref,
                    "argv": list(rviz.argv),
                    "environment": dict(rviz.environment),
                },
            )
        except (AgentUnavailable, AgentHTTPError, OSError) as exc:
            self._record_display_error(
                run_id,
                rviz.process_id,
                {"status": "START_FAILED", "error": type(exc).__name__},
            )
            return
        self._append_event(
            run_id,
            "DISPLAY_STARTED",
            {"panel": "rviz", "process_id": rviz.process_id, "display": rviz.display},
        )

    def _stop_optional_displays(self, run_id: str, manifest: ScenarioManifest) -> list[str]:
        rviz = display_plan(manifest).rviz
        if rviz is None:
            return []
        try:
            response = self.agents[rviz.agent].probe(run_id)
            record = next(
                (item for item in response.get("processes", []) if item.get("process_id") == rviz.process_id),
                None,
            )
            if record is None or record.get("status") != "RUNNING":
                return []
            self.agents[rviz.agent].stop(
                run_id,
                {
                    "process_id": rviz.process_id,
                    "grace_seconds": {"sigint": 10, "sigterm": 5, "sigkill": 5},
                },
            )
            self._append_event(run_id, "DISPLAY_STOPPED", {"panel": "rviz", "process_id": rviz.process_id})
            return []
        except (AgentUnavailable, AgentHTTPError, OSError) as exc:
            return [f"{rviz.process_id}:{type(exc).__name__}"]

    def _record_display_error(
        self,
        run_id: str,
        process_id: str,
        record: Mapping[str, Any] | None,
    ) -> None:
        key = (run_id, process_id)
        if key in self._display_failures:
            return
        self._display_failures.add(key)
        self._append_event(
            run_id,
            "DISPLAY_ERROR",
            {
                "panel": "rviz",
                "process_id": process_id,
                "status": None if record is None else record.get("status"),
            },
            severity="warning",
        )

    def _stop_steps(self, run_id: str, context: RunContext) -> list[str]:
        errors: list[str] = []
        states = self._step_states(run_id)
        for step_id in context.manifest.stop_order:
            if states.get(step_id) == "PENDING":
                continue
            step = next(item for item in context.manifest.steps if item.id == step_id)
            try:
                self._stop_step(run_id, step)
            except (AgentUnavailable, AgentHTTPError, OSError) as exc:
                errors.append(f"{step.id}:{type(exc).__name__}")
                self._update_step(run_id, step.id, "STOP_FAILED", detail="Agent stop failed", ended=True)
        return errors

    def _stop_step(self, run_id: str, step: ScenarioStep) -> None:
        grace = step.grace_seconds.model_dump(mode="json")
        self.agents[step.agent].stop(run_id, {"process_id": step.id, "grace_seconds": grace})
        self._update_step(run_id, step.id, "STOPPED", ended=True)
        self._append_event(run_id, "STEP_STOPPED", {"step_id": step.id, "agent": step.agent})

    def _fail(self, run_id: str, reason: ReasonCode) -> RunRecord:
        current = self.states.get(run_id)
        if current.state is RunState.FAILED:
            return current
        failed = self.states.transition(run_id, RunState.FAILED, reason_code=reason)
        self._release_lease(run_id)
        return failed

    def _heartbeat(self, run_id: str) -> None:
        self.leases.heartbeat(run_id, self.owner_id)

    def _release_lease(self, run_id: str) -> None:
        try:
            self.leases.release(run_id, self.owner_id)
        except LeaseError:
            pass

    def _context(self, run: RunRecord) -> RunContext:
        payload = run.resolved_manifest
        manifest = ScenarioManifest.model_validate(payload["manifest"])
        parameters = dict(payload["parameters"])
        step_payloads = {item["step_id"]: item for item in payload["steps"]}
        invocations = {
            step.id: CommandInvocation(
                argv=tuple(step_payloads[step.id]["argv"]),
                environment=step_payloads[step.id]["environment"],
            )
            for step in manifest.steps
        }
        return RunContext(manifest, parameters, invocations)

    def _append_event(self, run_id: str, kind: str, payload: Mapping[str, Any], severity: str = "info") -> None:
        self.events.append(
            run_id,
            source="orchestrator",
            kind=kind,
            severity=severity,
            payload=payload,
            wall_time=datetime.now(timezone.utc),
        )

    def _update_step(
        self,
        run_id: str,
        step_id: str,
        state: str,
        *,
        detail: str | None = None,
        started: bool = False,
        ended: bool = False,
    ) -> None:
        now = datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")
        assignments = ["state = ?", "detail = ?"]
        values: list[Any] = [state, detail]
        if started:
            assignments.append("started_at = COALESCE(started_at, ?)")
            values.append(now)
        if ended:
            assignments.append("ended_at = ?")
            values.append(now)
        values.extend((run_id, step_id))
        with self.states.database.transaction(immediate=True) as connection:
            connection.execute(
                f"UPDATE run_steps SET {', '.join(assignments)} WHERE run_id = ? AND step_id = ?",  # noqa: S608
                values,
            )

    def _step_states(self, run_id: str) -> dict[str, str]:
        with self.states.database.connect() as connection:
            rows = connection.execute(
                "SELECT step_id, state FROM run_steps WHERE run_id = ?",
                (run_id,),
            ).fetchall()
        return {row["step_id"]: row["state"] for row in rows}

    def _lock_for(self, run_id: str) -> Lock:
        with self._registry_lock:
            return self._locks.setdefault(run_id, Lock())


def _dependency_order(steps: Sequence[ScenarioStep]) -> tuple[ScenarioStep, ...]:
    remaining = {step.id: step for step in steps}
    completed: set[str] = set()
    ordered: list[ScenarioStep] = []
    while remaining:
        ready = [step for step in steps if step.id in remaining and set(step.depends_on) <= completed]
        if not ready:
            raise ValueError("scenario dependency graph cannot be ordered")
        for step in ready:
            ordered.append(step)
            completed.add(step.id)
            remaining.pop(step.id)
    return tuple(ordered)


def _host_port(value: str) -> tuple[str, int]:
    host, separator, port_text = value.rpartition(":")
    if not separator or not host or not port_text.isdigit():
        raise ValueError("TCP readiness name must be host:port")
    return host, int(port_text)


def _port_is_free(host: str, port: int) -> bool:
    family = socket.AF_INET6 if ":" in host else socket.AF_INET
    probe = socket.socket(family, socket.SOCK_STREAM)
    try:
        probe.bind((host, port))
    except OSError:
        return False
    finally:
        probe.close()
    return True


__all__ = [
    "CommandNotSupported",
    "Orchestrator",
    "OrchestratorError",
    "PreflightCheck",
    "PreflightChecker",
    "PreflightFailed",
    "ReadinessTimeout",
    "RunConflict",
    "RunNotActive",
    "RuntimeAgent",
    "UnexpectedProcessExit",
]
