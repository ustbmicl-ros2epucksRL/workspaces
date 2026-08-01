"""Single-run orchestration across the signed ROS1 and ROS2 Runtime Agents."""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import logging
import os
from pathlib import Path
import shutil
import socket
from threading import Event as ThreadEvent
from threading import Lock, Thread
import time
from typing import Any, Protocol
import uuid
import yaml

from . import __version__
from .agents import AgentHTTPError, AgentUnavailable
from .artifacts import Artifact, ArtifactStore
from .catalog import CatalogError, ScenarioCatalog
from .commands import CommandInvocation, render_invocation
from .contracts import ScenarioManifest, ScenarioStep
from .displays import display_plan
from .events import EventStore
from .leases import HEARTBEAT_INTERVAL_SECONDS, LeaseConflict, LeaseError, LeaseNotFound, LeaseStore
from .recording import RecorderPlan, build_recorder_plans, normalize_recording
from .db import datetime_to_text
from .state_machine import ReasonCode, RunNotFound, RunRecord, RunState, RunStateMachine


READINESS_POLL_SECONDS = 0.25
_LOGGER = logging.getLogger(__name__)


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

    def graph_snapshot(self, run_id: str, payload: Mapping[str, Any]) -> dict[str, Any]: ...


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
    recording: dict[str, Any]
    recorders: tuple[RecorderPlan, ...]


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
            normalized_recording = normalize_recording(recording)
            build_recorder_plans(manifest, normalized_recording, self.artifacts.run_dir("validation"))
        except (CatalogError, ValueError, KeyError) as exc:
            raise PreflightFailed("INVALID_PARAMETERS", str(exc), {"scenario_id": scenario_id}) from exc
        checks = self.preflight.check(manifest)
        invocations = {
            step.id: render_invocation(step.command_ref, resolved_parameters, manifest=manifest)
            for step in manifest.steps
        }
        request = {
            "scenario_id": scenario_id,
            "parameters": dict(parameters or {}),
            "recording": normalized_recording,
        }
        resolved_display = display_plan(manifest)
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
            "display_plan": resolved_display.as_dict(),
            "recording": normalized_recording,
        }
        run_id = "run-" + uuid.uuid4().hex
        try:
            self.leases.acquire(
                run_id,
                scenario_id,
                self.owner_id,
                request=request,
                resolved_manifest=resolved,
                step_ids=tuple(step.id for step in manifest.steps)
                + ((resolved_display.rviz.process_id,) if resolved_display.rviz is not None else ()),
            )
        except LeaseConflict as exc:
            raise RunConflict(exc.active_run_id) from exc
        try:
            self.artifacts.write_json(run_id, "request", "request.json", request)
            self.artifacts.write_json(run_id, "resolved_scenario", "resolved-scenario.json", resolved)
        except BaseException:
            failed = self.states.transition(run_id, RunState.FAILED, reason_code=ReasonCode.PREFLIGHT_FAILED)
            self._release_lease(run_id)
            self._safe_finalize(failed)
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
            attempted_recorders: list[str] = []
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
                self._start_recorders(run_id, context, attempted_recorders)
            except ReadinessTimeout:
                return self._rollback_start_failure(
                    run_id, context, started, attempted_recorders, ReasonCode.READINESS_TIMEOUT
                )
            except UnexpectedProcessExit:
                return self._rollback_start_failure(
                    run_id, context, started, attempted_recorders, ReasonCode.UNEXPECTED_PROCESS_EXIT
                )
            except LeaseError:
                return self._rollback_start_failure(
                    run_id, context, started, attempted_recorders, ReasonCode.NEEDS_INSPECTION
                )
            except (AgentUnavailable, AgentHTTPError, KeyError, OSError, ValueError):
                return self._rollback_start_failure(
                    run_id, context, started, attempted_recorders, ReasonCode.PREFLIGHT_FAILED
                )
            return self.states.transition(run_id, RunState.RUNNING)

    def stop_run(self, run_id: str) -> RunRecord:
        lock = self._lock_for(run_id)
        with lock:
            current = self.states.get(run_id)
            if current.state in {RunState.COMPLETED, RunState.FAILED, RunState.STOPPING}:
                if current.state in {RunState.COMPLETED, RunState.FAILED}:
                    self._safe_finalize(current)
                return current
            self._stop_events.setdefault(run_id, ThreadEvent()).set()
            self.states.transition(run_id, RunState.STOPPING, reason_code=ReasonCode.USER_STOP)
            context = self._context(current)
            errors = self._stop_recorders(run_id, context)
            self._capture_graphs(run_id, context)
            errors.extend(self._stop_optional_displays(run_id, context.manifest))
            errors.extend(self._stop_steps(run_id, context))
            if errors:
                return self._fail(
                    run_id,
                    ReasonCode.NEEDS_INSPECTION,
                    extra_missing=self._shutdown_conditions(errors),
                )
            completed = self.states.transition(run_id, RunState.COMPLETED, reason_code=ReasonCode.USER_STOP)
            self._release_lease(run_id)
            self._safe_finalize(completed)
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
            for agent_kind in self._agent_kinds(context):
                try:
                    response = self.agents[agent_kind].probe(run.run_id)
                except (AgentUnavailable, AgentHTTPError, OSError):
                    ownership_proven = False
                    break
                self._backfill_recovered_display_state(run.run_id, context.manifest, agent_kind, response)
                expected = {step.id for step in context.manifest.steps if step.agent == agent_kind}
                expected.update(plan.process_id for plan in context.recorders if plan.agent == agent_kind)
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
            self._safe_finalize(failed, extra_missing=("ownership_unproven",))
            recovered.append(failed)
        return tuple(recovered)

    def finalize_incomplete_runs(self) -> None:
        for run in self.states.list(limit=1000):
            if run.state in {RunState.COMPLETED, RunState.FAILED} and not self._archive_is_complete(run.run_id):
                self._safe_finalize(run, extra_missing=("recovered_archive",))

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
                for agent_kind in self._agent_kinds(context):
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
                    for recorder in context.recorders:
                        if recorder.agent != agent_kind:
                            continue
                        record = records.get(recorder.process_id)
                        if record is None or record.get("status") != "RUNNING":
                            raise UnexpectedProcessExit(recorder.process_id)
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
                        errors = self._stop_recorders(run_id, context)
                        self._capture_graphs(run_id, context)
                        errors.extend(self._stop_optional_displays(run_id, context.manifest))
                        errors.extend(self._stop_steps(run_id, context))
                        self._fail(
                            run_id,
                            ReasonCode.UNEXPECTED_PROCESS_EXIT,
                            extra_missing=self._shutdown_conditions(errors),
                        )
                return
            except (AgentUnavailable, AgentHTTPError, LeaseError, OSError):
                lock = self._lock_for(run_id)
                with lock:
                    current = self.states.get(run_id)
                    if current.state is RunState.RUNNING:
                        context = self._context(current)
                        errors = self._stop_recorders(run_id, context)
                        self._capture_graphs(run_id, context)
                        errors.extend(self._stop_optional_displays(run_id, context.manifest))
                        errors.extend(self._stop_steps(run_id, context))
                        self._fail(
                            run_id,
                            ReasonCode.NEEDS_INSPECTION,
                            extra_missing=self._shutdown_conditions(errors),
                        )
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

    def _rollback_start_failure(
        self,
        run_id: str,
        context: RunContext,
        started: Sequence[str],
        attempted_recorders: Sequence[str],
        reason: ReasonCode,
    ) -> RunRecord:
        errors = self._stop_recorders(run_id, context, process_ids=set(attempted_recorders))
        self._capture_graphs(run_id, context)
        errors.extend(self._stop_optional_displays(run_id, context.manifest))
        errors.extend(self._rollback_started(run_id, context, started))
        return self._fail(run_id, reason, extra_missing=self._shutdown_conditions(errors))

    def _rollback_started(self, run_id: str, context: RunContext, started: Sequence[str]) -> list[str]:
        errors: list[str] = []
        started_set = set(started)
        for step_id in context.manifest.stop_order:
            if step_id not in started_set:
                continue
            step = next(item for item in context.manifest.steps if item.id == step_id)
            try:
                self._stop_step(run_id, step)
            except (AgentUnavailable, AgentHTTPError, OSError) as exc:
                errors.append(f"{step.id}:{type(exc).__name__}")
                self._update_step(run_id, step.id, "STOP_FAILED", detail="rollback stop failed", ended=True)
        return errors

    def _start_recorders(
        self,
        run_id: str,
        context: RunContext,
        attempted: list[str],
    ) -> None:
        for plan in context.recorders:
            plan.output_path.parent.mkdir(parents=True, exist_ok=True)
            attempted.append(plan.process_id)
            self.agents[plan.agent].start(
                run_id,
                {
                    "process_id": plan.process_id,
                    "command_ref": plan.command_ref,
                    "argv": list(plan.argv),
                    "environment": dict(plan.environment),
                },
            )
            self._append_event(
                run_id,
                "RECORDER_STARTED",
                {"agent": plan.agent, "process_id": plan.process_id, "profile": context.recording["profile"]},
            )

    def _stop_recorders(
        self,
        run_id: str,
        context: RunContext,
        *,
        process_ids: set[str] | None = None,
    ) -> list[str]:
        errors: list[str] = []
        for plan in reversed(context.recorders):
            if process_ids is not None and plan.process_id not in process_ids:
                continue
            try:
                response = self.agents[plan.agent].probe(run_id)
                record = next(
                    (item for item in response.get("processes", []) if item.get("process_id") == plan.process_id),
                    None,
                )
                if record is None:
                    errors.append(f"{plan.process_id}:ownership_missing")
                    continue
                status = record.get("status")
                if status == "RUNNING":
                    stopped = self.agents[plan.agent].stop(
                        run_id,
                        {
                            "process_id": plan.process_id,
                            "grace_seconds": {"sigint": 10, "sigterm": 5, "sigkill": 5},
                        },
                    )
                    if stopped.get("status") not in {"STOPPED", "EXITED"}:
                        errors.append(f"{plan.process_id}:stop_unconfirmed")
                        continue
                    self._append_event(
                        run_id,
                        "RECORDER_STOPPED",
                        {"agent": plan.agent, "process_id": plan.process_id},
                    )
                elif status not in {"STOPPED", "EXITED"}:
                    errors.append(f"{plan.process_id}:status_{status or 'unknown'}")
            except (AgentUnavailable, AgentHTTPError, OSError) as exc:
                errors.append(f"{plan.process_id}:{type(exc).__name__}")
        return errors

    def _start_optional_displays(self, run_id: str, manifest: ScenarioManifest) -> None:
        rviz = display_plan(manifest).rviz
        if rviz is None:
            return
        self._update_step(run_id, rviz.process_id, "STARTING", started=True)
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
        except AgentHTTPError as exc:
            state = "START_REJECTED" if exc.status < 500 and exc.status != 409 else "START_UNCONFIRMED"
            self._update_step(
                run_id,
                rviz.process_id,
                state,
                detail=f"display start failed: {type(exc).__name__}",
                ended=state == "START_REJECTED",
            )
            self._record_display_error(
                run_id,
                rviz.process_id,
                {"status": "START_FAILED", "error": type(exc).__name__},
            )
            return
        except (AgentUnavailable, OSError) as exc:
            self._update_step(
                run_id,
                rviz.process_id,
                "START_UNCONFIRMED",
                detail=f"display start failed: {type(exc).__name__}",
            )
            self._record_display_error(
                run_id,
                rviz.process_id,
                {"status": "START_FAILED", "error": type(exc).__name__},
            )
            return
        self._update_step(run_id, rviz.process_id, "READY")
        self._append_event(
            run_id,
            "DISPLAY_STARTED",
            {"panel": "rviz", "process_id": rviz.process_id, "display": rviz.display},
        )

    def _stop_optional_displays(self, run_id: str, manifest: ScenarioManifest) -> list[str]:
        rviz = display_plan(manifest).rviz
        if rviz is None:
            return []
        display_state = self._step_states(run_id).get(rviz.process_id, "PENDING")
        if display_state in {"PENDING", "START_REJECTED"}:
            return []
        try:
            ownership_expected = display_state in {"STARTING", "READY", "START_UNCONFIRMED"}
            response = self.agents[rviz.agent].probe(run_id)
            record = next(
                (item for item in response.get("processes", []) if item.get("process_id") == rviz.process_id),
                None,
            )
            if record is None:
                if not ownership_expected:
                    return []
                self._update_step(
                    run_id,
                    rviz.process_id,
                    "STOP_FAILED",
                    detail="display ownership record is missing",
                    ended=True,
                )
                return [f"{rviz.process_id}:ownership_missing"]
            status = record.get("status")
            if status in {"STOPPED", "EXITED"}:
                self._update_step(run_id, rviz.process_id, "STOPPED", ended=True)
                return []
            if status != "RUNNING":
                self._update_step(
                    run_id,
                    rviz.process_id,
                    "STOP_FAILED",
                    detail=f"unexpected display status: {status or 'unknown'}",
                    ended=True,
                )
                return [f"{rviz.process_id}:status_{status or 'unknown'}"]
            stopped = self.agents[rviz.agent].stop(
                run_id,
                {
                    "process_id": rviz.process_id,
                    "grace_seconds": {"sigint": 10, "sigterm": 5, "sigkill": 5},
                },
            )
            if stopped.get("status") not in {"STOPPED", "EXITED"}:
                self._update_step(
                    run_id,
                    rviz.process_id,
                    "STOP_FAILED",
                    detail="display stop was not confirmed",
                    ended=True,
                )
                return [f"{rviz.process_id}:stop_unconfirmed"]
            self._update_step(run_id, rviz.process_id, "STOPPED", ended=True)
            self._append_event(run_id, "DISPLAY_STOPPED", {"panel": "rviz", "process_id": rviz.process_id})
            return []
        except (AgentUnavailable, AgentHTTPError, OSError) as exc:
            self._update_step(
                run_id,
                rviz.process_id,
                "STOP_FAILED",
                detail=f"display stop failed: {type(exc).__name__}",
                ended=True,
            )
            return [f"{rviz.process_id}:{type(exc).__name__}"]

    def _backfill_recovered_display_state(
        self,
        run_id: str,
        manifest: ScenarioManifest,
        agent_kind: str,
        response: Mapping[str, Any],
    ) -> None:
        rviz = display_plan(manifest).rviz
        if rviz is None or rviz.agent != agent_kind or rviz.process_id in self._step_states(run_id):
            return
        record = next(
            (
                item
                for item in response.get("processes", [])
                if item.get("run_id") == run_id and item.get("process_id") == rviz.process_id
            ),
            None,
        )
        status = None if record is None else record.get("status")
        if status == "RUNNING" and isinstance(record.get("pgid"), int):
            state = "READY"
        elif status in {"STOPPED", "EXITED"}:
            state = "STOPPED"
        else:
            state = "START_UNCONFIRMED"
        now = datetime_to_text(datetime.now(timezone.utc))
        with self.states.database.transaction(immediate=True) as connection:
            connection.execute(
                """
                INSERT INTO run_steps (run_id, step_id, state, started_at, ended_at, detail)
                VALUES (?, ?, ?, ?, ?, ?)
                ON CONFLICT(run_id, step_id) DO NOTHING
                """,
                (
                    run_id,
                    rviz.process_id,
                    state,
                    now,
                    now if state == "STOPPED" else None,
                    "display state recovered during platform upgrade",
                ),
            )

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

    def _fail(
        self,
        run_id: str,
        reason: ReasonCode,
        *,
        extra_missing: Sequence[str] = (),
    ) -> RunRecord:
        current = self.states.get(run_id)
        if current.state is RunState.FAILED:
            self._safe_finalize(current, extra_missing=extra_missing)
            return current
        failed = self.states.transition(run_id, RunState.FAILED, reason_code=reason)
        self._release_lease(run_id)
        self._safe_finalize(failed, extra_missing=extra_missing)
        return failed

    def _heartbeat(self, run_id: str) -> None:
        self.leases.heartbeat(run_id, self.owner_id)

    def _release_lease(self, run_id: str) -> None:
        try:
            self.leases.release(run_id, self.owner_id)
        except LeaseError:
            pass

    def _safe_finalize(self, run: RunRecord, *, extra_missing: Sequence[str] = ()) -> None:
        try:
            self._persist_archive_conditions(run.run_id, extra_missing)
            self._finalize(run, extra_missing=self._archive_conditions(run.run_id))
        except Exception as exc:
            _LOGGER.exception("run archive finalization failed for %s", run.run_id)
            try:
                self._append_event(
                    run.run_id,
                    "ARCHIVE_FAILED",
                    {"error_type": type(exc).__name__},
                    severity="error",
                )
            except Exception:
                _LOGGER.exception("failed to persist archive failure event for %s", run.run_id)

    def _finalize(self, run: RunRecord, *, extra_missing: Sequence[str] = ()) -> None:
        run_dir = self.artifacts.run_dir(run.run_id)
        if self._archive_is_complete(run.run_id):
            return
        context = self._context(run)
        missing_data: list[str] = sorted(set(extra_missing) | set(self._archive_conditions(run.run_id)))

        resolved_yaml = yaml.safe_dump(run.resolved_manifest, sort_keys=False, allow_unicode=True).encode("utf-8")
        resolved_yaml_path = run_dir / "resolved-scenario.yaml"
        _atomic_bytes(resolved_yaml_path, resolved_yaml)
        self.artifacts.register(run.run_id, "resolved_scenario", resolved_yaml_path)

        events_path = self.events.prepare_archive(run.run_id)
        self.artifacts.register(run.run_id, "events", events_path)

        process_descriptors = [
            (step.agent, step.id, f"{step.agent}_{step.id}") for step in context.manifest.steps
        ]
        rviz = display_plan(context.manifest).rviz
        if rviz is not None:
            process_descriptors.append((rviz.agent, rviz.process_id, f"{rviz.agent}_{rviz.process_id}"))
        process_descriptors.extend(
            (plan.agent, plan.process_id, f"{plan.agent}_{plan.process_id}") for plan in context.recorders
        )
        for agent_kind, process_id, archive_stem in process_descriptors:
            for stream in ("stdout", "stderr"):
                path = run_dir / "process-logs" / f"{archive_stem}.{stream}.log"
                try:
                    self._archive_log(run.run_id, agent_kind, process_id, stream, path)
                    self.artifacts.register(run.run_id, "process_log", path)
                except (AgentUnavailable, AgentHTTPError, OSError, ValueError, KeyError, TypeError):
                    missing_data.append(f"process_log:{agent_kind}:{process_id}:{stream}")

        graph_snapshots: dict[str, dict[str, Any]] = {}
        agent_versions: dict[str, Any] = {}
        for agent_kind in self._agent_kinds(context):
            try:
                health = self.agents[agent_kind].health()
                agent_versions[agent_kind] = health.get("protocol_version")
            except (AgentUnavailable, AgentHTTPError, OSError, ValueError, KeyError, TypeError):
                agent_versions[agent_kind] = None
                missing_data.append(f"agent_version:{agent_kind}")
            graph_path = run_dir / "ros" / f"{agent_kind}-graph.json"
            try:
                snapshot = json.loads(graph_path.read_text(encoding="utf-8"))
                if not isinstance(snapshot, dict):
                    raise ValueError("graph snapshot must be an object")
            except (FileNotFoundError, OSError, ValueError, TypeError, json.JSONDecodeError):
                snapshot = self._capture_graph(run.run_id, context, agent_kind)
            graph_snapshots[agent_kind] = snapshot
            if snapshot.get("nodes_available") is not True:
                missing_data.append(f"ros_graph:{agent_kind}:nodes")
            if snapshot.get("topics_available") is not True:
                missing_data.append(f"ros_graph:{agent_kind}:topics")
            self.artifacts.register(run.run_id, "ros_graph", graph_path)

        topics_snapshot = {
            "topics": [
                {"agent": agent_kind, **topic}
                for agent_kind, snapshot in graph_snapshots.items()
                for topic in snapshot.get("topics", [])
                if isinstance(topic, dict)
            ]
        }
        nodes_snapshot = {
            "nodes": [
                {"agent": agent_kind, "name": node}
                for agent_kind, snapshot in graph_snapshots.items()
                for node in snapshot.get("nodes", [])
                if isinstance(node, str)
            ]
        }
        topics_path = run_dir / "ros" / "topics-snapshot.json"
        nodes_path = run_dir / "ros" / "nodes-snapshot.json"
        _atomic_json(topics_path, topics_snapshot)
        _atomic_json(nodes_path, nodes_snapshot)
        self.artifacts.register(run.run_id, "ros_topics", topics_path)
        self.artifacts.register(run.run_id, "ros_nodes", nodes_path)

        if {"ownership_unproven", "shutdown_unconfirmed"} & set(missing_data):
            bag_artifacts: list[Artifact] = []
            missing_data.extend(f"bag_unsettled:{plan.agent}" for plan in context.recorders)
        else:
            bag_artifacts = self._index_bags(run.run_id, context, missing_data)
        step_results = self._step_results(run.run_id)
        ended_at = run.ended_at or run.updated_at
        duration_seconds = max(0.0, (ended_at - run.created_at).total_seconds())
        summary = {
            "run_id": run.run_id,
            "duration_seconds": duration_seconds,
            "final_state": run.state.value,
            "missing_data": sorted(set(missing_data)),
            "topic_health": [],
            "task_counts": {},
            "exploration_progress_final": None,
            "response_group": None,
        }
        self.artifacts.write_json(run.run_id, "summary", "summary.json", summary)

        indexed = tuple(
            artifact
            for artifact in self.artifacts.list(run.run_id)
            if artifact.path.name not in {"manifest.json", "checksums.sha256"}
        )
        recorder_artifact_ids = {
            plan.process_id: [
                artifact.artifact_id
                for artifact in bag_artifacts
                if artifact.path.is_relative_to(plan.output_path)
            ]
            for plan in context.recorders
        }
        recording_manifest = {
            "enabled": context.recording["enabled"],
            "profile": context.recording["profile"],
            "recorders": [
                {
                    "agent": plan.agent,
                    "process_id": plan.process_id,
                    "maximum_bytes_per_second": plan.maximum_bytes_per_second,
                    "artifact_ids": recorder_artifact_ids[plan.process_id],
                }
                for plan in context.recorders
            ],
        }
        manifest = {
            "run_id": run.run_id,
            "scenario_id": run.scenario_id,
            "parameters": context.parameters,
            "platform_version": __version__,
            "agent_versions": agent_versions,
            "started_at": datetime_to_text(run.created_at),
            "ended_at": datetime_to_text(ended_at),
            "final_state": run.state.value,
            "failure_reason": run.reason_code.value if run.reason_code is not None else None,
            "step_results": step_results,
            "artifact_ids": [artifact.artifact_id for artifact in indexed],
            "recording": recording_manifest,
        }
        self.artifacts.write_json(run.run_id, "manifest", "manifest.json", manifest)
        self.artifacts.write_checksums(run.run_id)

    def _archive_is_complete(self, run_id: str) -> bool:
        run_dir = self.artifacts.run_dir(run_id)
        required = (
            run_dir / "manifest.json",
            run_dir / "summary.json",
            run_dir / "events.jsonl",
            run_dir / "resolved-scenario.yaml",
            run_dir / "ros" / "topics-snapshot.json",
            run_dir / "ros" / "nodes-snapshot.json",
            run_dir / "checksums.sha256",
        )
        if not all(path.is_file() and not path.is_symlink() for path in required):
            return False
        try:
            json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))
            summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
            if not set(self._archive_conditions(run_id)) <= set(summary.get("missing_data", [])):
                return False
            json.loads((run_dir / "ros" / "topics-snapshot.json").read_text(encoding="utf-8"))
            json.loads((run_dir / "ros" / "nodes-snapshot.json").read_text(encoding="utf-8"))
            indexed_paths = {artifact.path for artifact in self.artifacts.list(run_id)}
            if any(path.resolve() not in indexed_paths for path in required):
                return False
            indexed = tuple(
                artifact
                for artifact in self.artifacts.list(run_id)
                if artifact.path.name != "checksums.sha256"
            )
            expected_checksums = {
                artifact.path.relative_to(run_dir).as_posix(): artifact.checksum.removeprefix("sha256:")
                for artifact in indexed
            }
            observed_checksums: dict[str, str] = {}
            for line in (run_dir / "checksums.sha256").read_text(encoding="utf-8").splitlines():
                digest, separator, relative_text = line.partition("  ")
                relative = Path(relative_text)
                if not separator or len(digest) != 64 or relative.is_absolute() or ".." in relative.parts:
                    return False
                if relative_text in observed_checksums:
                    return False
                observed_checksums[relative_text] = digest
                candidate = run_dir / relative
                if not candidate.is_file() or candidate.is_symlink() or _file_digest(candidate) != digest:
                    return False
            if observed_checksums != expected_checksums:
                return False
        except (OSError, ValueError, TypeError, json.JSONDecodeError):
            return False
        return True

    def _persist_archive_conditions(self, run_id: str, conditions: Sequence[str]) -> None:
        normalized = {item for item in conditions if isinstance(item, str) and item}
        if not normalized:
            return
        with self.states.database.transaction(immediate=True) as connection:
            row = connection.execute(
                "SELECT archive_conditions_json FROM runs WHERE run_id = ?",
                (run_id,),
            ).fetchone()
            if row is None:
                raise RunNotFound(run_id)
            merged = sorted(set(json.loads(row["archive_conditions_json"])) | normalized)
            connection.execute(
                "UPDATE runs SET archive_conditions_json = ? WHERE run_id = ?",
                (json.dumps(merged, separators=(",", ":")), run_id),
            )

    def _archive_conditions(self, run_id: str) -> tuple[str, ...]:
        with self.states.database.connect() as connection:
            row = connection.execute(
                "SELECT archive_conditions_json FROM runs WHERE run_id = ?",
                (run_id,),
            ).fetchone()
        if row is None:
            raise RunNotFound(run_id)
        return tuple(json.loads(row["archive_conditions_json"]))

    @staticmethod
    def _shutdown_conditions(errors: Sequence[str]) -> tuple[str, ...]:
        if not errors:
            return ()
        return ("shutdown_unconfirmed", *(f"stop_error:{error}" for error in errors))

    def _archive_log(
        self,
        run_id: str,
        agent_kind: str,
        process_id: str,
        stream: str,
        path: Path,
    ) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_name(path.name + ".partial")
        cursor = 0
        with temporary.open("wb") as output:
            while True:
                chunk = self.agents[agent_kind].logs(
                    run_id,
                    process_id,
                    stream=stream,
                    after=cursor,
                    limit=1024 * 1024,
                )
                next_cursor = chunk.get("cursor")
                if not isinstance(next_cursor, int) or next_cursor <= cursor:
                    break
                data = chunk.get("data", "")
                if not isinstance(data, str):
                    raise TypeError("Agent log data must be text")
                output.write(data.encode("utf-8"))
                cursor = next_cursor
                if chunk.get("eof") is True:
                    break
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary, path)

    def _index_bags(
        self,
        run_id: str,
        context: RunContext,
        missing_data: list[str],
    ) -> list[Artifact]:
        indexed: list[Artifact] = []
        for plan in context.recorders:
            if not plan.output_path.exists():
                missing_data.append(f"bag:{plan.agent}")
                continue
            files = sorted(path for path in plan.output_path.rglob("*") if path.is_file() and not path.is_symlink())
            if not files:
                missing_data.append(f"bag:{plan.agent}")
                continue
            for path in files:
                kind = "ros1_bag" if plan.agent == "ros1" else "ros2_bag"
                indexed.append(self.artifacts.register(run_id, kind, path))
        return indexed

    def _capture_graphs(self, run_id: str, context: RunContext) -> None:
        for agent_kind in self._agent_kinds(context):
            try:
                self._capture_graph(run_id, context, agent_kind)
            except (OSError, ValueError):
                _LOGGER.exception("graph archive capture failed for %s/%s", run_id, agent_kind)

    def _capture_graph(self, run_id: str, context: RunContext, agent_kind: str) -> dict[str, Any]:
        try:
            snapshot = self.agents[agent_kind].graph_snapshot(
                run_id,
                {"runtime": context.manifest.runtime.model_dump(mode="json")},
            )
            if not isinstance(snapshot, dict):
                raise TypeError("graph snapshot must be an object")
        except (AttributeError, AgentUnavailable, AgentHTTPError, OSError, ValueError, KeyError, TypeError):
            snapshot = {
                "agent": agent_kind,
                "nodes": [],
                "topics": [],
                "available": False,
                "nodes_available": False,
                "topics_available": False,
            }
        graph_path = self.artifacts.run_dir(run_id) / "ros" / f"{agent_kind}-graph.json"
        _atomic_json(graph_path, snapshot)
        self.artifacts.register(run_id, "ros_graph", graph_path)
        return snapshot

    def _step_results(self, run_id: str) -> list[dict[str, Any]]:
        with self.states.database.connect() as connection:
            rows = connection.execute(
                "SELECT step_id, state, started_at, ended_at, detail FROM run_steps WHERE run_id = ? ORDER BY run_step_id",
                (run_id,),
            ).fetchall()
        return [dict(row) for row in rows]

    def _agent_kinds(self, context: RunContext) -> tuple[str, ...]:
        return tuple(
            dict.fromkeys(
                [step.agent for step in context.manifest.steps] + [plan.agent for plan in context.recorders]
            )
        )

    def _context(self, run: RunRecord) -> RunContext:
        payload = run.resolved_manifest
        manifest = ScenarioManifest.model_validate(payload["manifest"])
        parameters = dict(payload["parameters"])
        recording = normalize_recording(payload.get("recording"))
        step_payloads = {item["step_id"]: item for item in payload["steps"]}
        invocations = {
            step.id: CommandInvocation(
                argv=tuple(step_payloads[step.id]["argv"]),
                environment=step_payloads[step.id]["environment"],
            )
            for step in manifest.steps
        }
        recorders = build_recorder_plans(manifest, recording, self.artifacts.run_dir(run.run_id))
        return RunContext(manifest, parameters, invocations, recording, recorders)

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


def _atomic_json(path: Path, payload: Mapping[str, Any]) -> None:
    encoded = json.dumps(dict(payload), indent=2, sort_keys=True, ensure_ascii=False).encode("utf-8") + b"\n"
    _atomic_bytes(path, encoded)


def _atomic_bytes(path: Path, encoded: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".partial")
    with temporary.open("wb") as stream:
        stream.write(encoded)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def _file_digest(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        while chunk := stream.read(1024 * 1024):
            digest.update(chunk)
    return digest.hexdigest()


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
