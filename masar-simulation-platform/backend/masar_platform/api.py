"""FastAPI contract for the MASAR simulation platform."""

from __future__ import annotations

from contextlib import asynccontextmanager
import asyncio
import os
from typing import Any, Literal
from urllib.parse import quote
import uuid

from fastapi import FastAPI, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse, StreamingResponse
from pydantic import BaseModel, ConfigDict, Field
from starlette.exceptions import HTTPException as StarletteHTTPException
from starlette.concurrency import run_in_threadpool

from .agents import AgentClient, AgentHTTPError, AgentUnavailable, load_master_secret
from .artifacts import ArtifactNotFound, ArtifactStore
from .catalog import CatalogError, ScenarioCatalog
from .commands import render_invocation
from .displays import check_display_prerequisites, display_plan
from .config import Settings, get_settings
from .db import Database, datetime_to_text
from .events import EventStore
from .gateways import check_web_gateway_prerequisites
from .leases import LeaseStore
from .orchestrator import (
    CommandNotSupported,
    Orchestrator,
    OrchestratorError,
    PreflightChecker,
    PreflightFailed,
    RunConflict,
    RunNotActive,
)
from .profiles import get_display_profile, get_topic_profile
from .state_machine import RunNotFound, RunRecord, RunStateMachine


API_PREFIX = "/api/v1"


class RecordingRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    enabled: bool = False
    profile: Literal["default"] = "default"


class RunCreateRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    scenario_id: str = Field(min_length=1)
    parameters: dict[str, Any] = Field(default_factory=dict)
    recording: RecordingRequest = Field(default_factory=RecordingRequest)


class ValidateScenarioRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    parameters: dict[str, Any] = Field(default_factory=dict)


def build_orchestrator(settings: Settings | None = None) -> Orchestrator:
    runtime_settings = settings or get_settings()
    database = Database(runtime_settings.data_dir / "platform.sqlite3")
    database.initialize()
    events = EventStore(
        database,
        runtime_settings.data_dir,
        retention_days=runtime_settings.event_retention_days,
        retention_count=runtime_settings.event_retention_count,
    )
    states = RunStateMachine(database, events)
    leases = LeaseStore(database, events)
    artifacts = ArtifactStore(database, runtime_settings.data_dir)
    catalog = ScenarioCatalog.from_directory(runtime_settings.scenario_dir)
    secret = load_master_secret(runtime_settings.secret_path)
    agents: dict[str, Any] = {
        "ros1": AgentClient(runtime_settings.ros1_agent_host, runtime_settings.ros1_agent_port, secret),
        "ros2": AgentClient(runtime_settings.ros2_agent_host, runtime_settings.ros2_agent_port, secret),
    }
    preflight = PreflightChecker(
        runtime_settings.data_dir,
        agents,
        display_check=lambda manifest: check_display_prerequisites(
            manifest,
            runtime_settings.visual_stack_ready,
            gazebo_port=runtime_settings.ros1_novnc_port,
            rviz_port=runtime_settings.ros2_novnc_port,
            webots_port=runtime_settings.webots_novnc_port,
        ),
        gateway_check=lambda manifest: check_web_gateway_prerequisites(
            manifest,
            ros1_port=runtime_settings.ros1_rosbridge_port,
            ros2_port=runtime_settings.ros2_rosbridge_port,
        ),
    )
    return Orchestrator(catalog, states, leases, events, artifacts, agents, preflight)


def create_api_app(
    orchestrator: Orchestrator | None = None,
    settings: Settings | None = None,
) -> FastAPI:
    supplied_orchestrator = orchestrator

    @asynccontextmanager
    async def lifespan(application: FastAPI):
        application.state.orchestrator = supplied_orchestrator or build_orchestrator(settings)
        if supplied_orchestrator is None:
            application.state.orchestrator.recover_active_runs()
            application.state.orchestrator.finalize_incomplete_runs()
        yield

    application = FastAPI(
        title="MASAR Simulation Platform",
        version="0.2.0",
        lifespan=lifespan,
    )

    @application.middleware("http")
    async def request_identity(request: Request, call_next):
        request.state.request_id = "request-" + uuid.uuid4().hex
        response = await call_next(request)
        response.headers["X-Request-ID"] = request.state.request_id
        response.headers["Cache-Control"] = "no-store"
        return response

    @application.exception_handler(RequestValidationError)
    async def validation_error(request: Request, exc: RequestValidationError) -> JSONResponse:
        return _error(request, 422, "INVALID_REQUEST", "request validation failed", {"errors": exc.errors()})

    @application.exception_handler(StarletteHTTPException)
    async def http_error(request: Request, exc: StarletteHTTPException) -> JSONResponse:
        code = "NOT_FOUND" if exc.status_code == 404 else "HTTP_ERROR"
        return _error(request, exc.status_code, code, str(exc.detail), {})

    @application.exception_handler(Exception)
    async def platform_error(request: Request, exc: Exception) -> JSONResponse:
        if isinstance(exc, RunConflict):
            return _error(request, 409, "RUN_ALREADY_ACTIVE", str(exc), {"run_id": exc.run_id})
        if isinstance(exc, PreflightFailed):
            return _error(request, exc.status_code, exc.code, str(exc), exc.details)
        if isinstance(exc, RunNotFound):
            return _error(request, 404, "RUN_NOT_FOUND", "run was not found", {})
        if isinstance(exc, ArtifactNotFound):
            return _error(request, 404, "ARTIFACT_NOT_FOUND", "artifact was not found", {})
        if isinstance(exc, CatalogError):
            return _error(request, 422, "INVALID_PARAMETERS", str(exc), {})
        if isinstance(exc, CommandNotSupported):
            return _error(request, 409, "COMMAND_NOT_SUPPORTED", str(exc), {})
        if isinstance(exc, RunNotActive):
            return _error(request, 409, "RUN_NOT_ACTIVE", str(exc), {})
        if isinstance(exc, (AgentUnavailable, AgentHTTPError)):
            return _error(request, 503, "AGENT_UNAVAILABLE", "Runtime Agent is unavailable", {})
        if isinstance(exc, OrchestratorError):
            return _error(request, 500, "ORCHESTRATION_FAILED", str(exc), {})
        return _error(request, 500, "INTERNAL_ERROR", "internal platform error", {})

    @application.get("/healthz", tags=["system"])
    async def healthz() -> dict[str, str]:
        return {"status": "ok"}

    @application.get(f"{API_PREFIX}/capabilities")
    async def capabilities(request: Request) -> dict[str, Any]:
        orchestrated = _orchestrator(request)
        return {
            "schema_version": 1,
            "single_active_run": True,
            "workspaces": [
                {"id": "group_behavior", "label": "群行为与控制验证", "scenario_modules": ["swarm", "control"]},
                {"id": "exploration", "label": "探索组独立运行", "scenario_modules": ["exploration"]},
                {"id": "rescue", "label": "完整搜救系统", "scenario_modules": ["rescue"]},
            ],
            "simulators": ["gazebo", "webots"],
            "scenario_count": len(orchestrated.catalog),
            "event_transport": "websocket+cursor",
        }

    @application.get(f"{API_PREFIX}/scenarios")
    async def scenarios(request: Request) -> dict[str, Any]:
        return {"scenarios": [_scenario_summary(manifest) for manifest in _orchestrator(request).catalog]}

    @application.get(f"{API_PREFIX}/scenarios/{{scenario_id}}")
    async def scenario(request: Request, scenario_id: str) -> dict[str, Any]:
        try:
            manifest = _orchestrator(request).catalog.get(scenario_id)
        except CatalogError as exc:
            raise StarletteHTTPException(status_code=404, detail="scenario was not found") from exc
        return _scenario_detail(manifest)

    @application.post(f"{API_PREFIX}/scenarios/{{scenario_id}}/validate")
    async def validate_scenario(
        request: Request,
        scenario_id: str,
        payload: ValidateScenarioRequest,
    ) -> dict[str, Any]:
        orchestrated = _orchestrator(request)
        manifest = orchestrated.catalog.get(scenario_id)
        resolved = orchestrated.catalog.validate_parameters(scenario_id, payload.parameters)
        return {
            "valid": True,
            "scenario_id": scenario_id,
            "parameters": resolved,
            "runtime": manifest.runtime.model_dump(mode="json"),
            "display_plan": display_plan(manifest).as_dict(),
            "steps": [
                {
                    "step_id": step.id,
                    "agent": step.agent,
                    "invocation": {
                        "argv": list(render.argv),
                        "environment": dict(render.environment),
                    },
                }
                for step in manifest.steps
                for render in [render_invocation(step.command_ref, resolved, manifest=manifest)]
            ],
        }

    @application.post(f"{API_PREFIX}/runs", status_code=202)
    async def create_run(request: Request, payload: RunCreateRequest) -> dict[str, Any]:
        orchestrated = _orchestrator(request)
        run = orchestrated.create_run(
            payload.scenario_id,
            payload.parameters,
            payload.recording.model_dump(mode="json"),
        )
        orchestrated.start_async(run.run_id)
        return {"run_id": run.run_id, "state": run.state.value}

    @application.get(f"{API_PREFIX}/runs")
    async def runs(request: Request, limit: int = Query(default=100, ge=1, le=1000)) -> dict[str, Any]:
        return {"runs": [_run_dict(run) for run in _orchestrator(request).states.list(limit=limit)]}

    @application.get(f"{API_PREFIX}/runs/current")
    async def current_run(request: Request) -> dict[str, Any]:
        current = _orchestrator(request).states.current()
        return {"run": _run_dict(current) if current is not None else None}

    @application.get(f"{API_PREFIX}/runs/{{run_id}}")
    async def get_run(request: Request, run_id: str) -> dict[str, Any]:
        return _run_dict(_orchestrator(request).states.get(run_id))

    @application.post(f"{API_PREFIX}/runs/{{run_id}}/stop")
    async def stop_run(request: Request, run_id: str) -> dict[str, Any]:
        return _run_dict(_orchestrator(request).stop_run(run_id))

    @application.get(f"{API_PREFIX}/runs/{{run_id}}/events")
    async def run_events(request: Request, run_id: str, after: int = Query(default=0, ge=0)) -> dict[str, Any]:
        _orchestrator(request).states.get(run_id)
        events = _orchestrator(request).events.list(run_id, after_sequence=after)
        return {"events": [event.as_dict() for event in events]}

    @application.get(f"{API_PREFIX}/runs/{{run_id}}/logs")
    async def run_logs(
        request: Request,
        run_id: str,
        process: str,
        stream: Literal["stdout", "stderr"] = "stdout",
        after: int = Query(default=0, ge=0),
        limit: int = Query(default=65536, ge=1, le=1048576),
    ) -> dict[str, Any]:
        return _orchestrator(request).logs(run_id, process, stream=stream, after=after, limit=limit)

    @application.get(f"{API_PREFIX}/runs/{{run_id}}/artifacts")
    async def artifacts(request: Request, run_id: str) -> dict[str, Any]:
        _orchestrator(request).states.get(run_id)
        return {"artifacts": [artifact.as_dict() for artifact in _orchestrator(request).artifacts.list(run_id)]}

    @application.get(f"{API_PREFIX}/runs/{{run_id}}/artifacts/{{artifact_id}}/download")
    async def download_artifact(request: Request, run_id: str, artifact_id: str) -> StreamingResponse:
        range_header = request.headers.get("range")
        if range_header is not None and "," in range_header:
            raise StarletteHTTPException(status_code=400, detail="multiple byte ranges are not supported")
        opened = await run_in_threadpool(_orchestrator(request).artifacts.open, run_id, artifact_id)
        try:
            start, end, status_code = _byte_range(range_header, opened.artifact.size_bytes)
        except ValueError as exc:
            os.close(opened.file_descriptor)
            raise StarletteHTTPException(status_code=416, detail="requested byte range is invalid") from exc
        content_length = end - start + 1 if end >= start else 0
        headers = {
            "Accept-Ranges": "bytes",
            "Content-Length": str(content_length),
            "Content-Disposition": "attachment; filename*=UTF-8''" + quote(opened.artifact.path.name),
            "X-Artifact-Checksum": opened.artifact.checksum,
        }
        if status_code == 206:
            headers["Content-Range"] = f"bytes {start}-{end}/{opened.artifact.size_bytes}"
        return StreamingResponse(
            _stream_descriptor(opened.file_descriptor, start, content_length),
            status_code=status_code,
            headers=headers,
            media_type="application/octet-stream",
        )

    @application.post(f"{API_PREFIX}/runs/{{run_id}}/mission-commands/start")
    async def mission_start(request: Request, run_id: str) -> dict[str, Any]:
        return _orchestrator(request).issue_command(run_id, "mission.start")

    @application.post(f"{API_PREFIX}/runs/{{run_id}}/mission-commands/stop")
    async def mission_stop(request: Request, run_id: str) -> dict[str, Any]:
        return _orchestrator(request).issue_command(run_id, "mission.stop")

    @application.post(f"{API_PREFIX}/runs/{{run_id}}/mission-commands/home")
    async def mission_home(request: Request, run_id: str) -> dict[str, Any]:
        return _orchestrator(request).issue_command(run_id, "mission.home")

    @application.post(f"{API_PREFIX}/runs/{{run_id}}/response-commands/return-home")
    async def response_home(request: Request, run_id: str) -> dict[str, Any]:
        return _orchestrator(request).issue_command(run_id, "response.return_home")

    @application.get(f"{API_PREFIX}/runs/{{run_id}}/data-sources")
    async def data_sources(request: Request, run_id: str) -> dict[str, Any]:
        run = _orchestrator(request).states.get(run_id)
        manifest = _orchestrator(request).catalog.get(run.scenario_id)
        profile = get_topic_profile(manifest.data_profile)
        return {
            "data_sources": [
                {
                    "topic_pattern": topic.pattern,
                    "message_types": list(topic.message_types),
                    "graph": "ros2" if any("/msg/" in item for item in topic.message_types) else "ros1",
                    "capability": topic.capability,
                    "required": topic.required,
                    "availability": "UNKNOWN",
                }
                for topic in profile.topics
            ]
        }

    @application.get(f"{API_PREFIX}/runs/{{run_id}}/robots")
    async def robots(request: Request, run_id: str) -> dict[str, Any]:
        run = _orchestrator(request).states.get(run_id)
        manifest = _orchestrator(request).catalog.get(run.scenario_id)
        parameters = run.resolved_manifest.get("parameters", {})
        count = parameters.get("number_robots")
        if not isinstance(count, bool) and isinstance(count, int) and count > 0:
            model = parameters.get("robot")
            return {
                "robots": [
                    {
                        "robot_id": f"robot_{index}",
                        "label": f"机器人 {index}",
                        "model": model if isinstance(model, str) else None,
                        "topic_prefix": f"/robot_{index}",
                        "expected": True,
                    }
                    for index in range(count)
                ]
            }
        if manifest.module not in {"exploration", "rescue"}:
            return {"robots": []}
        return {
            "robots": [
                {
                    "robot_id": f"agent_{index}",
                    "label": f"搜索机器人 {index}",
                    "model": "robosar_search",
                    "topic_prefix": f"/robosar_agent_bringup_node/agent_{index}",
                    "expected": True,
                }
                for index in range(5)
            ]
        }

    @application.websocket(f"{API_PREFIX}/events")
    async def event_socket(websocket: WebSocket) -> None:
        await websocket.accept()
        orchestrated: Orchestrator = websocket.app.state.orchestrator
        run_id = websocket.query_params.get("run_id", "")
        try:
            cursor = int(websocket.query_params.get("after", "0"))
            if cursor < 0:
                raise ValueError
            orchestrated.states.get(run_id)
        except (ValueError, RunNotFound):
            await websocket.send_json({"type": "error", "code": "INVALID_EVENT_CURSOR"})
            await websocket.close(code=1008)
            return
        last_ping = asyncio.get_running_loop().time()
        try:
            while True:
                events = orchestrated.events.list(run_id, after_sequence=cursor)
                for event in events:
                    await websocket.send_json({"type": "event", "event": event.as_dict()})
                    cursor = event.sequence
                now = asyncio.get_running_loop().time()
                if now - last_ping >= 15:
                    await websocket.send_json({"type": "ping", "cursor": cursor})
                    last_ping = now
                await asyncio.sleep(0.25)
        except WebSocketDisconnect:
            return

    return application


def _orchestrator(request: Request) -> Orchestrator:
    return request.app.state.orchestrator


def _error(
    request: Request,
    status: int,
    code: str,
    message: str,
    details: dict[str, Any],
) -> JSONResponse:
    request_id = getattr(request.state, "request_id", "request-" + uuid.uuid4().hex)
    return JSONResponse(
        status_code=status,
        content={"error": {"code": code, "message": message, "details": details}, "request_id": request_id},
    )


def _scenario_summary(manifest) -> dict[str, Any]:
    ui_module = "group_behavior" if manifest.module in {"swarm", "control"} else manifest.module
    return {
        "scenario_id": manifest.scenario_id,
        "workspace": ui_module,
        "module": manifest.module,
        "simulator": manifest.simulator,
        "display_profile": manifest.display_profile,
        "panels": list(get_display_profile(manifest.display_profile).panels),
        "commands": sorted(_SCENARIO_COMMANDS_PUBLIC.get(manifest.scenario_id, ())),
    }


_SCENARIO_COMMANDS_PUBLIC = {
    "swarm.webots.ros2swarm": ("mission.start", "mission.stop"),
    "control.gazebo.response-deform": ("mission.start", "mission.stop", "response.return_home"),
    "control.gazebo.response-controller": (),
    "exploration.gazebo.tepper": ("mission.start", "mission.stop", "mission.home"),
    "exploration.webots.robosar": ("mission.start", "mission.stop", "mission.home"),
    "rescue.gazebo.tepper-response": (
        "mission.start",
        "mission.stop",
        "mission.home",
        "response.return_home",
    ),
}


def _scenario_detail(manifest) -> dict[str, Any]:
    detail = _scenario_summary(manifest)
    detail.update(
        {
            "runtime": manifest.runtime.model_dump(mode="json"),
            "parameters": {name: definition.model_dump(mode="json") for name, definition in manifest.parameters.items()},
            "data_profile": manifest.data_profile,
            "artifact_profile": manifest.artifact_profile,
            "display_plan": display_plan(manifest).as_dict(),
        }
    )
    return detail


def _run_dict(run: RunRecord) -> dict[str, Any]:
    return {
        "run_id": run.run_id,
        "scenario_id": run.scenario_id,
        "state": run.state.value,
        "reason_code": run.reason_code.value if run.reason_code is not None else None,
        "created_at": datetime_to_text(run.created_at),
        "updated_at": datetime_to_text(run.updated_at),
        "ended_at": datetime_to_text(run.ended_at) if run.ended_at is not None else None,
        "parameters": run.resolved_manifest.get("parameters", {}),
    }


def _byte_range(value: str | None, size: int) -> tuple[int, int, int]:
    if value is None:
        return 0, size - 1, 200
    if not value.startswith("bytes="):
        raise ValueError("range unit is invalid")
    boundary = value[6:]
    if "-" not in boundary:
        raise ValueError("range boundary is invalid")
    start_text, end_text = boundary.split("-", 1)
    if not start_text:
        suffix = int(end_text)
        if suffix <= 0 or size == 0:
            raise ValueError("suffix range is invalid")
        start = max(0, size - suffix)
        end = size - 1
    else:
        start = int(start_text)
        end = size - 1 if not end_text else int(end_text)
        if start < 0 or start >= size or end < start:
            raise ValueError("range is outside the artifact")
        end = min(end, size - 1)
    return start, end, 206


def _stream_descriptor(file_descriptor: int, start: int, length: int):
    try:
        os.lseek(file_descriptor, start, os.SEEK_SET)
        remaining = length
        while remaining:
            chunk = os.read(file_descriptor, min(1024 * 1024, remaining))
            if not chunk:
                break
            remaining -= len(chunk)
            yield chunk
    finally:
        os.close(file_descriptor)


__all__ = ["API_PREFIX", "build_orchestrator", "create_api_app"]
