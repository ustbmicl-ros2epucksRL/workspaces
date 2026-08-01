#!/usr/bin/env python3
"""Standard-library HTTP runtime Agent for either existing ROS container."""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import ipaddress
import json
from pathlib import Path
import re
import stat
import sys
from typing import Any, Dict, Mapping, Optional, Tuple
from urllib.parse import parse_qs, urlsplit
import uuid

if __package__:
    from .actions import ActionNotAllowed, ActionPublishError, ActionPublisher
    from .processes import (
        CommandNotAllowed,
        GracePeriods,
        ProcessConflict,
        ProcessManager,
        ProcessManagerError,
        ProcessNotFound,
        ProcessOwnershipError,
    )
    from .protocol import (
        PROTOCOL_VERSION,
        AuthenticationError,
        RequestAuthenticator,
        error_response,
        success_response,
    )
    from .probes import ProbeRequestError, ProbeRunner
else:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from processes import (  # type: ignore[no-redef]
        CommandNotAllowed,
        GracePeriods,
        ProcessConflict,
        ProcessManager,
        ProcessManagerError,
        ProcessNotFound,
        ProcessOwnershipError,
    )
    from protocol import (  # type: ignore[no-redef]
        PROTOCOL_VERSION,
        AuthenticationError,
        RequestAuthenticator,
        error_response,
        success_response,
    )
    from probes import ProbeRequestError, ProbeRunner  # type: ignore[no-redef]
    from actions import ActionNotAllowed, ActionPublishError, ActionPublisher  # type: ignore[no-redef]


_RUN_ROUTE = re.compile(r"^/v1/runs/([^/]+)/(start|stop|probe|logs|command|graph)$")
_MAX_BODY_BYTES = 1024 * 1024


class AgentApplication:
    def __init__(
        self,
        agent_kind: str,
        master_secret: bytes,
        processes: ProcessManager,
        probe_runner: Optional[ProbeRunner] = None,
        action_publisher: Optional[ActionPublisher] = None,
    ) -> None:
        self.agent_kind = agent_kind
        self.authenticator = RequestAuthenticator(master_secret)
        self.processes = processes
        self.probe_runner = probe_runner or ProbeRunner(agent_kind, processes.workspace)
        self.action_publisher = action_publisher or ActionPublisher(agent_kind)

    def handle(
        self,
        method: str,
        target: str,
        headers: Mapping[str, str],
        body: bytes,
    ) -> Tuple[int, bytes]:
        request_id = "request-" + uuid.uuid4().hex
        parsed = urlsplit(target)
        if method == "GET" and parsed.path == "/healthz":
            return 200, success_response(
                {"agent": self.agent_kind, "protocol_version": PROTOCOL_VERSION, "status": "ok"},
                request_id=request_id,
            )

        match = _RUN_ROUTE.fullmatch(parsed.path)
        if match is None:
            return 404, error_response("NOT_FOUND", "Agent route was not found", request_id=request_id)
        run_id, operation = match.groups()
        try:
            context = self.authenticator.authenticate(method, target, body, headers)
            if context.run_id != run_id:
                return 403, error_response(
                    "RUN_ID_MISMATCH",
                    "signed run_id does not match the request path",
                    request_id=request_id,
                )
            if operation == "start" and method == "POST":
                payload = _json_object(body)
                record = self.processes.start(
                    run_id,
                    _required_string(payload, "process_id"),
                    _required_string(payload, "command_ref"),
                    _required_string_list(payload, "argv"),
                    _string_mapping(payload.get("environment", {}), "environment"),
                )
                return 202, success_response(record.as_dict(), request_id=request_id)
            if operation == "stop" and method == "POST":
                payload = _json_object(body)
                grace_value = payload.get("grace_seconds", {})
                grace_mapping = _number_mapping(grace_value, "grace_seconds")
                grace = GracePeriods(
                    grace_mapping.get("sigint", 10.0),
                    grace_mapping.get("sigterm", 5.0),
                    grace_mapping.get("sigkill", 5.0),
                )
                process_id = payload.get("process_id")
                if process_id is None:
                    records = self.processes.stop_all(run_id, grace)
                    return 200, success_response(
                        {"run_id": run_id, "processes": [record.as_dict() for record in records]},
                        request_id=request_id,
                    )
                record = self.processes.stop(run_id, _string_value(process_id, "process_id"), grace)
                return 200, success_response(record.as_dict(), request_id=request_id)
            if operation == "probe" and method == "GET":
                records = self.processes.probe(run_id)
                return 200, success_response(
                    {"run_id": run_id, "processes": [record.as_dict() for record in records]},
                    request_id=request_id,
                )
            if operation == "probe" and method == "POST":
                payload = _json_object(body)
                probes = payload.get("probes")
                if not isinstance(probes, list):
                    raise ProbeRequestError("probes must be a non-empty array")
                runtime = payload.get("runtime")
                if not isinstance(runtime, dict):
                    raise ProbeRequestError("runtime must be an object")
                results = self.probe_runner.run_many(probes, runtime)
                records = self.processes.probe(run_id)
                return 200, success_response(
                    {
                        "run_id": run_id,
                        "processes": [record.as_dict() for record in records],
                        "probes": results,
                    },
                    request_id=request_id,
                )
            if operation == "logs" and method == "GET":
                query = parse_qs(parsed.query, keep_blank_values=True)
                process_id = _single_query(query, "process_id")
                stream = _single_query(query, "stream", default="stdout")
                after = _query_integer(query, "after", default=0)
                limit = _query_integer(query, "limit", default=65536)
                chunk = self.processes.read_logs(run_id, process_id, stream, after, limit)
                data = {"run_id": run_id, "process_id": process_id, "stream": stream}
                data.update(chunk.as_dict())
                return 200, success_response(data, request_id=request_id)
            if operation == "command" and method == "POST":
                payload = _json_object(body)
                result = self.action_publisher.publish(
                    _required_string(payload, "action"),
                    payload.get("runtime", {}),
                )
                result["run_id"] = run_id
                return 200, success_response(result, request_id=request_id)
            if operation == "graph" and method == "POST":
                payload = _json_object(body)
                if set(payload) != {"runtime"} or not isinstance(payload["runtime"], dict):
                    raise ProbeRequestError("graph snapshot requires exactly one runtime object")
                result = self.probe_runner.graph_snapshot(payload["runtime"])
                result["run_id"] = run_id
                return 200, success_response(result, request_id=request_id)
            return 405, error_response(
                "METHOD_NOT_ALLOWED", "method is not allowed for this route", request_id=request_id
            )
        except AuthenticationError as exc:
            return 401, error_response(exc.code, str(exc), request_id=request_id)
        except CommandNotAllowed as exc:
            return 422, error_response("COMMAND_NOT_ALLOWED", str(exc), request_id=request_id)
        except ActionNotAllowed as exc:
            return 422, error_response("ACTION_NOT_ALLOWED", str(exc), request_id=request_id)
        except ActionPublishError:
            return 500, error_response("ACTION_PUBLISH_FAILED", "fixed ROS action publish failed", request_id=request_id)
        except ProcessConflict as exc:
            return 409, error_response("PROCESS_CONFLICT", str(exc), request_id=request_id)
        except ProcessNotFound as exc:
            return 404, error_response("PROCESS_NOT_FOUND", str(exc), request_id=request_id)
        except ProcessOwnershipError as exc:
            return 409, error_response("OWNERSHIP_NOT_PROVEN", str(exc), request_id=request_id)
        except (ValueError, TypeError, json.JSONDecodeError) as exc:
            return 422, error_response("INVALID_REQUEST", str(exc), request_id=request_id)
        except ProcessManagerError:
            return 500, error_response("PROCESS_ERROR", "owned process operation failed", request_id=request_id)


class AgentHTTPServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address: Tuple[str, int], app: AgentApplication) -> None:
        self.app = app
        super().__init__(address, AgentRequestHandler)


class AgentRequestHandler(BaseHTTPRequestHandler):
    server_version = "MasarRuntimeAgent/1"

    def do_GET(self) -> None:  # noqa: N802
        self._dispatch()

    def do_POST(self) -> None:  # noqa: N802
        self._dispatch()

    def _dispatch(self) -> None:
        content_length = _content_length(self.headers.get("Content-Length"))
        if content_length > _MAX_BODY_BYTES:
            body = error_response("BODY_TOO_LARGE", "request body exceeds the Agent limit")
            self._write(413, body)
            return
        request_body = self.rfile.read(content_length) if content_length else b""
        status, response_body = self.server.app.handle(  # type: ignore[attr-defined]
            self.command,
            self.path,
            self.headers,
            request_body,
        )
        self._write(status, response_body)

    def _write(self, status: int, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:
        return


def create_server(host: str, port: int, app: AgentApplication) -> AgentHTTPServer:
    try:
        address = ipaddress.ip_address(host)
    except ValueError as exc:
        raise ValueError("Agent host must be a loopback IP address") from exc
    if not address.is_loopback:
        raise ValueError("Agent host must be a loopback IP address")
    return AgentHTTPServer((host, port), app)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="MASAR container runtime Agent")
    parser.add_argument("--agent", choices=("ros1", "ros2"), required=True)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int)
    parser.add_argument("--workspace")
    parser.add_argument("--runs-dir", default="/root/workspace/masar-simulation-platform/runs")
    parser.add_argument("--secret-path", default="/run/masar-platform/agent.secret")
    return parser


def main(argv: Optional[list] = None) -> int:
    args = build_parser().parse_args(argv)
    defaults = {
        "ros1": (18101, "/root/workspace/masar_ws"),
        "ros2": (18102, "/root/workspace/ROS2swarm_w"),
    }
    default_port, default_workspace = defaults[args.agent]
    secret = _load_secret(Path(args.secret_path))
    manager = ProcessManager(
        agent_kind=args.agent,
        workspace=Path(args.workspace or default_workspace),
        runs_dir=Path(args.runs_dir) / ("agent-" + args.agent),
    )
    server = create_server(args.host, args.port or default_port, AgentApplication(args.agent, secret, manager))
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
    return 0


def _load_secret(path: Path) -> bytes:
    info = path.stat()
    if not stat.S_ISREG(info.st_mode) or stat.S_IMODE(info.st_mode) != 0o600:
        raise ValueError("Agent secret must be a regular file with mode 0600")
    secret = path.read_bytes()
    if len(secret) != 32:
        raise ValueError("Agent secret must contain exactly 32 bytes")
    return secret


def _content_length(value: Optional[str]) -> int:
    if value is None:
        return 0
    try:
        length = int(value)
    except ValueError as exc:
        raise ValueError("invalid Content-Length") from exc
    return max(0, length)


def _json_object(body: bytes) -> Dict[str, Any]:
    if not body:
        return {}
    value = json.loads(body.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("request body must be a JSON object")
    return value


def _required_string(payload: Mapping[str, Any], name: str) -> str:
    if name not in payload:
        raise ValueError("missing required field: " + name)
    return _string_value(payload[name], name)


def _string_value(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise ValueError(name + " must be a non-empty string")
    return value


def _required_string_list(payload: Mapping[str, Any], name: str) -> list:
    value = payload.get(name)
    if not isinstance(value, list) or not value or any(not isinstance(item, str) for item in value):
        raise ValueError(name + " must be a non-empty string array")
    return value


def _string_mapping(value: Any, name: str) -> Dict[str, str]:
    if not isinstance(value, dict) or any(
        not isinstance(key, str) or not isinstance(item, str) for key, item in value.items()
    ):
        raise ValueError(name + " must be a string map")
    return value


def _number_mapping(value: Any, name: str) -> Dict[str, float]:
    if not isinstance(value, dict):
        raise ValueError(name + " must be an object")
    allowed = {"sigint", "sigterm", "sigkill"}
    if set(value) - allowed:
        raise ValueError(name + " contains unknown fields")
    if any(isinstance(item, bool) or not isinstance(item, (int, float)) for item in value.values()):
        raise ValueError(name + " values must be numbers")
    return value


def _single_query(query: Mapping[str, list], name: str, default: Optional[str] = None) -> str:
    values = query.get(name)
    if values is None:
        if default is not None:
            return default
        raise ValueError("missing query parameter: " + name)
    if len(values) != 1 or not values[0]:
        raise ValueError(name + " must occur once and be non-empty")
    return values[0]


def _query_integer(query: Mapping[str, list], name: str, default: int) -> int:
    value = _single_query(query, name, default=str(default))
    try:
        result = int(value)
    except ValueError as exc:
        raise ValueError(name + " must be an integer") from exc
    if result < 0:
        raise ValueError(name + " must not be negative")
    return result


if __name__ == "__main__":
    raise SystemExit(main())
