from __future__ import annotations

from http.client import HTTPConnection
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import sys
from threading import Event, Thread
import time
from typing import Any

import pytest

from agents.runtime_agent.main import AgentApplication, create_server
from agents.runtime_agent.processes import CommandPolicy, ProcessManager
from agents.runtime_agent.protocol import signed_headers
from masar_platform.agents import AgentClient, AgentHTTPError


MASTER_SECRET = b"s" * 32


class _FakeActionPublisher:
    calls: list[tuple[str, dict[str, Any]]] = []

    def publish(self, action: str, runtime: dict[str, Any]) -> dict[str, Any]:
        type(self).calls.append((action, runtime))
        if action not in {"ros1.mission.start", "ros1.mission.stop"}:
            from agents.runtime_agent.actions import ActionNotAllowed

            raise ActionNotAllowed("action is not allowlisted")
        return {
            "action": action,
            "topic": "/system_mission_command",
            "message_type": "robosar_messages/mission_command",
            "published": True,
        }


def _request(
    port: int,
    method: str,
    target: str,
    *,
    sequence: int | None = None,
    payload: dict[str, Any] | None = None,
) -> tuple[int, dict[str, Any]]:
    body = b"" if payload is None else json.dumps(payload, separators=(",", ":")).encode()
    headers: dict[str, str] = {"Content-Type": "application/json"}
    if sequence is not None:
        headers.update(signed_headers(MASTER_SECRET, "run-1", sequence, method, target, body))
    connection = HTTPConnection("127.0.0.1", port, timeout=3)
    connection.request(method, target, body=body, headers=headers)
    response = connection.getresponse()
    parsed = json.loads(response.read())
    connection.close()
    return response.status, parsed


@pytest.fixture
def running_agent(tmp_path: Path) -> tuple[int, ProcessManager]:
    manager = ProcessManager(
        agent_kind="test",
        workspace=tmp_path,
        runs_dir=tmp_path / "runs",
        command_policies={"test.python": CommandPolicy("test", sys.executable)},
    )
    _FakeActionPublisher.calls = []
    app = AgentApplication("test", MASTER_SECRET, manager, action_publisher=_FakeActionPublisher())
    server = create_server("127.0.0.1", 0, app)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server.server_port, manager
    finally:
        for record in manager.probe("run-1"):
            if record.status == "RUNNING":
                manager.stop("run-1", record.process_id)
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)


def test_health_is_loopback_safe_and_contains_no_secret(running_agent: tuple[int, ProcessManager]) -> None:
    port, _ = running_agent

    status, body = _request(port, "GET", "/healthz")

    assert status == 200
    assert body["ok"] is True
    assert body["data"] == {
        "agent": "test",
        "protocol_version": 1,
        "status": "ok",
        "capabilities": [],
    }
    assert MASTER_SECRET.decode() not in json.dumps(body)


def test_v1_endpoint_rejects_missing_auth(running_agent: tuple[int, ProcessManager]) -> None:
    port, _ = running_agent

    status, body = _request(port, "GET", "/v1/runs/run-1/probe")

    assert status == 401
    assert body["error"]["code"] == "AUTH_MISSING_HEADER"


def test_start_probe_logs_and_stop_contract(running_agent: tuple[int, ProcessManager]) -> None:
    port, _ = running_agent
    start_target = "/v1/runs/run-1/start"
    payload = {
        "process_id": "step-1",
        "command_ref": "test.python",
        "argv": [sys.executable, "-c", "import time; print('online', flush=True); time.sleep(30)"],
        "environment": {},
    }

    status, started = _request(port, "POST", start_target, sequence=1, payload=payload)
    assert status == 202
    assert started["data"]["run_id"] == "run-1"
    assert started["data"]["process_id"] == "step-1"

    status, probed = _request(port, "GET", "/v1/runs/run-1/probe", sequence=2)
    assert status == 200
    assert probed["data"]["processes"][0]["pgid"] == started["data"]["pgid"]

    deadline = time.monotonic() + 3
    logs: dict[str, Any] = {}
    sequence = 3
    while time.monotonic() < deadline:
        status, logs = _request(
            port,
            "GET",
            "/v1/runs/run-1/logs?process_id=step-1&stream=stdout&after=0",
            sequence=sequence,
        )
        if "online" in logs.get("data", {}).get("data", ""):
            break
        time.sleep(0.01)
        sequence += 1
    assert status == 200
    assert "online" in logs["data"]["data"]

    stop_payload = {"process_id": "step-1", "grace_seconds": {"sigint": 0.1, "sigterm": 0.1, "sigkill": 0.1}}
    status, stopped = _request(port, "POST", "/v1/runs/run-1/stop", sequence=sequence + 1, payload=stop_payload)
    assert status == 200
    assert stopped["data"]["status"] == "STOPPED"


def test_invalid_command_ref_returns_422_without_starting(running_agent: tuple[int, ProcessManager]) -> None:
    port, manager = running_agent
    payload = {
        "process_id": "step-1",
        "command_ref": "shell.anything",
        "argv": ["/bin/sh", "-c", "echo unsafe"],
        "environment": {},
    }

    status, body = _request(port, "POST", "/v1/runs/run-1/start", sequence=1, payload=payload)

    assert status == 422
    assert body["error"]["code"] == "COMMAND_NOT_ALLOWED"
    assert manager.probe("run-1") == []
    assert MASTER_SECRET.decode() not in json.dumps(body)


def test_signed_readiness_probe_returns_process_and_exact_probe_results(
    running_agent: tuple[int, ProcessManager],
) -> None:
    port, manager = running_agent
    marker = manager.workspace / "ready.flag"
    marker.write_text("ready", encoding="utf-8")
    payload = {
        "runtime": {"ros1_master_uri": "http://127.0.0.1:11311", "ros2_domain_id": 42},
        "probes": [{"type": "file", "name": "ready.flag", "required": True}],
    }

    status, body = _request(port, "POST", "/v1/runs/run-1/probe", sequence=1, payload=payload)

    assert status == 200
    assert body["data"]["processes"] == []
    assert body["data"]["probes"][0]["status"] == "READY"
    assert body["data"]["probes"][0]["ready"] is True

    client = AgentClient("127.0.0.1", port, MASTER_SECRET, retry_attempts=0)
    result = client.probe_readiness("run-2", payload)
    assert result["probes"][0]["status"] == "READY"


def test_readiness_probe_rejects_missing_ros_message_type(
    running_agent: tuple[int, ProcessManager],
) -> None:
    port, _ = running_agent
    payload = {
        "runtime": {"ros1_master_uri": "http://127.0.0.1:11311", "ros2_domain_id": 42},
        "probes": [{"type": "ros1_topic", "name": "/clock"}],
    }

    status, body = _request(port, "POST", "/v1/runs/run-1/probe", sequence=1, payload=payload)

    assert status == 422
    assert body["error"]["code"] == "INVALID_REQUEST"


def test_signed_fixed_command_route_accepts_only_an_action_and_runtime(
    running_agent: tuple[int, ProcessManager],
) -> None:
    port, _ = running_agent
    payload = {
        "action": "ros1.mission.start",
        "runtime": {"ros1_master_uri": "http://127.0.0.1:11311", "ros2_domain_id": 42},
    }

    status, body = _request(port, "POST", "/v1/runs/run-1/command", sequence=1, payload=payload)

    assert status == 200
    assert body["data"]["published"] is True
    assert body["data"]["run_id"] == "run-1"
    assert _FakeActionPublisher.calls == [(payload["action"], payload["runtime"])]

    payload["action"] = "ros1.arbitrary"
    status, body = _request(port, "POST", "/v1/runs/run-1/command", sequence=2, payload=payload)
    assert status == 422
    assert body["error"]["code"] == "ACTION_NOT_ALLOWED"


def test_signed_graph_snapshot_route_and_host_client(tmp_path: Path) -> None:
    class GraphRunner:
        def graph_snapshot(self, runtime: dict[str, Any]) -> dict[str, Any]:
            assert runtime["ros2_domain_id"] == 42
            return {
                "agent": "ros2",
                "available": True,
                "nodes": ["/node_a"],
                "topics": [{"name": "/topic_a", "message_type": "std_msgs/msg/String"}],
            }

    manager = ProcessManager("ros2", tmp_path, tmp_path / "runs")
    app = AgentApplication("ros2", MASTER_SECRET, manager, probe_runner=GraphRunner())  # type: ignore[arg-type]
    server = create_server("127.0.0.1", 0, app)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = AgentClient("127.0.0.1", server.server_port, MASTER_SECRET, retry_attempts=0)
    try:
        snapshot = client.graph_snapshot(
            "run-graph",
            {"runtime": {"ros1_master_uri": "http://127.0.0.1:11311", "ros2_domain_id": 42}},
        )
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)

    assert snapshot["agent"] == "ros2"
    assert snapshot["topics"][0]["name"] == "/topic_a"


def test_signed_webots_preflight_route_and_host_client(tmp_path: Path) -> None:
    class WebotsProbeRunner:
        def webots_preflight(self, *, require_bridge: bool) -> dict[str, Any]:
            assert require_bridge is True
            return {
                "ready": True,
                "display": ":103",
                "renderer": "NVIDIA RTX 6000 Ada Generation/PCIe/SSE2",
                "bridge_setup": "/root/ros-humble-ros1-bridge/install/setup.bash",
                "bridge_executable": (
                    "/root/ros-humble-ros1-bridge/install/ros1_bridge/lib/ros1_bridge/dynamic_bridge"
                ),
            }

    manager = ProcessManager("ros2", tmp_path, tmp_path / "runs")
    app = AgentApplication("ros2", MASTER_SECRET, manager, probe_runner=WebotsProbeRunner())  # type: ignore[arg-type]
    server = create_server("127.0.0.1", 0, app)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = AgentClient("127.0.0.1", server.server_port, MASTER_SECRET, retry_attempts=0)
    try:
        result = client.webots_preflight("preflight-webots", require_bridge=True)
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)

    assert result["ready"] is True
    assert result["display"] == ":103"
    assert result["renderer"].startswith("NVIDIA")


class _RetryHandler(BaseHTTPRequestHandler):
    counts = {"start": 0, "probe": 0}
    sequences: list[int] = []

    def do_POST(self) -> None:  # noqa: N802
        type(self).counts["start"] += 1
        self._reply(503, {"ok": False, "error": {"code": "UNAVAILABLE", "message": "retry", "details": {}}})

    def do_GET(self) -> None:  # noqa: N802
        type(self).counts["probe"] += 1
        type(self).sequences.append(int(self.headers["X-Platform-Sequence"]))
        if type(self).counts["probe"] == 1:
            self._reply(503, {"ok": False, "error": {"code": "UNAVAILABLE", "message": "retry", "details": {}}})
        else:
            self._reply(200, {"ok": True, "data": {"processes": []}})

    def _reply(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:
        return


def test_host_client_retries_probe_but_never_start() -> None:
    _RetryHandler.counts = {"start": 0, "probe": 0}
    _RetryHandler.sequences = []
    server = ThreadingHTTPServer(("127.0.0.1", 0), _RetryHandler)
    thread = Thread(target=server.serve_forever, daemon=True)
    thread.start()
    client = AgentClient("127.0.0.1", server.server_port, MASTER_SECRET, retry_attempts=1)
    try:
        with pytest.raises(AgentHTTPError):
            client.start("run-1", {"process_id": "step-1", "command_ref": "test.python", "argv": []})
        assert _RetryHandler.counts["start"] == 1

        assert client.probe("run-1") == {"processes": []}
        assert _RetryHandler.counts["probe"] == 2
        assert _RetryHandler.sequences == sorted(_RetryHandler.sequences)
        assert len(set(_RetryHandler.sequences)) == 2
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=3)


class _ConcurrentSequenceHandler(BaseHTTPRequestHandler):
    first_received = Event()
    second_received = Event()
    accepted_sequences: list[int] = []

    def do_GET(self) -> None:  # noqa: N802
        sequence = int(self.headers["X-Platform-Sequence"])
        if sequence == 1:
            type(self).first_received.set()
            type(self).second_received.wait(timeout=1)
        elif sequence == 2:
            type(self).second_received.set()
        type(self).accepted_sequences.append(sequence)
        body = json.dumps({"ok": True, "data": {"processes": []}}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:
        return


def test_host_client_serializes_signed_requests_in_sequence_order() -> None:
    _ConcurrentSequenceHandler.first_received = Event()
    _ConcurrentSequenceHandler.second_received = Event()
    _ConcurrentSequenceHandler.accepted_sequences = []
    server = ThreadingHTTPServer(("127.0.0.1", 0), _ConcurrentSequenceHandler)
    server_thread = Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    client = AgentClient("127.0.0.1", server.server_port, MASTER_SECRET, retry_attempts=0)
    errors: list[BaseException] = []

    def probe() -> None:
        try:
            client.probe("run-concurrent")
        except BaseException as exc:  # pragma: no cover - surfaced by the assertion below
            errors.append(exc)

    first = Thread(target=probe)
    second = Thread(target=probe)
    try:
        first.start()
        assert _ConcurrentSequenceHandler.first_received.wait(timeout=2)
        second.start()
        first.join(timeout=3)
        second.join(timeout=3)
    finally:
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=3)

    assert not errors
    assert not first.is_alive() and not second.is_alive()
    assert _ConcurrentSequenceHandler.accepted_sequences == [1, 2]


def test_host_client_recovers_sequence_after_control_plane_restart(
    running_agent: tuple[int, ProcessManager],
) -> None:
    port, _ = running_agent
    run_id = "run-client-restart"

    first_client = AgentClient("127.0.0.1", port, MASTER_SECRET, retry_attempts=0)
    assert first_client.probe(run_id)["run_id"] == run_id

    restarted_client = AgentClient("127.0.0.1", port, MASTER_SECRET, retry_attempts=0)
    assert restarted_client.probe(run_id)["run_id"] == run_id


class _SlowStopHandler(BaseHTTPRequestHandler):
    def do_POST(self) -> None:  # noqa: N802
        time.sleep(0.15)
        body = json.dumps({"ok": True, "data": {"status": "STOPPED"}}).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:
        return


def test_host_client_stop_timeout_covers_declared_grace_periods() -> None:
    server = ThreadingHTTPServer(("127.0.0.1", 0), _SlowStopHandler)
    server_thread = Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    client = AgentClient(
        "127.0.0.1",
        server.server_port,
        MASTER_SECRET,
        read_timeout=0.05,
        retry_attempts=0,
    )
    try:
        stopped = client.stop(
            "run-slow-stop",
            {
                "process_id": "simulation",
                "grace_seconds": {"sigint": 0.2, "sigterm": 0.0, "sigkill": 0.0},
            },
        )
    finally:
        server.shutdown()
        server.server_close()
        server_thread.join(timeout=3)

    assert stopped["status"] == "STOPPED"
