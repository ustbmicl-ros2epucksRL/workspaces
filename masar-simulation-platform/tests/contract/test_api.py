from pathlib import Path
import time

from fastapi.testclient import TestClient

from masar_platform.api import create_api_app
from tests.platform_fakes import build_test_orchestrator


def _wait_for_state(client: TestClient, run_id: str, expected: str, timeout: float = 3.0) -> dict:
    deadline = time.monotonic() + timeout
    last: dict = {}
    while time.monotonic() < deadline:
        response = client.get(f"/api/v1/runs/{run_id}")
        assert response.status_code == 200, response.text
        last = response.json()
        if last["state"] == expected:
            return last
        time.sleep(0.01)
    raise AssertionError(f"run did not reach {expected}: {last}")


def test_capabilities_expose_three_workspaces_and_simultaneous_panel_contract(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    with TestClient(create_api_app(orchestrator), raise_server_exceptions=False) as client:
        capabilities = client.get("/api/v1/capabilities")
        scenarios = client.get("/api/v1/scenarios")

    assert capabilities.status_code == 200
    assert [item["id"] for item in capabilities.json()["workspaces"]] == [
        "group_behavior",
        "exploration",
        "rescue",
    ]
    by_id = {item["scenario_id"]: item for item in scenarios.json()["scenarios"]}
    assert "rviz" not in by_id["swarm.webots.ros2swarm"]["panels"]
    for scenario_id in ("exploration.gazebo.tepper", "exploration.webots.robosar"):
        assert {"simulation", "rviz", "map", "sensors"} <= set(by_id[scenario_id]["panels"])
    assert {"simulation", "rviz", "map", "sensors"} <= set(
        by_id["rescue.gazebo.tepper-response"]["panels"]
    )


def test_scenario_validation_is_static_and_returns_runtime_invocations(tmp_path: Path) -> None:
    orchestrator, agents, _ = build_test_orchestrator(tmp_path)
    with TestClient(create_api_app(orchestrator), raise_server_exceptions=False) as client:
        response = client.post(
            "/api/v1/scenarios/swarm.webots.ros2swarm/validate",
            json={"parameters": {"number_robots": 3, "robot": "e_puck", "sensor_type": "ir"}},
        )

    assert response.status_code == 200
    payload = response.json()
    assert payload["valid"] is True
    assert payload["runtime"]["ros2_domain_id"] == 42
    assert payload["steps"][0]["invocation"]["environment"]["MASAR_PLATFORM_MODE"] == "1"
    assert agents["ros1"].processes == {}
    assert agents["ros2"].processes == {}


def test_create_single_run_events_commands_artifacts_and_stop(tmp_path: Path) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path)
    with TestClient(create_api_app(orchestrator), raise_server_exceptions=False) as client:
        created = client.post(
            "/api/v1/runs",
            json={"scenario_id": "exploration.gazebo.tepper", "parameters": {}},
        )
        assert created.status_code == 202, created.text
        run_id = created.json()["run_id"]
        running = _wait_for_state(client, run_id, "RUNNING")
        assert running["reason_code"] is None

        conflict = client.post(
            "/api/v1/runs",
            json={"scenario_id": "control.gazebo.response-controller", "parameters": {}},
        )
        assert conflict.status_code == 409
        assert conflict.json()["error"]["code"] == "RUN_ALREADY_ACTIVE"
        assert conflict.json()["error"]["details"]["run_id"] == run_id
        assert conflict.json()["request_id"].startswith("request-")

        command = client.post(f"/api/v1/runs/{run_id}/mission-commands/start")
        assert command.status_code == 200
        assert command.json()["action"] == "ros1.mission.start"

        events = client.get(f"/api/v1/runs/{run_id}/events?after=0")
        assert events.status_code == 200
        sequences = [event["sequence"] for event in events.json()["events"]]
        assert sequences == sorted(sequences)
        assert any(event["kind"] == "COMMAND_PUBLISHED" for event in events.json()["events"])

        artifacts = client.get(f"/api/v1/runs/{run_id}/artifacts")
        assert artifacts.status_code == 200
        listed = artifacts.json()["artifacts"]
        assert {item["filename"] for item in listed} == {"request.json", "resolved-scenario.json"}
        downloaded = client.get(
            f"/api/v1/runs/{run_id}/artifacts/{listed[0]['artifact_id']}/download"
        )
        assert downloaded.status_code == 200
        assert downloaded.content

        stopped = client.post(f"/api/v1/runs/{run_id}/stop")
        assert stopped.status_code == 200
        assert stopped.json()["state"] == "COMPLETED"
        assert stopped.json()["reason_code"] == "USER_STOP"
        current = client.get("/api/v1/runs/current")
        assert current.json() == {"run": None}

    assert "command:ros1:ros1.mission.start" in trace


def test_deform_home_is_distinct_from_response_return_home(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    with TestClient(create_api_app(orchestrator), raise_server_exceptions=False) as client:
        created = client.post(
            "/api/v1/runs",
            json={"scenario_id": "control.gazebo.response-deform", "parameters": {}},
        )
        run_id = created.json()["run_id"]
        _wait_for_state(client, run_id, "RUNNING")

        rejected = client.post(f"/api/v1/runs/{run_id}/mission-commands/home")
        response_home = client.post(f"/api/v1/runs/{run_id}/response-commands/return-home")
        client.post(f"/api/v1/runs/{run_id}/stop")

    assert rejected.status_code == 409
    assert rejected.json()["error"]["code"] == "COMMAND_NOT_SUPPORTED"
    assert response_home.status_code == 200
    assert response_home.json()["action"] == "ros1.response.return_home"


def test_invalid_parameters_and_request_shape_use_the_frozen_error_body(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    with TestClient(create_api_app(orchestrator), raise_server_exceptions=False) as client:
        invalid_parameters = client.post(
            "/api/v1/scenarios/swarm.webots.ros2swarm/validate",
            json={"parameters": {"number_robots": 0}},
        )
        invalid_shape = client.post(
            "/api/v1/runs",
            json={"scenario_id": "swarm.webots.ros2swarm", "parameters": {}, "raw_shell": "rm"},
        )

    assert invalid_parameters.status_code == 422
    assert invalid_parameters.json()["error"]["code"] == "INVALID_PARAMETERS"
    assert set(invalid_parameters.json()) == {"error", "request_id"}
    assert invalid_shape.status_code == 422
    assert invalid_shape.json()["error"]["code"] == "INVALID_REQUEST"


def test_agent_unavailable_returns_503_and_creates_no_run(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path, ros1_health=False, real_preflight=True)
    with TestClient(create_api_app(orchestrator), raise_server_exceptions=False) as client:
        response = client.post(
            "/api/v1/runs",
            json={"scenario_id": "control.gazebo.response-deform", "parameters": {}},
        )
        runs = client.get("/api/v1/runs")

    assert response.status_code == 503
    assert response.json()["error"]["code"] == "AGENT_UNAVAILABLE"
    assert runs.json() == {"runs": []}
