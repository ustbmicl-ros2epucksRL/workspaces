from pathlib import Path

from fastapi.testclient import TestClient

from masar_platform.api import create_api_app
from tests.platform_fakes import build_test_orchestrator


def test_event_websocket_replays_from_cursor_and_rest_recovers_the_same_sequence(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-controller", {})

    with TestClient(create_api_app(orchestrator), raise_server_exceptions=False) as client:
        with client.websocket_connect(f"/api/v1/events?run_id={run.run_id}&after=0") as websocket:
            envelope = websocket.receive_json()
        rest = client.get(f"/api/v1/runs/{run.run_id}/events?after=0")

    assert envelope["type"] == "event"
    assert envelope["event"]["sequence"] == 1
    assert rest.status_code == 200
    assert rest.json()["events"][0] == envelope["event"]


def test_event_websocket_rejects_missing_run_and_negative_cursor(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    with TestClient(create_api_app(orchestrator), raise_server_exceptions=False) as client:
        with client.websocket_connect("/api/v1/events?run_id=missing&after=-1") as websocket:
            envelope = websocket.receive_json()

    assert envelope == {"type": "error", "code": "INVALID_EVENT_CURSOR"}
