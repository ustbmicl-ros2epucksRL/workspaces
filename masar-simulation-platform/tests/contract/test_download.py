from __future__ import annotations

from pathlib import Path
import threading

from fastapi.testclient import TestClient

from masar_platform.api import create_api_app
from tests.platform_fakes import build_test_orchestrator


def test_artifact_download_streams_full_and_single_range_with_checksum_headers(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    payload = orchestrator.artifacts.run_dir(run.run_id) / "large.bin"
    payload.write_bytes(b"0123456789")
    artifact = orchestrator.artifacts.register(run.run_id, "test", payload)
    url = f"/api/v1/runs/{run.run_id}/artifacts/{artifact.artifact_id}/download"

    with TestClient(create_api_app(orchestrator), raise_server_exceptions=False) as client:
        complete = client.get(url)
        partial = client.get(url, headers={"Range": "bytes=2-5"})

    assert complete.status_code == 200
    assert complete.content == b"0123456789"
    assert complete.headers["accept-ranges"] == "bytes"
    assert complete.headers["x-artifact-checksum"] == artifact.checksum
    assert partial.status_code == 206
    assert partial.content == b"2345"
    assert partial.headers["content-range"] == "bytes 2-5/10"
    assert partial.headers["accept-ranges"] == "bytes"
    assert partial.headers["x-artifact-checksum"] == artifact.checksum


def test_download_rejects_multi_range_tampered_path_and_symlink(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    payload = orchestrator.artifacts.run_dir(run.run_id) / "safe.bin"
    payload.write_bytes(b"safe-data")
    artifact = orchestrator.artifacts.register(run.run_id, "test", payload)
    url = f"/api/v1/runs/{run.run_id}/artifacts/{artifact.artifact_id}/download"
    outside = tmp_path / "outside.bin"
    outside.write_bytes(b"outside")

    with TestClient(create_api_app(orchestrator), raise_server_exceptions=False) as client:
        multiple = client.get(url, headers={"Range": "bytes=0-1,3-4"})
        with orchestrator.artifacts.database.transaction(immediate=True) as connection:
            connection.execute(
                "UPDATE artifacts SET path = ? WHERE run_id = ? AND artifact_id = ?",
                (str(outside), run.run_id, artifact.artifact_id),
            )
        escaped = client.get(url)
        with orchestrator.artifacts.database.transaction(immediate=True) as connection:
            connection.execute(
                "UPDATE artifacts SET path = ? WHERE run_id = ? AND artifact_id = ?",
                (str(payload), run.run_id, artifact.artifact_id),
            )
        payload.unlink()
        payload.symlink_to(outside)
        linked = client.get(url)

    assert multiple.status_code == 400
    assert escaped.status_code == 404
    assert linked.status_code == 404


def test_download_rejects_content_changed_after_registration(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    payload = orchestrator.artifacts.run_dir(run.run_id) / "immutable.bin"
    payload.write_bytes(b"original")
    artifact = orchestrator.artifacts.register(run.run_id, "test", payload)
    payload.write_bytes(b"tampered")

    with TestClient(create_api_app(orchestrator), raise_server_exceptions=False) as client:
        response = client.get(f"/api/v1/runs/{run.run_id}/artifacts/{artifact.artifact_id}/download")

    assert response.status_code == 404


def test_artifact_open_and_hash_runs_outside_the_api_event_loop(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    payload = orchestrator.artifacts.run_dir(run.run_id) / "threaded.bin"
    payload.write_bytes(b"threaded")
    artifact = orchestrator.artifacts.register(run.run_id, "test", payload)
    application = create_api_app(orchestrator)
    event_loop_threads: list[int] = []
    open_threads: list[int] = []
    original_open = orchestrator.artifacts.open

    @application.middleware("http")
    async def capture_event_loop_thread(request, call_next):
        event_loop_threads.append(threading.get_ident())
        return await call_next(request)

    def tracked_open(run_id: str, artifact_id: str):
        open_threads.append(threading.get_ident())
        return original_open(run_id, artifact_id)

    orchestrator.artifacts.open = tracked_open  # type: ignore[method-assign]
    with TestClient(application, raise_server_exceptions=False) as client:
        response = client.get(f"/api/v1/runs/{run.run_id}/artifacts/{artifact.artifact_id}/download")

    assert response.status_code == 200
    assert open_threads and event_loop_threads
    assert open_threads[0] != event_loop_threads[0]
