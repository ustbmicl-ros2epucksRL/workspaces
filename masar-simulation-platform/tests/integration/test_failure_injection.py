from __future__ import annotations

from pathlib import Path
import shutil
from threading import Event

import pytest

from masar_platform.agents import AgentUnavailable, SecretFileError
from masar_platform.api import build_orchestrator
from masar_platform.config import Settings
from masar_platform.gateways import check_web_gateway_prerequisites
from masar_platform.orchestrator import PreflightChecker, PreflightFailed
from masar_platform.state_machine import ReasonCode, RunState
from tests.platform_fakes import build_test_orchestrator


ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize(
    ("scenario_id", "expected_ports"),
    (
        ("swarm.webots.ros2swarm", [18_112]),
        ("exploration.webots.robosar", [18_111, 18_112]),
        ("rescue.gazebo.tepper-response", [18_111]),
    ),
)
def test_web_gateway_preflight_selects_only_scenario_graphs(
    tmp_path: Path,
    scenario_id: str,
    expected_ports: list[int],
) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    observed: list[int] = []

    ready, detail = check_web_gateway_prerequisites(
        orchestrator.catalog.get(scenario_id),
        probe=lambda port: observed.append(port) is None,
    )

    assert ready is True
    assert observed == expected_ports
    assert "rosbridge ready" in detail


def test_software_webots_renderer_fails_before_process_start(tmp_path: Path) -> None:
    orchestrator, agents, trace = build_test_orchestrator(tmp_path, real_preflight=True)
    agents["ros2"].webots_preflight = lambda _run_id, **_kwargs: {  # type: ignore[method-assign]
        "ready": True,
        "display": ":103",
        "renderer": "llvmpipe (LLVM 15.0.7, 256 bits)",
    }

    with pytest.raises(PreflightFailed) as captured:
        orchestrator.create_run("swarm.webots.ros2swarm", {})

    assert captured.value.status_code == 422
    assert captured.value.details["check"] == "webots_renderer"
    assert captured.value.details["result"]["renderer"].startswith("llvmpipe")
    assert not any(item.startswith("start:") for item in trace)
    assert orchestrator.states.list() == ()


def test_missing_required_agent_fails_without_durable_run(tmp_path: Path) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path, real_preflight=True)
    orchestrator.preflight.agents.pop("ros2")

    with pytest.raises(PreflightFailed) as captured:
        orchestrator.create_run("swarm.webots.ros2swarm", {})

    assert captured.value.code == "AGENT_UNAVAILABLE"
    assert captured.value.status_code == 503
    assert not any(item.startswith("start:") for item in trace)
    assert orchestrator.states.list() == ()


def test_readiness_timeout_rolls_back_and_archives_failure(tmp_path: Path) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path, ros1_ready=False)
    run = orchestrator.create_run("exploration.gazebo.tepper", {})

    failed = orchestrator.start_run(run.run_id)

    assert failed.state is RunState.FAILED
    assert failed.reason_code is ReasonCode.READINESS_TIMEOUT
    assert "stop:ros1:exploration_gazebo" in trace
    assert (orchestrator.artifacts.run_dir(run.run_id) / "manifest.json").is_file()


def test_agent_disconnect_during_monitor_fails_and_attempts_owned_stop(tmp_path: Path) -> None:
    orchestrator, agents, trace = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    assert orchestrator.start_run(run.run_id).state is RunState.RUNNING

    agents["ros1"].probe = lambda _run_id: (_ for _ in ()).throw(AgentUnavailable("injected disconnect"))  # type: ignore[method-assign]

    class OneIteration(Event):
        calls = 0

        def wait(self, timeout=None):
            self.calls += 1
            return self.calls > 1

    orchestrator._monitor(run.run_id, OneIteration())

    failed = orchestrator.states.get(run.run_id)
    assert failed.state is RunState.FAILED
    assert failed.reason_code is ReasonCode.NEEDS_INSPECTION
    assert "stop:ros1:response_demo" in trace


def test_disconnected_rosbridge_fails_observability_preflight(tmp_path: Path) -> None:
    orchestrator, agents, trace = build_test_orchestrator(tmp_path)
    orchestrator.preflight = PreflightChecker(
        tmp_path / "runs",
        agents,
        gateway_check=lambda _manifest: (False, "ROS1 rosbridge is unavailable"),
    )

    with pytest.raises(PreflightFailed) as captured:
        orchestrator.create_run("exploration.gazebo.tepper", {})

    assert captured.value.code == "WEB_GATEWAY_UNAVAILABLE"
    assert captured.value.status_code == 503
    assert captured.value.details["check"] == "web_gateway"
    assert not any(item.startswith("start:") for item in trace)


def test_disk_below_manifest_minimum_fails_before_agent_contact(tmp_path: Path, monkeypatch) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path, real_preflight=True)
    monkeypatch.setattr(shutil, "disk_usage", lambda _path: type("Usage", (), {"free": 1024})())

    with pytest.raises(PreflightFailed) as captured:
        orchestrator.create_run("control.gazebo.response-deform", {})

    assert captured.value.details["check"] == "disk_space"
    assert captured.value.details["available_bytes"] == 1024
    assert not trace


def test_unexpected_process_exit_during_readiness_is_archived(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path, ros1_exit=True)
    run = orchestrator.create_run("control.gazebo.response-controller", {})

    failed = orchestrator.start_run(run.run_id)

    assert failed.state is RunState.FAILED
    assert failed.reason_code is ReasonCode.UNEXPECTED_PROCESS_EXIT
    manifest = (orchestrator.artifacts.run_dir(run.run_id) / "manifest.json").read_text(encoding="utf-8")
    assert "UNEXPECTED_PROCESS_EXIT" in manifest


def test_restart_with_missing_secret_never_stops_unproven_processes(tmp_path: Path) -> None:
    original, _, trace = build_test_orchestrator(tmp_path, owner_id="old-api")
    run = original.create_run("control.gazebo.response-deform", {})
    assert original.start_run(run.run_id).state is RunState.RUNNING
    settings = Settings(
        data_dir=tmp_path,
        scenario_dir=ROOT / "scenarios",
        secret_path=tmp_path / "missing-agent.secret",
        visual_stack_ready=True,
    )
    with pytest.raises(SecretFileError, match="does not exist"):
        build_orchestrator(settings)

    assert [item for item in trace if item.startswith("stop:")] == []
