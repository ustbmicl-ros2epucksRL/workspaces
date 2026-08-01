from pathlib import Path

from masar_platform.orchestrator import PreflightFailed
from masar_platform.state_machine import ReasonCode, RunState
from tests.platform_fakes import build_test_orchestrator


def test_preflight_checks_have_stable_order_before_any_run_is_created(tmp_path: Path) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path, real_preflight=True)
    manifest = orchestrator.catalog.get("control.gazebo.response-deform")

    checks = orchestrator.preflight.check(manifest)

    assert [check.name for check in checks] == ["data_directory", "disk_space", "agent_ros1", "display"]
    assert trace == ["health:ros1"]
    assert orchestrator.states.current() is None


def test_webots_starts_ros2_then_ros1_and_stops_in_manifest_reverse_order(tmp_path: Path) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("exploration.webots.robosar", {})

    running = orchestrator.start_run(run.run_id)

    assert running.state is RunState.RUNNING
    assert [item for item in trace if item.startswith("start:")] == [
        "start:ros2:ros2_webots_bridge",
        "start:ros1:ros1_robosar",
    ]
    completed = orchestrator.stop_run(run.run_id)
    assert completed.state is RunState.COMPLETED
    assert completed.reason_code is ReasonCode.USER_STOP
    assert [item for item in trace if item.startswith("stop:")] == [
        "stop:ros1:ros1_robosar",
        "stop:ros2:ros2_webots_bridge",
    ]


def test_second_step_readiness_timeout_rolls_back_both_steps_in_reverse_order(tmp_path: Path) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path, ros1_ready=False, ros2_ready=True)
    run = orchestrator.create_run("exploration.webots.robosar", {})

    failed = orchestrator.start_run(run.run_id)

    assert failed.state is RunState.FAILED
    assert failed.reason_code is ReasonCode.READINESS_TIMEOUT
    assert [item for item in trace if item.startswith("stop:")] == [
        "stop:ros1:ros1_robosar",
        "stop:ros2:ros2_webots_bridge",
    ]


def test_process_exit_before_readiness_is_an_unexpected_exit(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path, ros1_exit=True)
    run = orchestrator.create_run("control.gazebo.response-deform", {})

    failed = orchestrator.start_run(run.run_id)

    assert failed.state is RunState.FAILED
    assert failed.reason_code is ReasonCode.UNEXPECTED_PROCESS_EXIT


def test_unavailable_required_agent_fails_preflight_without_durable_run(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path, ros1_health=False, real_preflight=True)

    try:
        orchestrator.create_run("control.gazebo.response-deform", {})
    except PreflightFailed as exc:
        assert exc.status_code == 503
        assert exc.code == "AGENT_UNAVAILABLE"
    else:
        raise AssertionError("preflight unexpectedly passed")

    assert orchestrator.states.current() is None
    assert orchestrator.states.list() == ()


def test_restart_without_agent_ownership_marks_run_needs_inspection_without_stop(tmp_path: Path) -> None:
    original, _, trace = build_test_orchestrator(tmp_path, owner_id="old-api")
    run = original.create_run("control.gazebo.response-deform", {})
    restarted, agents, _ = build_test_orchestrator(tmp_path, owner_id="new-api")
    agents["ros1"].health_ok = False

    recovered = restarted.recover_active_runs()

    assert recovered[0].run_id == run.run_id
    assert recovered[0].state is RunState.FAILED
    assert recovered[0].reason_code is ReasonCode.NEEDS_INSPECTION
    assert not any(item.startswith("stop:") for item in trace)


def test_restart_adopts_a_fully_proven_running_agent_tree(tmp_path: Path) -> None:
    original, original_agents, _ = build_test_orchestrator(tmp_path, owner_id="old-api")
    run = original.create_run("exploration.webots.robosar", {})
    assert original.start_run(run.run_id).state is RunState.RUNNING
    restarted, _, _ = build_test_orchestrator(tmp_path, owner_id="new-api")
    restarted.agents = original_agents

    recovered = restarted.recover_active_runs()

    assert recovered[0].state is RunState.RUNNING
    assert restarted.leases.get(run.run_id).owner_id == "new-api"
    assert restarted.stop_run(run.run_id).state is RunState.COMPLETED


def test_fixed_mission_command_mapping_and_deform_home_rejection(tmp_path: Path) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    orchestrator.start_run(run.run_id)

    published = orchestrator.issue_command(run.run_id, "mission.start")

    assert published["action"] == "ros1.mission.start"
    assert "command:ros1:ros1.mission.start" in trace
    from masar_platform.orchestrator import CommandNotSupported

    try:
        orchestrator.issue_command(run.run_id, "mission.home")
    except CommandNotSupported:
        pass
    else:
        raise AssertionError("DEFORM HOME unexpectedly published")
    orchestrator.stop_run(run.run_id)


def test_request_and_resolved_manifest_are_durable_artifacts(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)

    run = orchestrator.create_run("swarm.webots.ros2swarm", {"number_robots": 3})

    persisted = orchestrator.states.get(run.run_id)
    assert persisted.request["parameters"] == {"number_robots": 3}
    assert persisted.resolved_manifest["parameters"]["number_robots"] == 3
    artifacts = orchestrator.artifacts.list(run.run_id)
    assert {artifact.path.name for artifact in artifacts} == {"request.json", "resolved-scenario.json"}
    orchestrator.stop_run(run.run_id)
