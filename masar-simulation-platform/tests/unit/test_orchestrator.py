from pathlib import Path
from threading import Event, Thread
import time

from masar_platform.agents import AgentHTTPError, AgentUnavailable
from masar_platform.leases import LeaseError
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


def test_webots_preflight_checks_nvidia_before_creating_any_run(tmp_path: Path) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path, real_preflight=True)
    swarm = orchestrator.catalog.get("swarm.webots.ros2swarm")
    exploration = orchestrator.catalog.get("exploration.webots.robosar")

    swarm_checks = orchestrator.preflight.check(swarm)
    exploration_checks = orchestrator.preflight.check(exploration)

    assert [item.name for item in swarm_checks][-2:] == ["webots_renderer", "display"]
    assert [item.name for item in exploration_checks][-2:] == ["webots_renderer", "display"]
    assert "webots-preflight:ros2:False" in trace
    assert "webots-preflight:ros2:True" in trace
    assert orchestrator.states.current() is None


def test_restarted_api_uses_a_new_webots_preflight_identity_with_the_same_agent(tmp_path: Path) -> None:
    orchestrator, agents, _ = build_test_orchestrator(tmp_path, real_preflight=True)
    swarm = orchestrator.catalog.get("swarm.webots.ros2swarm")

    orchestrator.preflight.check(swarm)
    restarted_preflight = type(orchestrator.preflight)(tmp_path / "runs", agents)
    restarted_preflight.check(swarm)

    assert len(set(agents["ros2"].webots_preflight_run_ids)) == 2


def test_webots_starts_ros2_then_ros1_and_stops_in_manifest_reverse_order(tmp_path: Path) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("exploration.webots.robosar", {})

    running = orchestrator.start_run(run.run_id)

    assert running.state is RunState.RUNNING
    assert [item for item in trace if item.startswith("start:")] == [
        "start:ros2:ros2_webots_bridge",
        "start:ros1:ros1_robosar",
        "start:ros1:display_rviz",
    ]
    completed = orchestrator.stop_run(run.run_id)
    assert completed.state is RunState.COMPLETED
    assert completed.reason_code is ReasonCode.USER_STOP
    assert [item for item in trace if item.startswith("stop:")] == [
        "stop:ros1:display_rviz",
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


def test_successful_readiness_refreshes_lease_before_running(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    heartbeat_calls: list[str] = []
    original_heartbeat = orchestrator._heartbeat

    def record_heartbeat(run_id: str) -> None:
        heartbeat_calls.append(run_id)
        original_heartbeat(run_id)

    orchestrator._heartbeat = record_heartbeat  # type: ignore[method-assign]
    run = orchestrator.create_run("control.gazebo.response-deform", {})

    running = orchestrator.start_run(run.run_id)

    assert running.state is RunState.RUNNING
    assert heartbeat_calls == [run.run_id, run.run_id]


def test_blocking_readiness_probe_keeps_the_run_lease_alive(tmp_path: Path, monkeypatch) -> None:
    import masar_platform.orchestrator as orchestrator_module

    monkeypatch.setattr(orchestrator_module, "HEARTBEAT_INTERVAL_SECONDS", 0.02)
    orchestrator, agents, _ = build_test_orchestrator(tmp_path)
    entered = Event()
    release = Event()
    original_probe = agents["ros2"].probe_readiness

    def blocking_probe(run_id: str, payload: dict) -> dict:
        entered.set()
        assert release.wait(1)
        return original_probe(run_id, payload)

    agents["ros2"].probe_readiness = blocking_probe  # type: ignore[method-assign]
    run = orchestrator.create_run("swarm.webots.ros2swarm", {"number_robots": 5})
    results = []
    worker = Thread(target=lambda: results.append(orchestrator.start_run(run.run_id)), daemon=True)
    worker.start()
    try:
        assert entered.wait(1)
        heartbeat_at = orchestrator.leases.get(run.run_id).heartbeat_at
        time.sleep(0.08)
        assert orchestrator.leases.get(run.run_id).heartbeat_at > heartbeat_at
    finally:
        release.set()
        worker.join(timeout=1)

    assert [result.state for result in results] == [RunState.RUNNING]


def test_swarm_readiness_requires_every_requested_robot_odom(tmp_path: Path) -> None:
    orchestrator, agents, _ = build_test_orchestrator(tmp_path)
    captured: list[list[str]] = []
    original = agents["ros2"].probe_readiness

    def record_readiness(run_id: str, payload: dict) -> dict:
        captured.append([probe["name"] for probe in payload["probes"]])
        return original(run_id, payload)

    agents["ros2"].probe_readiness = record_readiness  # type: ignore[method-assign]
    run = orchestrator.create_run("swarm.webots.ros2swarm", {"number_robots": 3})

    assert orchestrator.start_run(run.run_id).state is RunState.RUNNING
    assert captured
    assert captured[0] == ["/clock", "/robot_0/odom", "/robot_1/odom", "/robot_2/odom"]


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
    run_dir = restarted.artifacts.run_dir(run.run_id)
    assert (run_dir / "manifest.json").is_file()
    assert "ownership_unproven" in (run_dir / "summary.json").read_text(encoding="utf-8")


def test_monitor_lease_failure_stops_recorders_before_business_and_archives(tmp_path: Path) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run(
        "swarm.webots.ros2swarm",
        {},
        {"enabled": True, "profile": "default"},
    )
    assert orchestrator.start_run(run.run_id).state is RunState.RUNNING

    def fail_heartbeat(_run_id: str) -> None:
        raise LeaseError("injected lease failure")

    class OneIteration(Event):
        calls = 0

        def wait(self, timeout=None):
            self.calls += 1
            return self.calls > 1

    orchestrator._heartbeat = fail_heartbeat  # type: ignore[method-assign]
    orchestrator._monitor(run.run_id, OneIteration())

    assert orchestrator.states.get(run.run_id).reason_code is ReasonCode.NEEDS_INSPECTION
    stops = [item for item in trace if item.startswith("stop:")]
    assert stops[:2] == ["stop:ros2:recorder_ros2", "stop:ros2:swarm_simulation"]
    assert (orchestrator.artifacts.run_dir(run.run_id) / "manifest.json").is_file()


def test_persistent_agent_failure_marks_shutdown_unconfirmed_and_skips_live_bag(tmp_path: Path) -> None:
    orchestrator, agents, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run(
        "swarm.webots.ros2swarm",
        {},
        {"enabled": True, "profile": "default"},
    )
    assert orchestrator.start_run(run.run_id).state is RunState.RUNNING
    bag = orchestrator.artifacts.run_dir(run.run_id) / "bags" / "ros2"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("still being written\n", encoding="utf-8")

    def unavailable(*_args, **_kwargs):
        raise AgentUnavailable("persistent injected failure")

    agents["ros2"].probe = unavailable  # type: ignore[method-assign]
    agents["ros2"].stop = unavailable  # type: ignore[method-assign]

    class OneIteration(Event):
        calls = 0

        def wait(self, timeout=None):
            self.calls += 1
            return self.calls > 1

    orchestrator._monitor(run.run_id, OneIteration())

    summary = (orchestrator.artifacts.run_dir(run.run_id) / "summary.json").read_text(encoding="utf-8")
    assert "shutdown_unconfirmed" in summary
    assert "bag_unsettled:ros2" in summary
    assert not any(item.kind == "ros2_bag" for item in orchestrator.artifacts.list(run.run_id))


def test_recorder_start_rollback_persists_unconfirmed_recorder_stop(tmp_path: Path) -> None:
    orchestrator, agents, trace = build_test_orchestrator(tmp_path)
    original_ros1_stop = agents["ros1"].stop
    original_ros2_start = agents["ros2"].start

    def fail_second_recorder(run_id, payload):
        if payload["process_id"] == "recorder_ros2":
            raise AgentUnavailable("injected recorder start failure")
        return original_ros2_start(run_id, payload)

    def fail_first_recorder_stop(run_id, payload):
        if payload["process_id"] == "recorder_ros1":
            raise AgentUnavailable("injected recorder stop failure")
        return original_ros1_stop(run_id, payload)

    agents["ros2"].start = fail_second_recorder  # type: ignore[method-assign]
    agents["ros1"].stop = fail_first_recorder_stop  # type: ignore[method-assign]
    run = orchestrator.create_run(
        "exploration.webots.robosar",
        {},
        {"enabled": True, "profile": "default"},
    )

    failed = orchestrator.start_run(run.run_id)

    assert failed.state is RunState.FAILED
    summary = (orchestrator.artifacts.run_dir(run.run_id) / "summary.json").read_text(encoding="utf-8")
    assert "shutdown_unconfirmed" in summary
    assert "bag_unsettled:ros1" in summary
    assert [item for item in trace if item.startswith("stop:")] == [
        "stop:ros1:display_rviz",
        "stop:ros1:ros1_robosar",
        "stop:ros2:ros2_webots_bridge",
    ]


def test_recorder_start_rollback_requires_display_ownership_proof(tmp_path: Path) -> None:
    orchestrator, agents, _ = build_test_orchestrator(tmp_path)
    original_ros2_start = agents["ros2"].start

    def lose_display_then_fail_second_recorder(run_id, payload):
        if payload["process_id"] == "recorder_ros2":
            agents["ros1"].processes.pop("display_rviz")
            raise AgentUnavailable("injected recorder start failure")
        return original_ros2_start(run_id, payload)

    agents["ros2"].start = lose_display_then_fail_second_recorder  # type: ignore[method-assign]
    run = orchestrator.create_run(
        "exploration.webots.robosar",
        {},
        {"enabled": True, "profile": "default"},
    )

    failed = orchestrator.start_run(run.run_id)

    assert failed.state is RunState.FAILED
    summary = (orchestrator.artifacts.run_dir(run.run_id) / "summary.json").read_text(encoding="utf-8")
    assert "shutdown_unconfirmed" in summary
    assert "display_rviz:ownership_missing" in summary


def test_display_stop_must_return_a_terminal_status(tmp_path: Path) -> None:
    orchestrator, agents, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("exploration.webots.robosar", {})
    assert orchestrator.start_run(run.run_id).state is RunState.RUNNING

    original_stop = agents["ros1"].stop

    def leave_display_running(run_id, payload):
        if payload["process_id"] == "display_rviz":
            return agents["ros1"].processes["display_rviz"]
        return original_stop(run_id, payload)

    agents["ros1"].stop = leave_display_running  # type: ignore[method-assign]

    stopped = orchestrator.stop_run(run.run_id)

    assert stopped.state is RunState.FAILED
    assert stopped.reason_code is ReasonCode.NEEDS_INSPECTION
    summary = (orchestrator.artifacts.run_dir(run.run_id) / "summary.json").read_text(encoding="utf-8")
    assert "display_rviz:stop_unconfirmed" in summary


def test_definitively_rejected_optional_display_does_not_block_completion(tmp_path: Path) -> None:
    orchestrator, agents, _ = build_test_orchestrator(tmp_path)
    original_start = agents["ros1"].start

    def reject_display(run_id, payload):
        if payload["process_id"] == "display_rviz":
            raise AgentHTTPError(422, "COMMAND_NOT_ALLOWED", "injected display rejection")
        return original_start(run_id, payload)

    agents["ros1"].start = reject_display  # type: ignore[method-assign]
    run = orchestrator.create_run("exploration.webots.robosar", {})

    assert orchestrator.start_run(run.run_id).state is RunState.RUNNING
    agents["ros1"].probe = lambda _run_id: (_ for _ in ()).throw(  # type: ignore[method-assign]
        AgentUnavailable("display probe must be skipped")
    )
    stopped = orchestrator.stop_run(run.run_id)

    assert stopped.state is RunState.COMPLETED
    assert stopped.reason_code is ReasonCode.USER_STOP


def test_recovered_display_attempt_requires_persisted_ownership_proof(tmp_path: Path) -> None:
    original, original_agents, _ = build_test_orchestrator(tmp_path, owner_id="old-api")
    run = original.create_run("exploration.webots.robosar", {})
    assert original.start_run(run.run_id).state is RunState.RUNNING
    restarted, _, _ = build_test_orchestrator(tmp_path, owner_id="new-api")
    restarted.agents = original_agents
    assert restarted.recover_active_runs()[0].state is RunState.RUNNING
    original_agents["ros1"].processes.pop("display_rviz")

    stopped = restarted.stop_run(run.run_id)

    assert stopped.state is RunState.FAILED
    assert stopped.reason_code is ReasonCode.NEEDS_INSPECTION
    summary = (restarted.artifacts.run_dir(run.run_id) / "summary.json").read_text(encoding="utf-8")
    assert "display_rviz:ownership_missing" in summary


def test_recovery_backfills_display_state_for_pre_upgrade_active_run(tmp_path: Path) -> None:
    original, original_agents, _ = build_test_orchestrator(tmp_path, owner_id="old-api")
    run = original.create_run("exploration.webots.robosar", {})
    assert original.start_run(run.run_id).state is RunState.RUNNING
    with original.states.database.transaction(immediate=True) as connection:
        connection.execute(
            "DELETE FROM run_steps WHERE run_id = ? AND step_id = 'display_rviz'",
            (run.run_id,),
        )
    restarted, _, _ = build_test_orchestrator(tmp_path, owner_id="new-api")
    restarted.agents = original_agents

    assert restarted.recover_active_runs()[0].state is RunState.RUNNING
    assert restarted._step_states(run.run_id)["display_rviz"] == "READY"
    original_agents["ros1"].processes.pop("display_rviz")
    stopped = restarted.stop_run(run.run_id)

    assert stopped.state is RunState.FAILED
    assert stopped.reason_code is ReasonCode.NEEDS_INSPECTION
    summary = (restarted.artifacts.run_dir(run.run_id) / "summary.json").read_text(encoding="utf-8")
    assert "display_rviz:ownership_missing" in summary


def test_missing_recorder_ownership_record_blocks_bag_indexing(tmp_path: Path) -> None:
    orchestrator, agents, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run(
        "swarm.webots.ros2swarm",
        {},
        {"enabled": True, "profile": "default"},
    )
    assert orchestrator.start_run(run.run_id).state is RunState.RUNNING
    agents["ros2"].processes.pop("recorder_ros2")
    bag = orchestrator.artifacts.run_dir(run.run_id) / "bags" / "ros2"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("ownership missing\n", encoding="utf-8")

    stopped = orchestrator.stop_run(run.run_id)

    assert stopped.reason_code is ReasonCode.NEEDS_INSPECTION
    summary = (orchestrator.artifacts.run_dir(run.run_id) / "summary.json").read_text(encoding="utf-8")
    assert "shutdown_unconfirmed" in summary
    assert not any(item.kind == "ros2_bag" for item in orchestrator.artifacts.list(run.run_id))


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
    from masar_platform.orchestrator import CommandNotSupported, RunNotActive

    try:
        orchestrator.issue_command(run.run_id, "mission.start")
    except RunNotActive:
        pass
    else:
        raise AssertionError("mission command published before the Run reached RUNNING")
    orchestrator.start_run(run.run_id)

    published = orchestrator.issue_command(run.run_id, "mission.start")

    assert published["action"] == "ros1.mission.start"
    assert "command:ros1:ros1.mission.start" in trace
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
