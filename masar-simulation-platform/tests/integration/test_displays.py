from __future__ import annotations

import importlib.util
import os
from pathlib import Path
import signal
import sys
import time

import pytest
import yaml

from masar_platform.catalog import ScenarioCatalog
from masar_platform.agents import AgentUnavailable
from masar_platform.displays import check_display_prerequisites, display_plan
from masar_platform.state_machine import RunState
from agents.runtime_agent.processes import DEFAULT_COMMAND_POLICIES
from tests.platform_fakes import build_test_orchestrator


ROOT = Path(__file__).resolve().parents[2]
DISPLAY_MANAGER_PATH = ROOT / "deploy" / "display" / "display_manager.py"


def _load_display_manager():
    specification = importlib.util.spec_from_file_location("display_manager", DISPLAY_MANAGER_PATH)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


def _fake_commands(module, _spec):
    sleeping = (
        sys.executable,
        "-c",
        "import signal,time; signal.signal(signal.SIGINT, signal.SIG_IGN); "
        "signal.signal(signal.SIGTERM, signal.SIG_IGN); time.sleep(60)",
    )
    return tuple(
        module.ComponentCommand(name=name, argv=sleeping)
        for name in ("xvfb", "desktop", "x11vnc", "websockify")
    )


def test_scenario_display_plans_separate_group_behavior_and_rescue() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")

    swarm = display_plan(catalog.get("swarm.webots.ros2swarm"))
    control = display_plan(catalog.get("control.gazebo.response-deform"))
    exploration = display_plan(catalog.get("exploration.gazebo.tepper"))
    webots_exploration = display_plan(catalog.get("exploration.webots.robosar"))
    rescue = display_plan(catalog.get("rescue.gazebo.tepper-response"))

    assert swarm.simulation.as_dict() == {
        "transport": "novnc",
        "path": "/display/webots/",
        "display": ":103",
        "required": True,
    }
    assert swarm.rviz is None
    assert control.simulation.display == ":101"
    assert control.rviz is None
    assert exploration.simulation.path == "/display/gazebo/"
    assert exploration.rviz is not None
    assert exploration.rviz.display == ":102"
    assert exploration.rviz.argv == (
        "rviz",
        "-d",
        "/root/workspace/masar_ws/src/robosar/robosar_sim_bringup/rviz/multi_robot.rviz",
    )
    assert webots_exploration.simulation.path == "/display/webots/"
    assert webots_exploration.rviz == exploration.rviz
    assert rescue.rviz == exploration.rviz


def test_resolved_manifest_records_gui_environment_and_exact_rviz_command(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)

    gazebo = orchestrator.create_run("exploration.gazebo.tepper", {})
    orchestrator.stop_run(gazebo.run_id)
    webots = orchestrator.create_run("exploration.webots.robosar", {})

    gazebo_resolved = gazebo.resolved_manifest
    assert gazebo_resolved["steps"][0]["environment"]["DISPLAY"] == ":101"
    assert gazebo_resolved["display_plan"]["rviz"] == {
        "agent": "ros1",
        "process_id": "display_rviz",
        "command_ref": "display.ros1.rviz",
        "argv": [
            "rviz",
            "-d",
            "/root/workspace/masar_ws/src/robosar/robosar_sim_bringup/rviz/multi_robot.rviz",
        ],
        "environment": {
            "DISPLAY": ":102",
            "MASAR_PLATFORM_MODE": "1",
            "NO_PROXY": "127.0.0.1,localhost,::1",
            "ROS_DOMAIN_ID": "42",
            "ROS_MASTER_URI": "http://127.0.0.1:11311",
            "no_proxy": "127.0.0.1,localhost,::1",
        },
        "required": False,
    }
    assert webots.resolved_manifest["steps"][0]["environment"]["DISPLAY"] == ":103"


def test_runtime_agent_allows_only_the_fixed_rviz_display_command() -> None:
    policy = DEFAULT_COMMAND_POLICIES["display.ros1.rviz"]

    assert policy.agent == "ros1"
    assert policy.executable == "rviz"


def test_optional_rviz_start_failure_does_not_stop_the_ros_run(tmp_path: Path) -> None:
    orchestrator, agents, _ = build_test_orchestrator(tmp_path)
    original_start = agents["ros1"].start

    def fail_rviz(run_id, payload):
        if payload["process_id"] == "display_rviz":
            raise AgentUnavailable("test panel failure")
        return original_start(run_id, payload)

    agents["ros1"].start = fail_rviz
    run = orchestrator.create_run("exploration.gazebo.tepper", {})

    running = orchestrator.start_run(run.run_id)

    assert running.state is RunState.RUNNING
    assert agents["ros1"].processes["exploration_gazebo"]["status"] == "RUNNING"
    assert any(event.kind == "DISPLAY_ERROR" for event in orchestrator.events.list(run.run_id))
    orchestrator.stop_run(run.run_id)


def test_preflight_checks_only_persistent_displays_required_by_the_scenario() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")

    disabled = check_display_prerequisites(
        catalog.get("swarm.webots.ros2swarm"),
        False,
        probe=lambda _port: True,
    )
    assert disabled[0] is False

    observed: list[int] = []

    def probe(port: int) -> bool:
        observed.append(port)
        return True

    swarm = check_display_prerequisites(catalog.get("swarm.webots.ros2swarm"), True, probe=probe)
    assert swarm[0] is True
    assert observed == [18133]
    observed.clear()

    check_display_prerequisites(catalog.get("control.gazebo.response-deform"), True, probe=probe)
    assert observed == [18131]
    observed.clear()
    check_display_prerequisites(catalog.get("exploration.webots.robosar"), True, probe=probe)
    assert observed == [18133, 18132]
    observed.clear()
    check_display_prerequisites(catalog.get("rescue.gazebo.tepper-response"), True, probe=probe)
    assert observed == [18131, 18132]


def test_display_lifecycle_owns_separate_process_groups_and_stops_bounded(tmp_path: Path) -> None:
    module = _load_display_manager()
    manager = module.DisplayLifecycleManager(
        state_dir=tmp_path / "state",
        x11_dir=tmp_path / "x11",
        command_factory=lambda spec: _fake_commands(module, spec),
        readiness_check=lambda _spec: True,
    )
    persisted_component_counts: list[int] = []
    original_write_state = manager._write_state

    def record_write(state):
        persisted_component_counts.append(
            sum(len(display.components) for display in state.displays.values())
        )
        original_write_state(state)

    manager._write_state = record_write

    try:
        state = manager.start("run-display-test", ("gazebo", "rviz"))
        records = [component for display in state.displays.values() for component in display.components]

        assert state.displays["gazebo"].display == ":101"
        assert state.displays["rviz"].display == ":102"
        assert len(records) == 8
        assert len({record.pid for record in records}) == 8
        assert all(record.pid == record.pgid for record in records)
        assert set(range(9)).issubset(persisted_component_counts)
        time.sleep(0.1)

        started = time.monotonic()
        stopped = manager.stop(
            "run-display-test",
            module.ShutdownGrace(sigint=0.01, sigterm=0.01, sigkill=0.2),
        )
        elapsed = time.monotonic() - started

        assert elapsed < 2
        assert all(display.status == "STOPPED" for display in stopped.displays.values())
        assert all(
            "SIGKILL" in record.signals_sent
            for display in stopped.displays.values()
            for record in display.components
        )
    finally:
        manager.stop(
            "run-display-test",
            module.ShutdownGrace(sigint=0, sigterm=0, sigkill=0.1),
            missing_ok=True,
        )


def test_unrelated_x_display_is_never_reused_or_removed(tmp_path: Path) -> None:
    module = _load_display_manager()
    x11_dir = tmp_path / "x11"
    x11_dir.mkdir()
    unrelated_socket = x11_dir / "X101"
    unrelated_socket.write_text("unrelated", encoding="utf-8")
    manager = module.DisplayLifecycleManager(
        state_dir=tmp_path / "state",
        x11_dir=x11_dir,
        command_factory=lambda spec: _fake_commands(module, spec),
        readiness_check=lambda _spec: True,
    )

    with pytest.raises(module.DisplayConflict):
        manager.start("run-conflict", ("gazebo",))

    assert unrelated_socket.read_text(encoding="utf-8") == "unrelated"
    assert not (tmp_path / "state" / "state.json").exists()


def test_stop_refuses_a_reused_component_pid(tmp_path: Path) -> None:
    module = _load_display_manager()
    manager = module.DisplayLifecycleManager(
        state_dir=tmp_path / "state",
        x11_dir=tmp_path / "x11",
        command_factory=lambda spec: _fake_commands(module, spec),
        readiness_check=lambda _spec: True,
    )

    try:
        state = manager.start("run-reused-pid", ("gazebo",))
        component = state.displays["gazebo"].components[0]
        reused = module.replace(component, start_ticks=component.start_ticks + 1)

        with pytest.raises(module.DisplayOwnershipError):
            manager._stop_component(reused, module.ShutdownGrace(0, 0, 0))
    finally:
        manager.stop(
            "run-reused-pid",
            module.ShutdownGrace(sigint=0, sigterm=0, sigkill=0.1),
            missing_ok=True,
        )


def test_one_panel_failure_is_recorded_without_stopping_the_other(tmp_path: Path) -> None:
    module = _load_display_manager()
    manager = module.DisplayLifecycleManager(
        state_dir=tmp_path / "state",
        x11_dir=tmp_path / "x11",
        command_factory=lambda spec: _fake_commands(module, spec),
        readiness_check=lambda _spec: True,
    )

    try:
        state = manager.start("run-panel-failure", ("gazebo", "rviz"))
        failed = next(
            item for item in state.displays["gazebo"].components if item.name == "websockify"
        )
        os.killpg(failed.pgid, signal.SIGKILL)
        time.sleep(0.03)

        refreshed = manager.poll()

        assert refreshed.displays["gazebo"].status == "ERROR"
        assert refreshed.displays["rviz"].status == "RUNNING"
        assert all(item.status == "RUNNING" for item in refreshed.displays["rviz"].components)
    finally:
        manager.stop(
            "run-panel-failure",
            module.ShutdownGrace(sigint=0, sigterm=0, sigkill=0.1),
            missing_ok=True,
        )


def test_healthcheck_inspects_state_without_writing_it(tmp_path: Path, monkeypatch) -> None:
    module = _load_display_manager()
    owner = module.DisplayLifecycleManager(
        state_dir=tmp_path / "state",
        x11_dir=tmp_path / "x11",
        command_factory=lambda spec: _fake_commands(module, spec),
        readiness_check=lambda _spec: True,
    )

    try:
        owner.start("run-health-read-only", ("gazebo",))
        before = owner.state_path.read_bytes()
        observer = module.DisplayLifecycleManager(
            state_dir=tmp_path / "state",
            x11_dir=tmp_path / "x11",
        )
        monkeypatch.setattr(module, "_tcp_ready", lambda _port: True)
        monkeypatch.setattr(
            observer,
            "_write_state",
            lambda _state: pytest.fail("healthcheck must not write lifecycle state"),
        )

        assert module._health(observer) == 0
        assert owner.state_path.read_bytes() == before
    finally:
        owner.stop(
            "run-health-read-only",
            module.ShutdownGrace(sigint=0, sigterm=0, sigkill=0.1),
            missing_ok=True,
        )


def test_restarted_manager_adopts_a_valid_owned_stack(tmp_path: Path) -> None:
    module = _load_display_manager()
    owner = module.DisplayLifecycleManager(
        state_dir=tmp_path / "state",
        x11_dir=tmp_path / "x11",
        command_factory=lambda spec: _fake_commands(module, spec),
        readiness_check=lambda _spec: True,
    )

    state = owner.start("run-recover", ("gazebo",))
    expected_pids = tuple(component.pid for component in state.displays["gazebo"].components)
    restarted = module.DisplayLifecycleManager(
        state_dir=tmp_path / "state",
        x11_dir=tmp_path / "x11",
        command_factory=lambda _spec: pytest.fail("a valid display stack must not be respawned"),
    )

    try:
        recovered = restarted.start_or_recover("run-recover", ("gazebo",))

        assert tuple(component.pid for component in recovered.displays["gazebo"].components) == expected_pids
        assert recovered.status == "RUNNING"
    finally:
        cleanup_manager = restarted if restarted._state is not None else owner
        cleanup_manager.stop(
            "run-recover",
            module.ShutdownGrace(sigint=0, sigterm=0, sigkill=0.1),
            missing_ok=True,
        )


def test_novnc_compose_and_caddy_contracts_are_loopback_only() -> None:
    compose = yaml.safe_load((ROOT / "deploy" / "compose.yaml").read_text(encoding="utf-8"))
    service = compose["services"]["display"]
    dockerfile = (ROOT / "deploy" / "novnc" / "Dockerfile").read_text(encoding="utf-8")
    caddy = (ROOT / "deploy" / "caddy" / "Caddyfile").read_text(encoding="utf-8")

    assert service["network_mode"] == "host"
    assert service["environment"]["NO_PROXY"] == "127.0.0.1,localhost,::1"
    assert service["environment"]["no_proxy"] == "127.0.0.1,localhost,::1"
    assert "/tmp/.X11-unix:/tmp/.X11-unix" in service["volumes"]
    assert "ports" not in service
    assert compose["services"]["api"]["environment"]["MASAR_PLATFORM_VISUAL_STACK_READY"] == "true"
    assert "0.0.0.0" not in dockerfile
    assert "63107bd06d9e1f6136ff21aeda8cd62cbf0d433e" in dockerfile
    assert "e81894751365afc19fe64fc9d0e5c6fc52655c36" in dockerfile
    assert "reverse_proxy 127.0.0.1:18131" in caddy
    assert "reverse_proxy 127.0.0.1:18132" in caddy
    assert "reverse_proxy 127.0.0.1:18133" in caddy
    assert "COPY deploy/display/xstartup-webots /usr/local/bin/xstartup-webots" in dockerfile

    module = _load_display_manager()
    assert module.DISPLAY_SPECS["webots"] == module.DisplaySpec("webots", ":103", 19133, 18133)
    for spec in module.DISPLAY_SPECS.values():
        commands = {command.name: command for command in module.production_commands(spec)}
        assert "-rfbportv6" in commands["x11vnc"].argv
        assert "-listenv6" in commands["x11vnc"].argv
        assert "::1" in commands["x11vnc"].argv
        assert f"127.0.0.1:{spec.novnc_port}" in commands["websockify"].argv
        assert f"127.0.0.1:{spec.vnc_port}" in commands["websockify"].argv
