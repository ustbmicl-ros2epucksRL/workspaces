from pathlib import Path

import pytest

from masar_platform.catalog import ScenarioCatalog
from masar_platform.commands import COMMAND_REFS, CommandRenderError, render_command


ROOT = Path(__file__).resolve().parents[2]


def test_static_registry_contains_exactly_seven_refs() -> None:
    assert set(COMMAND_REFS) == {
        "ros2.swarm",
        "ros1.response_deform",
        "ros1.response_controller",
        "ros1.exploration_gazebo",
        "ros2.exploration_webots",
        "ros1.exploration_webots",
        "ros1.tepper_response",
    }


def test_rendered_argv_is_shell_free_and_does_not_reference_retired_agent() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    forbidden = ("18088", "masar-unified-platform", "agent.py", "bash -c", "$(", "|", ">", ";", "&", "\n")
    for manifest in catalog:
        for step in manifest.steps:
            argv = render_command(step.command_ref, manifest.parameters_defaults)
            assert isinstance(argv, list)
            assert all(isinstance(item, str) and item for item in argv)
            assert not any(token in item for item in argv for token in forbidden)


def test_response_controller_mode_is_parameterized_after_fixed_options() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get("control.gazebo.response-controller")
    argv = render_command("ros1.response_controller", {"mode": "transport"}, manifest=manifest)
    assert argv[:3] == ["./run_response_controller_demo.sh", "--no-rviz", "--no-vglrun"]
    assert argv[3] == "transport"


def test_ros2_launch_renders_allowlisted_parameters() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get("swarm.webots.ros2swarm")
    argv = render_command(
        "ros2.swarm",
        {"pattern": "aggregation_pattern", "robot": "e_puck", "sensor_type": "ir", "number_robots": 3},
        manifest=manifest,
    )
    assert argv[:4] == ["ros2", "launch", "launch_webots", "create_enviroment.launch.py"]
    assert "pattern:=aggregation_pattern" in argv
    assert "robot:=e_puck" in argv
    assert "sensor_type:=ir" in argv
    assert "number_robots:=3" in argv


def test_renderer_rejects_unknown_or_invalid_values() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get("control.gazebo.response-controller")
    with pytest.raises(CommandRenderError):
        render_command("ros1.response_controller", {"mode": "nope"}, manifest=manifest)
    with pytest.raises(CommandRenderError):
        render_command("ros1.response_controller", {"mode": "transport", "raw": "x"}, manifest=manifest)
