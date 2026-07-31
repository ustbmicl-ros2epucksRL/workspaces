from pathlib import Path

import pytest

from masar_platform.catalog import ScenarioCatalog
from masar_platform.commands import (
    COMMAND_REFS,
    CommandRef,
    CommandRenderError,
    command_environment,
    render_command,
    render_invocation,
)


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


def test_response_controller_mode_precedes_fixed_options() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get("control.gazebo.response-controller")
    argv = render_command("ros1.response_controller", {"mode": "transport"}, manifest=manifest)
    assert argv == [
        "./run_response_controller_demo.sh",
        "transport",
        "--no-rviz",
        "--no-vglrun",
        "--shared-master",
    ]


def test_standalone_control_demos_join_the_platform_ros_master() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")

    deform = catalog.get("control.gazebo.response-deform")
    assert render_command("ros1.response_deform", {}, manifest=deform) == [
        "./run_response_deform_demo.sh",
        "--no-rviz",
        "--no-vglrun",
        "--shared-master",
    ]

    controller = catalog.get("control.gazebo.response-controller")
    argv = render_command("ros1.response_controller", {}, manifest=controller)
    assert argv[-1] == "--shared-master"


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
    assert "webots_stream:=true" in argv
    assert "webots_port:=18141" in argv


def test_webots_exploration_enables_platform_stream_explicitly() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get("exploration.webots.robosar")
    assert render_command("ros2.exploration_webots", {}, manifest=manifest) == [
        "./start_robosar_webots.sh",
        "--no-vglrun",
    ]
    invocation = render_invocation("ros2.exploration_webots", {}, manifest=manifest)
    assert invocation.argv == ("./start_robosar_webots.sh", "--no-vglrun")
    assert dict(invocation.environment) == {"WEBOTS_STREAM": "true", "WEBOTS_PORT": "18141"}
    assert dict(command_environment("ros2.exploration_webots")) == {
        "WEBOTS_STREAM": "true",
        "WEBOTS_PORT": "18141",
    }
    with pytest.raises(TypeError):
        invocation.environment["WEBOTS_PORT"] = "1"  # type: ignore[index]


def test_fixed_environment_rejects_shell_and_control_characters() -> None:
    with pytest.raises(ValueError, match="environment"):
        CommandRef(
            agent="ros2",
            executable="./start_robosar_webots.sh",
            fixed_environment=(("WEBOTS_STREAM", "true\nWEBOTS_PORT=1"),),
        )


def test_swarm_default_argv_preserves_start_simulation_defaults() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get("swarm.webots.ros2swarm")
    assert render_command("ros2.swarm", {}, manifest=manifest) == [
        "ros2",
        "launch",
        "launch_webots",
        "create_enviroment.launch.py",
        "webots_stream:=true",
        "webots_port:=18141",
        "webots_world:=command_world.wbt",
        "pattern:=roundflocking_pattern",
        "number_robots:=5",
        "total_robots:=10",
        "log_level:=info",
        "webots_mode:=realtime",
        "webots_gui:=true",
        "robot:=burger",
        "sensor_type:=lidar",
        "x_min:=-1.0",
        "x_max:=1.0",
        "y_min:=-1.0",
        "y_max:=1.0",
        "min_distance:=0.2",
        "z_height:=0.0",
        "driving_swarm:=False",
        "logging:=False",
        "num_obstacles:=0",
        "transport_object_shape:=cube",
    ]


def test_renderer_rejects_unknown_or_invalid_values() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get("control.gazebo.response-controller")
    with pytest.raises(CommandRenderError):
        render_command("ros1.response_controller", {"mode": "nope"}, manifest=manifest)
    with pytest.raises(CommandRenderError):
        render_command("ros1.response_controller", {"mode": "transport", "raw": "x"}, manifest=manifest)
