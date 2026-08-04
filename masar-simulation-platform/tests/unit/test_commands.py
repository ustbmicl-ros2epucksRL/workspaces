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
        "--vglrun",
        "--shared-master",
    ]


def test_standalone_control_demos_join_the_platform_ros_master() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")

    deform = catalog.get("control.gazebo.response-deform")
    assert render_command("ros1.response_deform", {}, manifest=deform) == [
        "./run_response_deform_demo.sh",
        "--no-rviz",
        "--vglrun",
        "--shared-master",
    ]

    controller = catalog.get("control.gazebo.response-controller")
    argv = render_command("ros1.response_controller", {}, manifest=controller)
    assert argv[-1] == "--shared-master"


@pytest.mark.parametrize(
    ("scenario_id", "command_ref"),
    [
        ("control.gazebo.response-deform", "ros1.response_deform"),
        ("control.gazebo.response-controller", "ros1.response_controller"),
        ("exploration.gazebo.tepper", "ros1.exploration_gazebo"),
        ("rescue.gazebo.tepper-response", "ros1.tepper_response"),
    ],
)
def test_platform_gazebo_runs_use_virtualgl_gpu_rendering(
    scenario_id: str,
    command_ref: str,
) -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get(scenario_id)
    invocation = render_invocation(command_ref, {}, manifest=manifest)

    assert "--vglrun" in invocation.argv
    assert "--no-vglrun" not in invocation.argv
    assert invocation.environment["DISPLAY"] == ":101"
    assert invocation.environment["ROBOSAR_VGLRUN_DEVICE"] == ":0"


def test_ros2_launch_renders_allowlisted_parameters() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get("swarm.webots.ros2swarm")
    argv = render_command(
        "ros2.swarm",
        {"pattern": "aggregation_pattern", "robot": "e_puck", "sensor_type": "ir", "number_robots": 3},
        manifest=manifest,
    )
    assert argv[:7] == ["vglrun", "-d", ":0", "ros2", "launch", "launch_webots", "create_enviroment.launch.py"]
    assert "pattern:=aggregation_pattern" in argv
    assert "robot:=e_puck" in argv
    assert "sensor_type:=ir" in argv
    assert "number_robots:=3" in argv
    assert "webots_stream:=false" in argv

    discussed = render_command(
        "ros2.swarm",
        {"pattern": "discussed_dispersion_pattern", "robot": "e_puck", "number_robots": 3},
        manifest=manifest,
    )
    assert "pattern:=discussed_dispersion_pattern" in discussed


def test_webots_exploration_uses_platform_display_and_bridge_overlay() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get("exploration.webots.robosar")
    assert render_command("ros2.exploration_webots", {}, manifest=manifest) == [
        "./start_robosar_webots.sh",
        "--vglrun",
    ]
    invocation = render_invocation("ros2.exploration_webots", {}, manifest=manifest)
    assert invocation.argv == ("./start_robosar_webots.sh", "--vglrun")
    assert dict(invocation.environment) == {
        "DISPLAY": ":103",
        "MASAR_PLATFORM_MODE": "1",
        "NO_PROXY": "127.0.0.1,localhost,::1",
        "no_proxy": "127.0.0.1,localhost,::1",
        "ROBOSAR_VGLRUN_DEVICE": ":0",
        "ROS2SWARM_VNC_DISPLAY": ":103",
        "WEBOTS_STREAM": "false",
        "WEBOTS_PORT": "1234",
        "ROS_MASTER_URI": "http://127.0.0.1:11311",
        "ROS_DOMAIN_ID": "42",
        "ROS1_MASTER_IP": "127.0.0.1",
    }
    assert dict(command_environment("ros2.exploration_webots")) == {
        "MASAR_PLATFORM_MODE": "1",
        "NO_PROXY": "127.0.0.1,localhost,::1",
        "no_proxy": "127.0.0.1,localhost,::1",
        "ROBOSAR_VGLRUN_DEVICE": ":0",
        "ROS2SWARM_VNC_DISPLAY": ":103",
        "WEBOTS_STREAM": "false",
        "WEBOTS_PORT": "1234",
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
        "vglrun",
        "-d",
        ":0",
        "ros2",
        "launch",
        "launch_webots",
        "create_enviroment.launch.py",
        "webots_stream:=false",
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


def test_manifest_runtime_is_injected_without_overriding_native_script_defaults() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    swarm = render_invocation("ros2.swarm", {}, manifest=catalog.get("swarm.webots.ros2swarm"))
    assert swarm.environment["ROS_DOMAIN_ID"] == "42"
    assert swarm.environment["ROS_MASTER_URI"] == "http://127.0.0.1:11311"
    assert swarm.environment["MASAR_PLATFORM_MODE"] == "1"
    assert swarm.environment["DISPLAY"] == ":103"

    rescue = render_invocation(
        "ros1.tepper_response",
        {},
        manifest=catalog.get("rescue.gazebo.tepper-response"),
    )
    assert rescue.argv[-2:] == ("--no-gui", "--shared-master")
    assert rescue.environment["ROS_DOMAIN_ID"] == "42"


def test_renderer_rejects_unknown_or_invalid_values() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get("control.gazebo.response-controller")
    with pytest.raises(CommandRenderError):
        render_command("ros1.response_controller", {"mode": "nope"}, manifest=manifest)
    with pytest.raises(CommandRenderError):
        render_command("ros1.response_controller", {"mode": "transport", "raw": "x"}, manifest=manifest)
