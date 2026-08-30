import os
import importlib.util
import subprocess
from pathlib import Path
import sys


REPO_ROOT = Path(__file__).resolve().parents[1]
BASELINE_COMMIT = "99c516ed6f05c1775d51d5332d6a919e948ea1ee"
BASELINE_WORKSPACE_ROOT = "/home/zjc/workspace"


def _run_script(script_name: str, *args: str, env: dict | None = None) -> subprocess.CompletedProcess:
    cmd = ["bash", str(REPO_ROOT / script_name), *args]
    merged_env = os.environ.copy()
    merged_env["WORKSPACE_DIR"] = str(REPO_ROOT)
    if env:
        merged_env.update(env)

    try:
        return subprocess.run(
            cmd,
            cwd=REPO_ROOT,
            env=merged_env,
            text=True,
            capture_output=True,
            timeout=5,
            check=False,
        )
    except subprocess.TimeoutExpired as exc:
        stdout = exc.stdout.decode() if isinstance(exc.stdout, bytes) else (exc.stdout or "")
        stderr = exc.stderr.decode() if isinstance(exc.stderr, bytes) else (exc.stderr or "")
        return subprocess.CompletedProcess(cmd, 124, stdout, stderr)


def _write_executable(path: Path, content: str) -> None:
    path.write_text(content)
    path.chmod(0o755)


def _baseline(script_name: str) -> str:
    path = (
        REPO_ROOT
        / "tests"
        / "fixtures"
        / "baseline"
        / f"{Path(script_name).stem}.{BASELINE_COMMIT}.stdout.txt"
    )
    return path.read_text(encoding="utf-8").replace(
        BASELINE_WORKSPACE_ROOT,
        str(REPO_ROOT.parent),
    )


def _environment_generator():
    path = REPO_ROOT / "src/launch_webots/launch_webots/environment_generator.py"
    specification = importlib.util.spec_from_file_location("platform_environment_generator", path)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


def _startup_policy():
    path = REPO_ROOT / "src/launch_webots/launch_webots/startup_policy.py"
    specification = importlib.util.spec_from_file_location("platform_startup_policy", path)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    sys.modules[specification.name] = module
    specification.loader.exec_module(module)
    return module


def test_platform_generated_world_aligns_timestep_without_changing_native_default(tmp_path: Path, monkeypatch):
    generator = _environment_generator()
    template = tmp_path / "template.wbt"
    native_output = tmp_path / "native.wbt"
    platform_output = tmp_path / "platform.wbt"
    template.write_text(
        "#VRML_SIM R2025a utf8\nWorldInfo {\n  basicTimeStep 16\n}\n",
        encoding="utf-8",
    )

    generator.generate_world_file(template, native_output, "e_puck", 0, [])
    monkeypatch.setenv("MASAR_PLATFORM_MODE", "1")
    assert generator.platform_basic_time_step_ms("e_puck") == 20
    assert generator.platform_basic_time_step_ms("burger") == 10
    generator.generate_world_file(
        template,
        platform_output,
        "e_puck",
        0,
        [],
        basic_time_step_ms=20,
    )

    assert "basicTimeStep 16" in native_output.read_text(encoding="utf-8")
    assert "basicTimeStep 20" in platform_output.read_text(encoding="utf-8")


def test_default_robosar_dry_run_matches_pre_change_baseline():
    baseline_env = {
        "ROS2SWARM_VNC_DISPLAY": ":99",
        "ROS2SWARM_XAUTHORITY": "/tmp/masar-platform-baseline.xauth",
    }

    script_name = "start_robosar_webots.sh"
    result = _run_script(script_name, "--dry-run", "--no-vglrun", env=baseline_env)
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stderr == ""
    assert result.stdout == _baseline(script_name)


def test_default_simulation_targets_reproducible_three_epuck_2m_scene():
    result = _run_script(
        "start_simulation.sh",
        "--dry-run",
        "--no-vglrun",
        env={
            "ROS2SWARM_VNC_DISPLAY": ":99",
            "ROS2SWARM_XAUTHORITY": "/tmp/masar-platform-baseline.xauth",
        },
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    for argument in (
        "webots_world:=command_world_2m.wbt",
        "pattern:=navigation_pattern",
        "number_robots:=3",
        "robot:=e_puck",
        "sensor_type:=lidar",
        "x_min:=-0.75",
        "x_max:=0.75",
        "y_min:=-0.75",
        "y_max:=0.75",
        "random_seed:=42",
    ):
        assert argument in output


def test_robosar_platform_stream_environment_is_forwarded_explicitly():
    result = _run_script(
        "start_robosar_webots.sh",
        "--dry-run",
        "--no-vglrun",
        env={"WEBOTS_STREAM": "true", "WEBOTS_PORT": "18141"},
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "webots_stream:=true" in output
    assert "webots_port:=18141" in output


def test_robosar_platform_mode_sources_installed_ros1_bridge_overlay():
    result = _run_script(
        "start_robosar_webots.sh",
        "--dry-run",
        "--vglrun",
        env={
            "MASAR_PLATFORM_MODE": "1",
            "ROS2SWARM_VNC_DISPLAY": ":103",
            "ROBOSAR_VGLRUN_DEVICE": ":0",
            "WEBOTS_STREAM": "false",
            "WEBOTS_PORT": "1234",
        },
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "DISPLAY=:103" in output
    assert "vglrun -d :0 ros2 launch" in output
    assert "bridge-source: source /root/ros-humble-ros1-bridge/install/setup.bash" in output


def test_both_webots_launch_paths_forward_default_off_stream_arguments():
    start_source = (REPO_ROOT / "src/launch_webots/launch/start_webots.launch.py").read_text()
    swarm_source = (REPO_ROOT / "src/launch_webots/launch/create_enviroment.launch.py").read_text()
    robosar_source = (REPO_ROOT / "src/launch_webots/launch/robosar_webots_sim.launch.py").read_text()
    robot_nodes_source = (REPO_ROOT / "src/launch_webots/launch_webots/robot_nodes.py").read_text()

    assert "'webots_stream',\n        default_value='false'" in start_source
    assert "'webots_port',\n        default_value='1234'" in start_source
    assert "stream=webots_stream" in start_source
    assert "port=webots_port" in start_source

    for source in (swarm_source, robosar_source):
        assert "'webots_stream':" in source
        assert "'webots_port':" in source

    assert swarm_source.count("webots_port=webots_port") == 2
    assert "_build_robot_actions(number_robots, robot_type, webots_port)" in robosar_source
    assert robot_nodes_source.count("port=webots_port") == 2


def test_robot_controller_spawners_are_serialized_per_robot():
    source = (REPO_ROOT / "src/launch_webots/launch_webots/robot_nodes.py").read_text()

    assert source.count(
        "arguments=['diffdrive_controller', 'joint_state_broadcaster']"
    ) == 2
    assert source.count("'--service-call-timeout', '50'") == 2
    assert source.count("ros_control_spawners = [_staggered_action(controller_spawner, robot_index)]") == 2
    assert "TimerAction(period=delay, actions=[action])" in source


def test_platform_controller_start_delay_is_opt_in_and_indexed(monkeypatch):
    policy = _startup_policy()
    monkeypatch.delenv("MASAR_PLATFORM_MODE", raising=False)
    assert policy.controller_start_delay_seconds(3) == 0.0

    monkeypatch.setenv("MASAR_PLATFORM_MODE", "1")
    monkeypatch.delenv("MASAR_PLATFORM_CONTROLLER_START_DELAY_SECONDS", raising=False)
    assert policy.controller_start_delay_seconds(0) == 0.0
    assert policy.controller_start_delay_seconds(3) == 3.0

    monkeypatch.setenv("MASAR_PLATFORM_CONTROLLER_START_DELAY_SECONDS", "0.25")
    assert policy.controller_start_delay_seconds(4) == 1.0


def test_robosar_stream_environment_rejects_partial_or_invalid_values():
    partial = _run_script(
        "start_robosar_webots.sh",
        "--dry-run",
        "--no-vglrun",
        env={"WEBOTS_STREAM": "true"},
    )
    invalid = _run_script(
        "start_robosar_webots.sh",
        "--dry-run",
        "--no-vglrun",
        env={"WEBOTS_STREAM": "true", "WEBOTS_PORT": "70000"},
    )

    assert partial.returncode == 2
    assert "WEBOTS_STREAM and WEBOTS_PORT must be set together" in partial.stderr
    assert invalid.returncode == 2
    assert "WEBOTS_PORT must be between 1 and 65535" in invalid.stderr


def test_run_vnc_gpu_source_exports_vnc_display_and_virtualgl_defaults(tmp_path: Path):
    xauth_file = tmp_path / "docker.xauth"
    xauth_file.write_text("")

    command = (
        f"cd {REPO_ROOT} && "
        f"export ROS2SWARM_VNC_DISPLAY=:7 && "
        f"export ROS2SWARM_XAUTHORITY={xauth_file} && "
        "source ./run_vnc_gpu.sh --quiet && "
        "printf 'DISPLAY=%s\n' \"$DISPLAY\" && "
        "printf 'XAUTHORITY=%s\n' \"$XAUTHORITY\" && "
        "printf 'ROS2SWARM_USE_VGLRUN=%s\n' \"$ROS2SWARM_USE_VGLRUN\" && "
        "printf 'ROBOSAR_USE_VGLRUN=%s\n' \"$ROBOSAR_USE_VGLRUN\" && "
        "printf 'ROS2SWARM_VGLRUN_DEVICE=%s\n' \"$ROS2SWARM_VGLRUN_DEVICE\" && "
        "printf 'ROBOSAR_VGLRUN_DEVICE=%s\n' \"$ROBOSAR_VGLRUN_DEVICE\" && "
        "printf 'QT_X11_NO_MITSHM=%s\n' \"$QT_X11_NO_MITSHM\""
    )

    result = subprocess.run(
        ["bash", "-lc", command],
        cwd=REPO_ROOT,
        text=True,
        capture_output=True,
        timeout=5,
        check=False,
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "DISPLAY=:7" in output
    assert f"XAUTHORITY={xauth_file}" in output
    assert "ROS2SWARM_USE_VGLRUN=true" in output
    assert "ROBOSAR_USE_VGLRUN=true" in output
    assert "ROS2SWARM_VGLRUN_DEVICE=/dev/dri/card2" in output
    assert "ROBOSAR_VGLRUN_DEVICE=/dev/dri/card2" in output
    assert "QT_X11_NO_MITSHM=1" in output


def test_start_simulation_dry_run_auto_sources_vnc_gpu_helper(tmp_path: Path):
    xauth_file = tmp_path / "docker.xauth"
    xauth_file.write_text("")

    result = _run_script(
        "start_simulation.sh",
        "--dry-run",
        env={
            "ROS2SWARM_VNC_DISPLAY": ":8",
            "ROS2SWARM_XAUTHORITY": str(xauth_file),
        },
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert f"[start_simulation][dry-run] vnc-gpu: DISPLAY=:8 XAUTHORITY={xauth_file}" in output
    assert "vglrun -d /dev/dri/card2 ros2 launch launch_webots create_enviroment.launch.py" in output


def test_start_robosar_webots_dry_run_auto_sources_vnc_gpu_helper(tmp_path: Path):
    xauth_file = tmp_path / "docker.xauth"
    xauth_file.write_text("")

    result = _run_script(
        "start_robosar_webots.sh",
        "--dry-run",
        env={
            "ROS2SWARM_VNC_DISPLAY": ":9",
            "ROS2SWARM_XAUTHORITY": str(xauth_file),
        },
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert f"[start_robosar_webots][dry-run] vnc-gpu: DISPLAY=:9 XAUTHORITY={xauth_file}" in output
    assert "vglrun -d /dev/dri/card2 ros2 launch launch_webots robosar_webots_sim.launch.py" in output


def test_start_simulation_dry_run_wraps_launch_with_vglrun():
    result = _run_script(
        "start_simulation.sh",
        "--dry-run",
        "--vglrun",
        env={"ROS2SWARM_VGLRUN_DEVICE": "/dev/dri/card9"},
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "vglrun -d /dev/dri/card9 ros2 launch launch_webots create_enviroment.launch.py" in output


def test_start_simulation_dry_run_can_disable_vglrun():
    result = _run_script(
        "start_simulation.sh",
        "--dry-run",
        "--no-vglrun",
        env={"ROS2SWARM_USE_VGLRUN": "true"},
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "ros2 launch launch_webots create_enviroment.launch.py" in output
    assert "vglrun -d" not in output


def test_start_simulation_dry_run_includes_default_transport_object_shape():
    result = _run_script(
        "start_simulation.sh",
        "--dry-run",
        "--no-vglrun",
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "transport_object_shape:=cube" in output


def test_start_simulation_help_describes_transport_object_shape_variable():
    result = _run_script("start_simulation.sh", "--help")

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "TRANSPORT_OBJECT_SHAPE" in output
    assert "cube、cylinder、triangular_prism" in output
    assert "--transport-object-shape" not in output


def test_start_simulation_help_lists_top_level_launch_config_variables():
    result = _run_script("start_simulation.sh", "--help")

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    for variable_name in (
        "WEBOTS_WORLD",
        "PATTERN",
        "NUMBER_ROBOTS",
        "TOTAL_ROBOTS",
        "LOG_LEVEL",
        "ROBOT",
        "SENSOR_TYPE",
        "X_MIN",
        "X_MAX",
        "Y_MIN",
        "Y_MAX",
        "MIN_DISTANCE",
        "Z_HEIGHT",
        "DRIVING_SWARM",
        "LOGGING",
        "NUM_OBSTACLES",
    ):
        assert variable_name in output


def test_start_robosar_webots_dry_run_wraps_launch_with_vglrun():
    result = _run_script(
        "start_robosar_webots.sh",
        "--dry-run",
        "--vglrun",
        env={"ROBOSAR_VGLRUN_DEVICE": "/dev/dri/card7"},
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "vglrun -d /dev/dri/card7 ros2 launch launch_webots robosar_webots_sim.launch.py" in output


def test_start_robosar_webots_dry_run_can_disable_vglrun():
    result = _run_script(
        "start_robosar_webots.sh",
        "--dry-run",
        "--no-vglrun",
        env={"ROBOSAR_USE_VGLRUN": "true"},
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
    assert "ros2 launch launch_webots robosar_webots_sim.launch.py" in output
    assert "vglrun -d" not in output


def test_start_simulation_real_run_tolerates_unset_colcon_trace(tmp_path: Path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    fake_workspace = tmp_path / "workspace"
    fake_install = fake_workspace / "install"
    fake_install.mkdir(parents=True)
    (fake_install / "setup.bash").write_text(
        "if [[ -n \"$COLCON_TRACE\" ]]; then :; fi\n",
        encoding="utf-8",
    )

    _write_executable(
        fake_bin / "colcon",
        "#!/usr/bin/env bash\nexit 0\n",
    )
    _write_executable(
        fake_bin / "ros2",
        "#!/usr/bin/env bash\nexit 0\n",
    )

    result = _run_script(
        "start_simulation.sh",
        "--no-vglrun",
        env={
            "PATH": f"{fake_bin}:{os.environ['PATH']}",
            "WORKSPACE_DIR": str(fake_workspace),
        },
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
