import os
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]


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
    assert "ROS2SWARM_VGLRUN_DEVICE=/dev/dri/card0" in output
    assert "ROBOSAR_VGLRUN_DEVICE=/dev/dri/card0" in output
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
    assert "vglrun -d /dev/dri/card0 ros2 launch launch_webots create_enviroment.launch.py" in output


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
    assert "vglrun -d /dev/dri/card0 ros2 launch launch_webots robosar_webots_sim.launch.py" in output


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
        env={"PATH": f"{fake_bin}:{os.environ['PATH']}"},
    )

    output = result.stdout + result.stderr
    assert result.returncode == 0, output
