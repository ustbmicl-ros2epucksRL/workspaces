import os
import subprocess
from pathlib import Path


PACKAGE_DIR = Path(__file__).resolve().parents[1]
SCRIPT = PACKAGE_DIR / "scripts" / "run_response_deform_demo.sh"
WORKSPACE_DIR = PACKAGE_DIR.parents[2]
ROOT_SCRIPT = WORKSPACE_DIR / "run_response_deform_demo.sh"
REFERENCE_WORLD = PACKAGE_DIR / "worlds" / "deform_navigation_demo.world"
PACKAGE_START_SCRIPT = PACKAGE_DIR / "scripts" / "start_response_tasks.sh"
ROOT_START_SCRIPT = WORKSPACE_DIR / "start_response_tasks.sh"


def run_dry_script(*args, **env_overrides):
    env = os.environ.copy()
    env.update(env_overrides)
    env.setdefault("ROBOSAR_SKIP_SETUP", "1")
    return subprocess.run(
        ["bash", str(SCRIPT), "--dry-run", *args],
        cwd=str(PACKAGE_DIR.parents[2]),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )


def test_response_demo_run_script_builds_default_roslaunch_command():
    result = run_dry_script(ROBOSAR_USE_VGLRUN="0")

    assert result.returncode == 0, result.stderr
    assert "roslaunch robosar_emergency_response response_deform_demo.launch" in result.stdout
    assert "gz_gui:=true" in result.stdout
    assert "rviz:=true" in result.stdout
    assert "model:=burger" in result.stdout
    assert f"world_name:={REFERENCE_WORLD}" in result.stdout
    assert "ROS_MASTER_URI=http://127.0.0.1:11411" in result.stdout
    assert "GAZEBO_MASTER_URI=http://127.0.0.1:11445" in result.stdout
    assert "vglrun" not in result.stdout
    assert "tepper" not in result.stdout
    assert "robosar_sim_bringup" not in result.stdout


def test_response_demo_run_script_supports_vglrun_and_passthrough_overrides():
    result = run_dry_script(
        "--vglrun",
        "--headless",
        "goal_sequence_file:=/tmp/goals.yaml",
        ROBOSAR_VGLRUN_DEVICE="/dev/dri/test-card",
    )

    assert result.returncode == 0, result.stderr
    assert "vglrun -d /dev/dri/test-card" in result.stdout
    assert "gz_gui:=false" in result.stdout
    assert "rviz:=false" in result.stdout
    assert "goal_sequence_file:=/tmp/goals.yaml" in result.stdout


def test_workspace_root_script_delegates_to_response_demo_launcher():
    env = os.environ.copy()
    env["ROBOSAR_SKIP_SETUP"] = "1"
    env["ROBOSAR_USE_VGLRUN"] = "0"
    result = subprocess.run(
        ["bash", str(ROOT_SCRIPT), "--dry-run", "--headless"],
        cwd=str(WORKSPACE_DIR),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "roslaunch robosar_emergency_response response_deform_demo.launch" in result.stdout
    assert "gz_gui:=false" in result.stdout
    assert "rviz:=false" in result.stdout
    assert f"world_name:={REFERENCE_WORLD}" in result.stdout


def test_workspace_root_script_keeps_response_demo_independent_from_search_group():
    env = os.environ.copy()
    env["ROBOSAR_SKIP_SETUP"] = "1"
    env["ROBOSAR_USE_VGLRUN"] = "0"
    result = subprocess.run(
        ["bash", str(ROOT_SCRIPT), "--dry-run"],
        cwd=str(WORKSPACE_DIR),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "response_deform_demo.launch" in result.stdout
    assert "launch_robosar_g.sh" not in result.stdout
    assert "robosar_sim_tepper.launch" not in result.stdout
    assert "robosar_sim_bringup" not in result.stdout
    assert "agent_" not in result.stdout
    assert f"world_name:={REFERENCE_WORLD}" in result.stdout


def test_response_demo_run_script_can_reuse_shared_master_when_requested():
    result = run_dry_script(
        "--shared-master",
        ROS_MASTER_URI="http://example.invalid:12345",
        GAZEBO_MASTER_URI="http://example.invalid:12346",
        ROBOSAR_USE_VGLRUN="0",
    )

    assert result.returncode == 0, result.stderr
    assert "ROS_MASTER_URI=http://example.invalid:12345" in result.stdout
    assert "GAZEBO_MASTER_URI=http://example.invalid:12346" in result.stdout


def test_response_demo_run_script_reports_targeted_cleanup_for_ctrl_c():
    result = run_dry_script("--cleanup-only", ROBOSAR_USE_VGLRUN="0")

    assert result.returncode == 0, result.stderr
    assert "cleanup-only" in result.stdout
    assert "ROS_MASTER_URI=http://127.0.0.1:11411" in result.stdout
    assert "GAZEBO_MASTER_URI=http://127.0.0.1:11445" in result.stdout
    assert "rosmaster --core -p 11411" in result.stdout
    assert "gzserver" in result.stdout
    assert "gzclient" in result.stdout
    assert "tepper" not in result.stdout


def test_response_demo_run_script_has_ctrl_c_trap_and_managed_process_group():
    script_text = SCRIPT.read_text()

    assert "trap 'handle_interrupt' INT TERM" in script_text
    assert 'setsid "${COMMAND[@]}" &' in script_text
    assert "kill_process_group" in script_text
    assert 'kill "-$signal" "-$pid"' in script_text


def test_response_demo_cleanup_tolerates_processes_that_exit_while_scanning_proc():
    script_text = SCRIPT.read_text()

    assert "env_lines=" in script_text
    assert 'cat "$env_file" 2>/dev/null | tr' in script_text
    assert "kill_pid_list always succeeds for cleanup callers" in script_text
    assert "pids_with_env_value always succeeds for cleanup callers" in script_text


def test_response_demo_launch_defaults_to_deform_navigation_demo_world():
    launch_file = PACKAGE_DIR / "launch" / "response_deform_demo.launch"
    launch_xml = launch_file.read_text()

    assert "deform_navigation_demo.world" in launch_xml
    assert "deform_navigation_reference.world" not in launch_xml
    assert "response_corridor.world" not in launch_xml
    assert '<arg name="frm_type" default="3"/>' in launch_xml
    assert '<arg name="max_frm_type" default="3"/>' in launch_xml


def test_response_demo_task_start_script_publishes_start_signal():
    env = os.environ.copy()
    env["ROBOSAR_SKIP_SETUP"] = "1"
    result = subprocess.run(
        ["bash", str(PACKAGE_START_SCRIPT), "--dry-run"],
        cwd=str(WORKSPACE_DIR),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "/response_group/start" in result.stdout
    assert "std_msgs/Bool" in result.stdout
    assert "data: true" in result.stdout


def test_workspace_task_start_script_delegates_to_package_script():
    env = os.environ.copy()
    env["ROBOSAR_SKIP_SETUP"] = "1"
    result = subprocess.run(
        ["bash", str(ROOT_START_SCRIPT), "--dry-run"],
        cwd=str(WORKSPACE_DIR),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "/response_group/start" in result.stdout
