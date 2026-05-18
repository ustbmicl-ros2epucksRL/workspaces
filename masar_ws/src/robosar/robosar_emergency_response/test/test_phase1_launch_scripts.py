import os
import subprocess
from pathlib import Path
import xml.etree.ElementTree as ET


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORKSPACE_DIR = PACKAGE_DIR.parents[2]
PACKAGE_SCRIPT = PACKAGE_DIR / "scripts" / "run_tepper_response_demo.sh"
ROOT_SCRIPT = WORKSPACE_DIR / "run_tepper_response_demo.sh"
ROBOSAR_G_SCRIPT = WORKSPACE_DIR / "launch_robosar_g.sh"
ROBOSAR_W_SCRIPT = WORKSPACE_DIR / "launch_robosar_w.sh"
ROOT_RESPONSE_SCRIPT = WORKSPACE_DIR / "run_response_deform_demo.sh"
ROOT_START_SCRIPT = WORKSPACE_DIR / "start_response_tasks.sh"
LAUNCH_FILE = PACKAGE_DIR / "launch" / "tepper_response_demo.launch"
CONFIG_FILE = PACKAGE_DIR / "config" / "tepper_response_group.yaml"
TEPPER_WORLD = (
    WORKSPACE_DIR / "src" / "robosar" / "robosar_gazebo_sim" / "world" / "tepper.world"
)


def test_phase12_overlay_launch_wires_only_response_group_and_task_controller():
    launch_text = LAUNCH_FILE.read_text()

    assert "response_group_gazebo.launch" in launch_text
    assert 'start_gazebo" value="false"' in launch_text
    assert "response_group_deform.launch" in launch_text
    assert "emergency_task_manager_node.py" in launch_text
    assert "response_task_controller_node.py" in launch_text
    assert "response_global_info_node.py" in launch_text
    assert "response_group_coordinator_node.py" in launch_text
    assert "response_cmd_mux_node.py" in launch_text
    assert "robosar_sim_tepper.launch" not in launch_text
    assert "robosar_agent_bringup_node.launch" not in launch_text
    assert "robosar_mapping_sim_g.launch" not in launch_text
    assert "task_generator.launch" not in launch_text
    assert "mission_executive.launch" not in launch_text
    assert "mission_commander.py" not in launch_text
    assert "robosar_gui.py" not in launch_text


def test_phase12_launch_exposes_requested_tepper_response_spawn_positions():
    config = CONFIG_FILE.read_text()
    launch_text = LAUNCH_FILE.read_text()

    assert "spawn_positions:" not in config
    assert 'name="tb_0_x" default="17.00"' in launch_text
    assert 'name="tb_0_y" default="8.70"' in launch_text
    assert 'name="tb_1_x" default="16.55"' in launch_text
    assert 'name="tb_1_y" default="9.04"' in launch_text
    assert 'name="tb_2_x" default="16.55"' in launch_text
    assert 'name="tb_2_y" default="8.36"' in launch_text
    assert 'name="tb_3_x" default="16.10"' in launch_text
    assert 'name="tb_3_y" default="8.70"' in launch_text
    assert "enable_goal_region_arrival: true" in config
    assert "goal_region_robot_radius: 1.35" in config
    assert "checkpoint_controller_enabled: true" in config


def test_response_group_yaml_does_not_duplicate_deform_launch_arguments():
    config = CONFIG_FILE.read_text()
    response_demo_config = (
        WORKSPACE_DIR
        / "src"
        / "robosar"
        / "robosar_emergency_response"
        / "config"
        / "response_group.yaml"
    ).read_text()

    for text in (config, response_demo_config):
        assert "leader_id:" not in text
        assert "tb_num:" not in text
        assert "other_num_in_opti:" not in text
        assert "formation_interval:" not in text
        assert "formation_type:" not in text


def test_phase12_run_script_starts_base_stack_then_response_overlay_and_supports_vglrun():
    env = os.environ.copy()
    env["ROBOSAR_SKIP_SETUP"] = "1"
    env["ROBOSAR_USE_VGLRUN"] = "1"
    env["ROBOSAR_VGLRUN_DEVICE"] = "/dev/dri/test-card"
    result = subprocess.run(
        ["bash", str(PACKAGE_SCRIPT), "--dry-run", "--headless"],
        cwd=str(WORKSPACE_DIR),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert "roscore" in result.stdout
    assert "robosar_agent_bringup robosar_agent_bringup_node.launch" in result.stdout
    assert "vglrun -d /dev/dri/test-card" in result.stdout
    assert "robosar_sim_bringup robosar_sim_tepper.launch" in result.stdout
    assert "slam_toolbox robosar_mapping_sim_g.launch" in result.stdout
    assert "robosar_task_generator task_generator.launch" in result.stdout
    assert "robosar_navigation mission_executive.launch" in result.stdout
    assert "robosar_task_allocator mission_commander.py" in result.stdout
    assert "src/robosar/robosar_gui/src/robosar_gui.py" in result.stdout
    assert "roslaunch robosar_emergency_response tepper_response_demo.launch" in result.stdout
    assert "gz_gui:=false" in result.stdout
    assert "rviz:=false" in result.stdout
    assert "start_response_tasks" not in result.stdout
    assert "ROS_MASTER_URI=http://127.0.0.1:11511" not in result.stdout


def test_workspace_phase12_script_delegates_to_package_script():
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
    assert "tepper_response_demo.launch" in result.stdout
    assert "gz_gui:=false" in result.stdout
    assert "rviz:=false" in result.stdout


def test_original_gazebo_launcher_supports_help_dry_run_and_vglrun():
    env = os.environ.copy()
    env["ROBOSAR_SKIP_SETUP"] = "1"
    env["ROBOSAR_VGLRUN_DEVICE"] = "/dev/dri/test-card"

    help_result = subprocess.run(
        ["bash", str(ROBOSAR_G_SCRIPT), "--help"],
        cwd=str(WORKSPACE_DIR),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert help_result.returncode == 0, help_result.stderr
    assert "作用：" in help_result.stdout
    assert "--vglrun" in help_result.stdout
    assert "--dry-run" in help_result.stdout

    dry_result = subprocess.run(
        ["bash", str(ROBOSAR_G_SCRIPT), "--dry-run", "--headless", "--vglrun"],
        cwd=str(WORKSPACE_DIR),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert dry_result.returncode == 0, dry_result.stderr
    assert "roscore" in dry_result.stdout
    assert "rosparam set /use_sim_time true" in dry_result.stdout
    assert "vglrun -d /dev/dri/test-card" in dry_result.stdout
    assert "robosar_sim_bringup robosar_sim_tepper.launch" in dry_result.stdout
    assert "gz_gui:=false" in dry_result.stdout
    assert "rviz:=false" in dry_result.stdout
    assert "robosar_gui.py" in dry_result.stdout


def test_original_webots_launcher_supports_help_dry_run_and_vglrun():
    env = os.environ.copy()
    env["ROBOSAR_SKIP_SETUP"] = "1"
    env["ROBOSAR_VGLRUN_DEVICE"] = "/dev/dri/test-card"

    help_result = subprocess.run(
        ["bash", str(ROBOSAR_W_SCRIPT), "--help"],
        cwd=str(WORKSPACE_DIR),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert help_result.returncode == 0, help_result.stderr
    assert "作用：" in help_result.stdout
    assert "--vglrun" in help_result.stdout
    assert "--dry-run" in help_result.stdout

    dry_result = subprocess.run(
        ["bash", str(ROBOSAR_W_SCRIPT), "--dry-run", "--no-rviz", "--vglrun"],
        cwd=str(WORKSPACE_DIR),
        env=env,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    assert dry_result.returncode == 0, dry_result.stderr
    assert "roscore" in dry_result.stdout
    assert "vglrun -d /dev/dri/test-card" in dry_result.stdout
    assert "robosar_sim_bringup robosar_sim_w.launch" in dry_result.stdout
    assert "rviz:=false" in dry_result.stdout
    assert "robosar_gui.py" in dry_result.stdout


def test_workspace_shortcut_scripts_have_chinese_purpose_and_usage_header():
    for script in (
        ROBOSAR_G_SCRIPT,
        ROBOSAR_W_SCRIPT,
        ROOT_SCRIPT,
        ROOT_RESPONSE_SCRIPT,
        ROOT_START_SCRIPT,
    ):
        text = script.read_text()
        first_lines = "\n".join(text.splitlines()[:16])
        assert "作用：" in first_lines
        assert "用法：" in first_lines


def test_tepper_run_script_uses_simple_process_group_cleanup_without_isolated_scan():
    script = PACKAGE_SCRIPT.read_text()

    assert "cleanup_existing_tepper_response_processes" not in script
    assert "collect_ros_master_uri_pids" not in script
    assert "TEPPER_RESPONSE_ROS_PORT" not in script
    assert "--isolated" not in script
    assert "--shared-master" not in script
    assert "--cleanup-only" not in script
    assert "kill -INT -\"${pid}\"" in script
    assert "kill -0 \"$pid\"" not in script
    assert "pkill -INT -x gzserver" in script
    assert "kill -KILL" in script


def test_tepper_gazebo_world_closes_lower_openings_to_match_static_map():
    tree = ET.parse(TEPPER_WORLD)

    expected = {
        "Wall_lower_left_closure": {
            "size": "4.54158 0.15 2.5",
            "pose": "8.040406 5.840463 0 0 -0 1.498292",
            "state_pose": "2.639594 3.159587 0 0 0 -1.643300",
        },
        "Wall_lower_right_closure": {
            "size": "2.94806 0.15 2.5",
            "pose": "-8.060286 -0.175905 0 0 -0 1.470880",
            "state_pose": "18.740286 9.175955 0 0 0 -1.670712",
        },
    }

    for name, values in expected.items():
        link = tree.find("./world/model[@name='tepper_lockers']/link[@name='{}']".format(name))
        state_link = tree.find(
            "./world/state/model[@name='tepper_lockers']/link[@name='{}']".format(name)
        )

        assert link is not None
        assert state_link is not None
        assert link.findtext("./collision/geometry/box/size") == values["size"]
        assert link.findtext("pose") == values["pose"]
        assert state_link.findtext("pose") == values["state_pose"]


def test_emergency_response_public_filenames_do_not_use_phase_labels():
    checked_roots = [
        WORKSPACE_DIR,
        PACKAGE_DIR,
        WORKSPACE_DIR / "docs" / "superpowers" / "plans",
    ]

    offenders = []
    for root in checked_roots:
        if root.is_file():
            candidates = [root]
        elif root == WORKSPACE_DIR:
            candidates = [path for path in root.iterdir() if path.is_file()]
        elif root.exists():
            candidates = [path for path in root.rglob("*") if path.is_file()]
        else:
            candidates = []
        offenders.extend(
            str(path.relative_to(WORKSPACE_DIR))
            for path in candidates
            if any(label in path.name for label in ("stage0", "stage1", "stage2", "stage_"))
            and "__pycache__" not in path.parts
        )

    assert offenders == []


def test_apriltag_sim_only_deletes_gazebo_entities_that_exist():
    source = (
        WORKSPACE_DIR
        / "src"
        / "robosar"
        / "robosar_sim_bringup"
        / "scripts"
        / "AprilTagSim.py"
    ).read_text()

    assert "/gazebo/get_world_properties" in source
    assert "delete_model(model_name)" in source
    assert "existing_models" in source
