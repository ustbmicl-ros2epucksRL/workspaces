import inspect
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from robosar_emergency_response import home_pose  # noqa: E402
from robosar_emergency_response.checkpoint_controller import MatrixPose  # noqa: E402
from robosar_emergency_response.task_manager import (  # noqa: E402
    EmergencyObjectSpec,
    ResponseTaskQueue,
    STATUS_ACTING,
    STATUS_DONE,
    STATUS_EN_ROUTE,
    TASK_FIRE,
    TASK_VICTIM,
    all_expected_tasks_done,
    emergency_object_key,
    resume_mode_for_status,
)


def _queue():
    return ResponseTaskQueue(
        assigned_robots=["tb_0", "tb_1", "tb_2", "tb_3"],
        transport_rear_distance=0.0,
        hold_seconds=1.0,
    )


def _spec(task_type, object_id, x, y):
    return EmergencyObjectSpec(
        object_type=task_type,
        object_id=object_id,
        scene_name="tepper",
        detector_robot="agent_0",
        pose=MatrixPose(x, y),
    )


def _removed_legacy_names():
    return {
        "_".join(("navigation", "only")),
        "_".join(("discover", "all", "snapshot", "objects")),
        "_".join(("dispatch", "policy")),
        "-".join(("nav", "only", "long" + "run")),
        "_".join(("nav", "only", "long" + "run")),
    }


def test_completed_response_task_releases_active_task_and_dispatches_next():
    queue = _queue()
    center = MatrixPose(0.0, 0.0)
    assert queue.upsert_object(_spec(TASK_FIRE, 10, 1.0, 0.0), center)
    assert queue.upsert_object(_spec(TASK_VICTIM, 11, 2.0, 0.0), center)

    first = queue.dispatch_next()
    assert first is not None
    assert first.task_type == TASK_VICTIM

    done = queue.mark_task_done(first.task_id)
    assert done is not None
    assert done.status == STATUS_DONE
    assert queue.current_task() is None

    second = queue.dispatch_next()
    assert second is not None
    assert second.task_type == TASK_FIRE


def test_completed_tasks_satisfy_expected_task_return_home_gate():
    queue = _queue()
    center = MatrixPose(0.0, 0.0)
    spec = _spec(TASK_VICTIM, 42, 3.0, 0.0)
    assert queue.upsert_object(spec, center)
    active = queue.dispatch_next()
    assert active is not None
    queue.mark_task_done(active.task_id)

    assert all_expected_tasks_done(
        queue.all_tasks(),
        {emergency_object_key(TASK_VICTIM, 42, "tepper")},
    )


def test_resume_mode_matches_preserved_task_phase():
    assert resume_mode_for_status(STATUS_EN_ROUTE, return_home_active=False) == "NAV_FORMATION"
    assert resume_mode_for_status(STATUS_ACTING, return_home_active=False) == "TASK_CONTROL"
    assert resume_mode_for_status(STATUS_EN_ROUTE, return_home_active=True) == "NAV_FORMATION"
    assert resume_mode_for_status(STATUS_DONE, return_home_active=False) == "STOP"


def test_manual_home_terminates_active_task_without_dispatching_next():
    queue = _queue()
    center = MatrixPose(0.0, 0.0)
    assert queue.upsert_object(_spec(TASK_FIRE, 1, 8.0, 0.0), center)
    assert queue.upsert_object(_spec(TASK_VICTIM, 2, 4.0, 0.0), center)
    active = queue.dispatch_next()

    cancelled = queue.cancel_active_for_return_home()

    assert cancelled.task_id == active.task_id
    assert cancelled.status == queue.STATUS_FAILED
    assert queue.current_task() is None


def test_response_queue_exposes_only_priority_dispatch():
    init_parameters = inspect.signature(ResponseTaskQueue.__init__).parameters
    dispatch_parameters = inspect.signature(ResponseTaskQueue.dispatch_next).parameters

    assert list(init_parameters) == [
        "self",
        "assigned_robots",
        "transport_rear_distance",
        "hold_seconds",
        "use_spin_placeholder",
    ]
    assert list(dispatch_parameters) == ["self"]


def test_tepper_config_fallback_home_matches_actual_response_spawn():
    config_path = os.path.join(PACKAGE_ROOT, "config", "tepper_response_group.yaml")
    with open(config_path, "r", encoding="utf-8") as fh:
        config = yaml.safe_load(fh)

    assert config["initial_response_center_x"] == 4.0
    assert config["initial_response_center_y"] == 16.4
    assert config["home_capture_min_robot_fraction"] == 1.0
    assert config["auto_return_home"] is False
    assert _removed_legacy_names().isdisjoint(config)


def test_response_command_mux_defaults_are_stopped():
    config_paths = [
        os.path.join(PACKAGE_ROOT, "config", "response_group.yaml"),
        os.path.join(PACKAGE_ROOT, "config", "tepper_response_group.yaml"),
    ]
    for config_path in config_paths:
        with open(config_path, "r", encoding="utf-8") as fh:
            config = yaml.safe_load(fh)
        assert config["default_mode"] == "STOP"


def test_tepper_launch_uses_complete_response_flow():
    launch_path = os.path.join(PACKAGE_ROOT, "launch", "tepper_response_demo.launch")
    root = ET.parse(launch_path).getroot()
    defaults = {
        element.attrib["name"]: element.attrib.get("default")
        for element in root.findall("arg")
        if "name" in element.attrib
    }

    assert defaults["initial_response_center_x"] == "4.00"
    assert defaults["initial_response_center_y"] == "16.40"
    assert defaults["home_capture_min_robot_fraction"] == "1.0"
    assert [(defaults[f"tb_{index}_x"], defaults[f"tb_{index}_y"]) for index in range(4)] == [
        ("3.70", "16.70"),
        ("4.30", "16.70"),
        ("3.70", "16.10"),
        ("4.30", "16.10"),
    ]
    assert [defaults[f"tb_{index}_yaw"] for index in range(4)] == ["-1.5708"] * 4
    assert _removed_legacy_names().isdisjoint(defaults)

    task_nodes = [
        node
        for node in root.findall("node")
        if node.attrib.get("type") == "response_task_controller_node.py"
    ]
    assert len(task_nodes) == 1
    assert "if" not in task_nodes[0].attrib
    assert "unless" not in task_nodes[0].attrib

    navigation_path = os.path.join(PACKAGE_ROOT, "config", "formation_navigation.yaml")
    with open(navigation_path, "r", encoding="utf-8") as fh:
        navigation = yaml.safe_load(fh)
    assert navigation["robot_radius"] == 0.11
    assert navigation["formation_clearance"] == 0.06
    assert navigation["transport_box_half_length"] == 0.25
    assert navigation["arrival"]["transport"] == {
        "center_tolerance": 0.45,
        "rms_slot_tolerance": 0.55,
        "max_slot_tolerance": 0.85,
        "hold_seconds": 0.8,
    }


def test_teammate_safety_allows_the_minimum_scaled_formation_spacing():
    navigation_path = os.path.join(PACKAGE_ROOT, "config", "formation_navigation.yaml")
    with open(navigation_path, "r", encoding="utf-8") as fh:
        navigation = yaml.safe_load(fh)
    safety_path = os.path.join(PACKAGE_ROOT, "config", "response_safety.yaml")
    with open(safety_path, "r", encoding="utf-8") as fh:
        safety = yaml.safe_load(fh)

    minimum_slot_spacing = 2.0 * navigation["robot_radius"] + navigation["inter_robot_safety_gap"]
    minimum_center_distance = float(safety["teammate_min_center_distance"])

    assert 0.0 < minimum_center_distance <= minimum_slot_spacing


def test_response_launch_has_exactly_one_formation_navigator():
    launch_path = os.path.join(PACKAGE_ROOT, "launch", "response_group_deform.launch")
    root = ET.parse(launch_path).getroot()
    nodes = [
        node for node in root.findall("node")
        if node.attrib.get("type") == "formation_navigation_node"
    ]
    assert len(nodes) == 1


def test_response_launch_has_no_bridge_or_python_arrival_coordinator():
    for name in ("response_deform_demo.launch", "tepper_response_demo.launch"):
        text = open(os.path.join(PACKAGE_ROOT, "launch", name), encoding="utf-8").read()
        assert "deform_dispatch_bridge_node.py" not in text
        assert "response_group_coordinator_node.py" not in text
        assert "response_plan_manager_one_run.xml" not in text


def test_initial_home_pose_recording_keeps_first_odometry_sample():
    poses = {}
    first = MatrixPose(3.7, 16.7)
    moved = MatrixPose(12.0, 6.0)

    assert hasattr(home_pose, "record_initial_pose")
    assert home_pose.record_initial_pose(poses, "tb_0", first)
    assert not home_pose.record_initial_pose(poses, "tb_0", moved)
    assert poses["tb_0"] == first


def test_tepper_full_mode_dry_run_starts_complete_stack():
    script = os.path.abspath(
        os.path.join(PACKAGE_ROOT, "..", "..", "..", "run_tepper_response_demo.sh")
    )
    environment = os.environ.copy()
    environment.pop("ROS_MASTER_URI", None)
    environment.pop("GAZEBO_MASTER_URI", None)
    output = subprocess.check_output(
        [script, "--dry-run", "--headless", "--no-vglrun"],
        cwd=os.path.dirname(script),
        env=environment,
        text=True,
    )

    for process in (
        "agent_bringup",
        "slam",
        "task_generator",
        "mission_executive",
        "traffic_controller",
        "mission_commander",
        "response_overlay",
    ):
        assert process in output
    for removed in _removed_legacy_names():
        assert removed not in output
    assert "master_mode=clean" in output
    assert "ROS_MASTER_URI=http://127.0.0.1:11311" in output
    assert "GAZEBO_MASTER_URI=http://127.0.0.1:11345" in output
    assert "master_preflight: stop existing ROS nodes" in output


def test_tepper_response_source_starts_before_gui_is_exposed():
    script = os.path.abspath(
        os.path.join(PACKAGE_ROOT, "..", "..", "..", "run_tepper_response_demo.sh")
    )
    output = subprocess.check_output(
        [script, "--dry-run", "--no-gz-gui", "--no-rviz", "--gui", "--no-vglrun"],
        cwd=os.path.dirname(script),
        text=True,
    )

    assert output.index("response_overlay:") < output.index("[tepper-response][dry-run] gui:")


def test_tepper_shared_master_is_explicit_and_preserves_caller_uris():
    script = os.path.abspath(
        os.path.join(PACKAGE_ROOT, "..", "..", "..", "run_tepper_response_demo.sh")
    )
    environment = os.environ.copy()
    environment["ROS_MASTER_URI"] = "http://127.0.0.1:11911"
    environment["GAZEBO_MASTER_URI"] = "http://127.0.0.1:11945"
    output = subprocess.check_output(
        [script, "--dry-run", "--headless", "--no-vglrun", "--shared-master"],
        cwd=os.path.dirname(script),
        env=environment,
        text=True,
    )

    assert "master_mode=shared" in output
    assert "ROS_MASTER_URI=http://127.0.0.1:11911" in output
    assert "GAZEBO_MASTER_URI=http://127.0.0.1:11945" in output


def test_tepper_cleanup_is_scoped_to_launched_process_groups():
    implementation = os.path.join(PACKAGE_ROOT, "scripts", "run_tepper_response_demo.sh")
    text = open(implementation, encoding="utf-8").read()

    assert "kill -INT -\"${pid}\"" in text
    assert "kill -TERM -\"${pid}\"" in text
    assert "pkill" not in text


def test_standalone_controller_demos_follow_production_controller_defaults():
    expected = {
        "roundflocking_controller_demo.launch": "heuristic",
        "transport_controller_demo.launch": "synchronized_slots",
    }
    for launch_name, controller in expected.items():
        root = ET.parse(os.path.join(PACKAGE_ROOT, "launch", launch_name)).getroot()
        controller_arg = next(
            element for element in root.findall("arg")
            if element.attrib.get("name") == "controller"
        )
        assert controller_arg.attrib["default"] == controller

        mux = next(
            node for node in root.findall("node")
            if node.attrib.get("type") == "response_cmd_mux_node.py"
        )
        external_names = next(
            element for element in mux.findall("rosparam")
            if element.attrib.get("param") == "external_robot_names"
        )
        assert yaml.safe_load(external_names.text) == []


def test_plain_controller_demo_aliases_select_production_controllers():
    script = os.path.abspath(
        os.path.join(PACKAGE_ROOT, "..", "..", "..", "run_response_controller_demo.sh")
    )
    for task, expected in (
        ("roundflocking", "controller:=heuristic"),
        ("transport", "controller:=synchronized_slots"),
    ):
        output = subprocess.check_output(
            [script, "--dry-run", "--headless", "--no-vglrun", task],
            cwd=os.path.dirname(script),
            text=True,
        )
        assert expected in output
