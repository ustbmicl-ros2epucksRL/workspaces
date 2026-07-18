import math
import os
import sys
import xml.etree.ElementTree as ET

import yaml


PACKAGE_ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from robosar_emergency_response.formation import (  # noqa: E402
    FormationSlot,
    Pose2D,
    assign_slots_by_current_geometry,
    generate_square_slots,
)


def _launch_arg_defaults(relative_path):
    launch_path = os.path.join(PACKAGE_ROOT, relative_path)
    root = ET.parse(launch_path).getroot()
    return {
        element.attrib["name"]: element.attrib.get("default")
        for element in root.findall("arg")
        if "name" in element.attrib
    }


def test_response_demo_initial_square_keeps_current_corner_assignment():
    slots = generate_square_slots(4, 0.6)
    robot_names = ["tb_0", "tb_1", "tb_2", "tb_3"]
    robot_poses = {
        "tb_0": Pose2D(4.2, -4.6),
        "tb_1": Pose2D(4.8, -4.6),
        "tb_2": Pose2D(4.2, -5.2),
        "tb_3": Pose2D(4.8, -5.2),
    }

    assignment = assign_slots_by_current_geometry(robot_names, robot_poses, slots, yaw=0.0)

    assert assignment == {
        "tb_0": 3,
        "tb_1": 0,
        "tb_2": 2,
        "tb_3": 1,
    }


def test_tepper_initial_square_uses_non_crossing_slot_assignment():
    defaults = _launch_arg_defaults("launch/tepper_response_demo.launch")
    with open(
        os.path.join(PACKAGE_ROOT, "config", "formation_navigation.yaml"),
        "r",
        encoding="utf-8",
    ) as fh:
        navigation = yaml.safe_load(fh)
    slots = generate_square_slots(4, float(navigation["formation_interval"]))
    robot_names = ["tb_0", "tb_1", "tb_2", "tb_3"]
    robot_poses = {
        name: Pose2D(float(defaults[f"{name}_x"]), float(defaults[f"{name}_y"]))
        for name in robot_names
    }

    assert [(robot_poses[name].x, robot_poses[name].y) for name in robot_names] == [
        (3.7, 16.7),
        (4.3, 16.7),
        (3.7, 16.1),
        (4.3, 16.1),
    ]
    assert assign_slots_by_current_geometry(robot_names, robot_poses, slots, yaw=0.0) == {
        "tb_0": 3,
        "tb_1": 0,
        "tb_2": 2,
        "tb_3": 1,
    }


def test_slot_assignment_uses_nearest_square_geometry_not_robot_id_order():
    slots = [
        FormationSlot(0.5, 0.5),
        FormationSlot(0.5, -0.5),
        FormationSlot(-0.5, -0.5),
        FormationSlot(-0.5, 0.5),
    ]
    robot_names = ["a", "b", "c", "d"]
    robot_poses = {
        "a": Pose2D(-0.52, 0.48),
        "b": Pose2D(0.47, 0.51),
        "c": Pose2D(-0.49, -0.46),
        "d": Pose2D(0.54, -0.50),
    }

    assignment = assign_slots_by_current_geometry(robot_names, robot_poses, slots, yaw=0.0)

    assert assignment == {"a": 3, "b": 0, "c": 2, "d": 1}


def test_slot_assignment_respects_rotated_current_formation():
    slots = generate_square_slots(4, 1.0)
    robot_names = ["tb_0", "tb_1", "tb_2", "tb_3"]
    yaw = math.pi / 4.0
    c = math.cos(yaw)
    s = math.sin(yaw)
    robot_poses = {}
    for name, slot_index in zip(robot_names, [3, 0, 2, 1]):
        slot = slots[slot_index]
        robot_poses[name] = Pose2D(c * slot.x - s * slot.y, s * slot.x + c * slot.y)

    assignment = assign_slots_by_current_geometry(robot_names, robot_poses, slots, yaw=yaw)

    assert assignment == {
        "tb_0": 3,
        "tb_1": 0,
        "tb_2": 2,
        "tb_3": 1,
    }
