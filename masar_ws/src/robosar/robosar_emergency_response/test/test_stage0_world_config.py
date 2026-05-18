from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


PACKAGE_DIR = Path(__file__).resolve().parents[1]
WORLD = PACKAGE_DIR / "worlds" / "deform_navigation_demo.world"
OLD_WORLD = PACKAGE_DIR / "worlds" / "stage0_deform_navigation_reference.world"
GOAL_SEQUENCE = PACKAGE_DIR / "config" / "stage0_goal_sequence.yaml"
DEFORM_ENV = (
    PACKAGE_DIR.parents[1]
    / "robosar_deform_vendor"
    / "perception"
    / "plan_env2d"
    / "config"
    / "env.yaml"
)
DEFORM_GRID_MAP_HEADER = (
    PACKAGE_DIR.parents[1]
    / "robosar_deform_vendor"
    / "perception"
    / "plan_env2d"
    / "include"
    / "plan_env"
    / "grid_map.h"
)
DEFORM_GRID_MAP_SOURCE = (
    PACKAGE_DIR.parents[1]
    / "robosar_deform_vendor"
    / "perception"
    / "plan_env2d"
    / "src"
    / "grid_map.cpp"
)
D435_GAZEBO_XACRO = (
    PACKAGE_DIR.parents[1]
    / "robosar_deform_vendor"
    / "utility"
    / "tb_simulation"
    / "realsense2_description"
    / "urdf"
    / "_d435.gazebo.xacro"
)

SENSOR_VISIBLE_FLAG = 1
GOAL_MARKER_VISIBILITY_FLAG = 2


def _world_model_names():
    tree = ET.parse(WORLD)
    return {model.attrib["name"] for model in tree.findall(".//model")}


def _stage0_goals():
    with GOAL_SEQUENCE.open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)["goals"]


def _mobile_robot_height():
    with DEFORM_ENV.open("r", encoding="utf-8") as stream:
        return float(yaml.safe_load(stream)["grid_map"]["mobile_robot_height"])


def _ground_height():
    with DEFORM_ENV.open("r", encoding="utf-8") as stream:
        return float(yaml.safe_load(stream)["grid_map"]["ground_height"])


def test_stage0_world_uses_requested_demo_name_only():
    assert WORLD.exists()
    assert not OLD_WORLD.exists()


def test_stage0_world_contains_reference_demo_geometry():
    names = _world_model_names()

    assert {"boundary_north", "boundary_south", "boundary_west", "boundary_east"} <= names
    assert "rectangular_wall_obstacle" in names
    assert "elliptical_obstacle" in names


def test_stage0_goal_markers_are_not_gazebo_obstacles():
    names = _world_model_names()

    assert not [name for name in names if name.startswith("white_goal_marker_")]

    tree = ET.parse(WORLD)
    for goal in _stage0_goals():
        marker = tree.find(".//model[@name='yellow_goal_marker_{}']".format(goal["id"]))
        assert marker is not None
        assert marker.find(".//collision") is None


def test_stage0_gazebo_goal_markers_are_yellow_and_match_goal_sequence():
    tree = ET.parse(WORLD)

    for goal in _stage0_goals():
        marker = tree.find(".//model[@name='yellow_goal_marker_{}']".format(goal["id"]))
        assert marker is not None
        pose = [float(value) for value in marker.findtext("pose").split()]
        length = float(marker.findtext(".//cylinder/length"))
        marker_bottom_z = pose[2] - length / 2.0

        assert pose[:2] == [goal["x"], goal["y"]]
        assert marker_bottom_z >= 0.0
        assert marker.findtext("static") == "1"
        assert marker.find(".//visual") is not None
        assert marker.findtext(".//material/ambient").startswith("1.0 0.86 0.0")
        assert marker.findtext(".//material/diffuse").startswith("1.0 0.86 0.0")


def test_stage0_goal_markers_are_visible_in_gazebo_but_hidden_from_robot_depth_cameras():
    tree = ET.parse(WORLD)

    for goal in _stage0_goals():
        visual = tree.find(".//model[@name='yellow_goal_marker_{}']//visual".format(goal["id"]))
        assert visual is not None
        assert int(visual.findtext("visibility_flags"), 0) == GOAL_MARKER_VISIBILITY_FLAG

    xacro = D435_GAZEBO_XACRO.read_text(encoding="utf-8")
    assert "<visibility_mask>{}</visibility_mask>".format(SENSOR_VISIBLE_FLAG) in xacro
    assert "<visibility_mask>{}</visibility_mask>".format(GOAL_MARKER_VISIBILITY_FLAG) not in xacro


def test_deform_grid_map_keeps_mobile_robot_height_as_float_threshold():
    header = DEFORM_GRID_MAP_HEADER.read_text(encoding="utf-8")
    source = DEFORM_GRID_MAP_SOURCE.read_text(encoding="utf-8")

    assert "double car_height" in header
    assert 'node_.param("grid_map/mobile_robot_height", mp_.car_height, 0.3)' in source
    assert _mobile_robot_height() == 0.3


def test_deform_grid_map_ignores_low_active_goal_region_obstacles_only():
    with DEFORM_ENV.open("r", encoding="utf-8") as stream:
        grid_map = yaml.safe_load(stream)["grid_map"]
    header = DEFORM_GRID_MAP_HEADER.read_text(encoding="utf-8")
    source = DEFORM_GRID_MAP_SOURCE.read_text(encoding="utf-8")

    assert 0.35 <= float(grid_map["goal_ignore_radius"]) <= 0.60
    assert 0.05 <= float(grid_map["goal_ignore_max_height"]) < _mobile_robot_height()
    assert "activeGoalCallback" in header
    assert "shouldIgnoreGoalRegionPoint" in header
    assert 'node_.subscribe("/move_base_simple/goal"' in source
    assert "grid_map/goal_ignore_radius" in source
    assert "grid_map/goal_ignore_max_height" in source
    assert "shouldIgnoreGoalRegionPoint(md_.proj_points_[i])" in source


def test_stage0_goal_markers_stay_below_the_ground_filter():
    tree = ET.parse(WORLD)
    marker_heights = []

    for goal in _stage0_goals():
        marker = tree.find(".//model[@name='yellow_goal_marker_{}']".format(goal["id"]))
        assert marker is not None
        pose = [float(value) for value in marker.findtext("pose").split()]
        length = float(marker.findtext(".//cylinder/length"))
        marker_heights.append(pose[2] + length / 2.0)

    assert max(marker_heights) < _ground_height()
    assert _ground_height() < _mobile_robot_height()


def test_stage0_goal_sequence_node_publishes_rviz_goal_markers():
    node = (PACKAGE_DIR / "scripts" / "response_goal_sequence_node.py").read_text()

    assert "/response_group/stage0_goal_markers" in node
    assert "MarkerArray" in node
    assert "marker.color.r = 1.0" in node


def test_stage0_goal_sequence_has_five_safe_waypoints():
    goals = _stage0_goals()
    assert len(goals) == 5
    assert [goal["id"] for goal in goals] == [0, 1, 2, 3, 4]

    for goal in goals:
        assert not _inside_stage0_obstacle(goal["x"], goal["y"])


def test_first_goal_keeps_single_file_tail_clear_of_west_wall():
    with GOAL_SEQUENCE.open("r", encoding="utf-8") as stream:
        config = yaml.safe_load(stream)

    first_goal = config["goals"][0]
    single_file_tail_offset = -2.4
    west_wall_inner_x = -6.2 + 0.08

    assert first_goal["x"] + single_file_tail_offset >= west_wall_inner_x + 0.30


def _inside_stage0_obstacle(x, y):
    inside_rectangle = -3.25 <= x <= -1.45 and -3.5 <= y <= 3.5
    inside_ellipse_footprint = 1.35 <= x <= 3.35 and -3.8 <= y <= 3.8
    return inside_rectangle or inside_ellipse_footprint


def test_stage0_world_has_centered_square_boundary():
    tree = ET.parse(WORLD)
    north = tree.find(".//model[@name='boundary_north']")
    south = tree.find(".//model[@name='boundary_south']")
    west = tree.find(".//model[@name='boundary_west']")
    east = tree.find(".//model[@name='boundary_east']")

    north_pose = [float(value) for value in north.findtext("pose").split()]
    south_pose = [float(value) for value in south.findtext("pose").split()]
    west_pose = [float(value) for value in west.findtext("pose").split()]
    east_pose = [float(value) for value in east.findtext("pose").split()]
    north_size = [float(value) for value in north.findtext(".//box/size").split()]
    west_size = [float(value) for value in west.findtext(".//box/size").split()]

    assert (north_pose[0], south_pose[0], west_pose[1], east_pose[1]) == (0.0, 0.0, 0.0, 0.0)
    assert (north_pose[1], south_pose[1], west_pose[0], east_pose[0]) == (6.2, -6.2, -6.2, 6.2)
    assert (north_size[0], west_size[1]) == (12.4, 12.4)


def test_stage0_world_has_side_by_side_obstacles_with_a_central_passage():
    tree = ET.parse(WORLD)
    rect = tree.find(".//model[@name='rectangular_wall_obstacle']")
    ellipse = tree.find(".//model[@name='elliptical_obstacle']")

    assert rect is not None
    assert ellipse is not None
    rect_pose = [float(value) for value in rect.findtext("pose").split()]
    ellipse_pose = [float(value) for value in ellipse.findtext("pose").split()]
    rect_size = [float(value) for value in rect.findtext(".//box/size").split()]
    ellipse_center_size = [
        float(value) for value in ellipse.findtext(".//link[@name='ellipse_center']//box/size").split()
    ]
    ellipse_radius = max(float(radius.text) for radius in ellipse.findall(".//cylinder/radius"))
    rect_east = rect_pose[0] + rect_size[0] / 2.0
    ellipse_west = ellipse_pose[0] - max(ellipse_center_size[0] / 2.0, ellipse_radius)
    central_passage_width = ellipse_west - rect_east

    assert rect_pose[0] == -ellipse_pose[0]
    assert rect_pose[1] == ellipse_pose[1] == 0.0
    assert 2.0 <= central_passage_width <= 3.0
    assert rect_size[1] > 6.0
    assert ellipse_center_size[1] + 2.0 * ellipse_radius > 7.0
