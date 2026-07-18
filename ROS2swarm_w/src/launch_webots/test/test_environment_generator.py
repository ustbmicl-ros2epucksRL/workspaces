from pathlib import Path

import pytest

from launch_webots.environment_generator import (
    ROUND_FLOCKING_TARGET_CLEARANCE,
    NoCollisionFreePositionError,
    TRANSPORT_OBJECT_MASS,
    TRANSPORT_OBJECT_SIZE,
    TRANSPORT_TARGET_ZONE_RADIUS,
    generate_grid_positions,
    generate_random_positions_with_obstacles,
    navigation_grid_columns,
    generate_two_column_positions,
    generate_world_file,
    inset_bounds_for_clearance,
    is_position_collision_free,
    occupied_positions_from_global_info,
)


WORLD_HEADER = """#VRML_SIM R2025a utf8
WorldInfo {
}
"""

UNIFORM_HEADING = "rotation 0 0 1 3.1415"


def _generate_world(
    tmp_path: Path,
    robot_type: str,
    num_robots: int = 2,
) -> str:
    template_world = tmp_path / "template.wbt"
    output_world = tmp_path / "generated.wbt"
    template_world.write_text(WORLD_HEADER)

    generate_world_file(
        str(template_world),
        str(output_world),
        robot_type,
        num_robots,
        positions=[(0.1, 0.2), (0.3, 0.4)],
        z_height=0.0,
        task_objects=None,
    )
    return output_world.read_text()


def test_generate_world_file_sets_consistent_epuck_heading(tmp_path: Path):
    content = _generate_world(tmp_path, "e_puck")

    assert content.count(UNIFORM_HEADING) == 2


def test_generate_world_file_sets_consistent_burger_heading(tmp_path: Path):
    content = _generate_world(tmp_path, "burger")

    assert content.count(UNIFORM_HEADING) == 2


def test_generate_two_column_positions_for_ten_navigation_targets():
    positions = generate_two_column_positions(10, -1.0, 1.0, -1.0, 1.0)

    expected = [
        -0.5, -0.8,
        -0.5, -0.4,
        -0.5, 0.0,
        -0.5, 0.4,
        -0.5, 0.8,
        0.5, -0.8,
        0.5, -0.4,
        0.5, 0.0,
        0.5, 0.4,
        0.5, 0.8,
    ]
    flattened = [coordinate for position in positions for coordinate in position]
    assert flattened == pytest.approx(expected)


def test_generate_grid_positions_supports_non_ten_counts():
    positions = generate_grid_positions(7, -1.0, 1.0, -1.0, 1.0)

    expected = [
        0.0, -0.8,
        0.0, -0.5333333333,
        0.0, -0.2666666667,
        0.0, 0.0,
        0.0, 0.2666666667,
        0.0, 0.5333333333,
        0.0, 0.8,
    ]
    flattened = [coordinate for position in positions for coordinate in position]
    assert flattened == pytest.approx(expected)


def test_generate_grid_positions_uses_smallest_factor_for_odd_composites():
    positions = generate_grid_positions(9, -1.0, 1.0, -1.0, 1.0)

    expected = [
        -0.5, -0.8,
        -0.5, 0.0,
        -0.5, 0.8,
        0.0, -0.8,
        0.0, 0.0,
        0.0, 0.8,
        0.5, -0.8,
        0.5, 0.0,
        0.5, 0.8,
    ]
    flattened = [coordinate for position in positions for coordinate in position]
    assert flattened == pytest.approx(expected)


def test_navigation_grid_column_rule():
    assert navigation_grid_columns(10) == 2
    assert navigation_grid_columns(7) == 1
    assert navigation_grid_columns(9) == 3


def test_generate_world_file_defaults_transport_object_shape_to_cube(tmp_path: Path):
    template_world = tmp_path / "template.wbt"
    output_world = tmp_path / "generated.wbt"
    template_world.write_text(WORLD_HEADER)

    generate_world_file(
        str(template_world),
        str(output_world),
        "e_puck",
        0,
        positions=[],
        z_height=0.0,
        task_objects={"transport_box": (0.1, 0.2)},
    )

    content = output_world.read_text()

    assert "geometry Box" in content
    assert "boundingObject Box" in content
    assert f"mass {TRANSPORT_OBJECT_MASS}" in content


def test_generate_world_file_supports_cylinder_transport_object(tmp_path: Path):
    template_world = tmp_path / "template.wbt"
    output_world = tmp_path / "generated.wbt"
    template_world.write_text(WORLD_HEADER)

    generate_world_file(
        str(template_world),
        str(output_world),
        "e_puck",
        0,
        positions=[],
        z_height=0.0,
        task_objects={
            "transport_box": (0.1, 0.2),
            "target_zone": (0.3, 0.4),
            "transport_object_shape": "cylinder",
        },
    )

    content = output_world.read_text()

    assert "geometry Cylinder" in content
    assert "boundingObject Cylinder" in content
    assert f"radius {TRANSPORT_OBJECT_SIZE / 2.0}" in content
    assert f"radius {TRANSPORT_TARGET_ZONE_RADIUS}" in content
    assert f"mass {TRANSPORT_OBJECT_MASS}" in content


def test_generate_world_file_supports_triangular_prism_transport_object(tmp_path: Path):
    template_world = tmp_path / "template.wbt"
    output_world = tmp_path / "generated.wbt"
    template_world.write_text(WORLD_HEADER)

    generate_world_file(
        str(template_world),
        str(output_world),
        "e_puck",
        0,
        positions=[],
        z_height=0.0,
        task_objects={
            "transport_box": (0.1, 0.2),
            "target_zone": (0.3, 0.4),
            "transport_object_shape": "triangular_prism",
        },
    )

    content = output_world.read_text()

    assert "IndexedFaceSet" in content
    assert "boundingObject Group" in content
    assert "transport_box_mesh" in content
    assert f"{TRANSPORT_OBJECT_SIZE / 2.0} {TRANSPORT_OBJECT_SIZE / 2.0}" in content
    assert f"radius {TRANSPORT_TARGET_ZONE_RADIUS}" in content
    assert f"mass {TRANSPORT_OBJECT_MASS}" in content


def test_transport_object_mass_is_point_seven_kilograms():
    assert TRANSPORT_OBJECT_MASS == pytest.approx(0.7)


def test_generate_world_file_applies_shape_to_transport_box_lists(tmp_path: Path):
    template_world = tmp_path / "template.wbt"
    output_world = tmp_path / "generated.wbt"
    template_world.write_text(WORLD_HEADER)

    generate_world_file(
        str(template_world),
        str(output_world),
        "e_puck",
        0,
        positions=[],
        z_height=0.0,
        task_objects={
            "transport_boxes": [(0.1, 0.2), (0.3, 0.4)],
            "transport_object_shape": "cylinder",
        },
    )

    content = output_world.read_text()

    assert content.count("geometry Cylinder") == 2


def test_roundflocking_target_bounds_keep_ring_clearance():
    lower, upper = inset_bounds_for_clearance(
        -1.0,
        1.0,
        ROUND_FLOCKING_TARGET_CLEARANCE,
    )

    assert lower == pytest.approx(-0.4)
    assert upper == pytest.approx(0.4)


def test_inset_bounds_falls_back_to_center_when_area_is_too_small():
    lower, upper = inset_bounds_for_clearance(-0.5, 0.5, 0.6)

    assert lower == 0.0
    assert upper == 0.0


def test_random_position_generation_fails_when_area_is_fully_occupied():
    with pytest.raises(NoCollisionFreePositionError, match='could not place item'):
        generate_random_positions_with_obstacles(
            1,
            -0.1,
            0.1,
            -0.1,
            0.1,
            min_distance=0.0,
            seed=1,
            existing_positions=[(0.0, 0.0, 1.0)],
        )


def test_global_info_occupancy_uses_scene_object_collision_radii():
    occupied = occupied_positions_from_global_info({
        'robots': {
            'robot_0': {'position': {'x': 0.1, 'y': 0.2}},
        },
        'objects': {
            'transport_box': {'position': {'x': 0.3, 'y': 0.4}},
            'obstacle_0': {'position': {'x': 0.5, 'y': 0.6}},
        },
    })

    assert occupied == pytest.approx([
        (0.1, 0.2, 0.0),
        (0.3, 0.4, TRANSPORT_OBJECT_SIZE / 2.0),
        (0.5, 0.6, 0.1),
    ])


def test_position_collision_check_matches_random_generator_clearance():
    occupied = [(0.0, 0.0, 0.1), (1.0, 1.0, (0.4, 0.4))]

    assert not is_position_collision_free(0.2, 0.0, 0.15, occupied)
    assert not is_position_collision_free(1.3, 1.0, 0.15, occupied)
    assert is_position_collision_free(0.5, 0.5, 0.15, occupied)
