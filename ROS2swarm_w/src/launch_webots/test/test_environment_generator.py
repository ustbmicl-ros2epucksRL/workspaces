from pathlib import Path

import pytest

from launch_webots.environment_generator import (
    ROUND_FLOCKING_TARGET_CLEARANCE,
    TRANSPORT_OBJECT_MASS,
    generate_world_file,
    inset_bounds_for_clearance,
)


WORLD_HEADER = """#VRML_SIM R2025a utf8
WorldInfo {
}
"""

UNIFORM_HEADING = "rotation 0 0 1 3.1415"


def _generate_world(tmp_path: Path, robot_type: str, num_robots: int = 2) -> str:
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
            "transport_object_shape": "cylinder",
        },
    )

    content = output_world.read_text()

    assert "geometry Cylinder" in content
    assert "boundingObject Cylinder" in content
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
            "transport_object_shape": "triangular_prism",
        },
    )

    content = output_world.read_text()

    assert "IndexedFaceSet" in content
    assert "boundingObject Group" in content
    assert "transport_box_mesh" in content
    assert f"mass {TRANSPORT_OBJECT_MASS}" in content


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
