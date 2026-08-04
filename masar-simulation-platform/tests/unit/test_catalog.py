import json
from collections.abc import Callable, Mapping
from copy import deepcopy
from pathlib import Path

import pytest
import yaml
from jsonschema import Draft202012Validator

from masar_platform.catalog import CatalogError, ScenarioCatalog
from masar_platform.contracts import ScenarioManifest


ROOT = Path(__file__).resolve().parents[2]


def test_catalog_registers_exactly_six_public_scenarios() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    assert catalog.scenario_ids == (
        "swarm.webots.ros2swarm",
        "control.gazebo.response-deform",
        "control.gazebo.response-controller",
        "exploration.gazebo.tepper",
        "exploration.webots.robosar",
        "rescue.gazebo.tepper-response",
    )
    assert len(catalog) == 6


def test_controller_demo_exposes_five_distinct_strategy_modes() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    mode = catalog.get("control.gazebo.response-controller").parameters["mode"]

    assert mode.default == "roundflocking-heuristic"
    assert mode.enum == (
        "roundflocking-checkpoint",
        "roundflocking-heuristic",
        "transport",
        "transport-checkpoint",
        "transport-heuristic",
    )


def test_manifests_have_required_runtime_contract() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    for manifest in catalog:
        assert manifest.minimum_free_bytes == 2_147_483_648
        assert manifest.runtime.ros1_master_uri == "http://127.0.0.1:11311"
        assert 0 <= manifest.runtime.ros2_domain_id <= 232
        assert manifest.display_profile
        assert manifest.data_profile
        assert manifest.artifact_profile
        assert manifest.stop_order
        assert isinstance(manifest.parameters, Mapping)
        assert manifest.steps
        for step in manifest.steps:
            assert step.grace_seconds.model_dump() == {"sigint": 10, "sigterm": 5, "sigkill": 5}


def test_readiness_contract_matches_spec_19_4_exactly() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    expected = {
        "swarm.webots.ros2swarm": [
            (
                180,
                (
                    ("ros2_topic", "/clock", "rosgraph_msgs/msg/Clock"),
                    ("ros2_topic", "/robot_0/odom", "nav_msgs/msg/Odometry"),
                ),
            )
        ],
        "control.gazebo.response-deform": [
            (
                120,
                (
                    ("ros1_topic", "/clock", "rosgraph_msgs/Clock"),
                    ("ros1_topic", "/tb_0/odom", "nav_msgs/Odometry"),
                    ("ros1_topic", "/response_group/status", "robosar_messages/ResponseGroupStatus"),
                ),
            )
        ],
        "control.gazebo.response-controller": [
            (
                120,
                (
                    ("ros1_topic", "/clock", "rosgraph_msgs/Clock"),
                    ("ros1_topic", "/tb_0/odom", "nav_msgs/Odometry"),
                    ("ros1_topic", "/response_group/behavior_mode", "std_msgs/String"),
                ),
            )
        ],
        "exploration.gazebo.tepper": [
            (
                180,
                (
                    ("ros1_topic", "/clock", "rosgraph_msgs/Clock"),
                    ("ros1_topic", "/sim_map", "nav_msgs/OccupancyGrid"),
                    (
                        "ros1_topic",
                        "/frontier_filter/filtered_frontiers",
                        "robosar_messages/PointArray",
                    ),
                ),
            )
        ],
        "exploration.webots.robosar": [
            (
                180,
                (
                    ("ros2_topic", "/clock", "rosgraph_msgs/msg/Clock"),
                    ("ros2_topic", "/robot_0/odom", "nav_msgs/msg/Odometry"),
                ),
            ),
            (
                180,
                (
                    ("ros1_topic", "/sim_map", "nav_msgs/OccupancyGrid"),
                    (
                        "ros1_topic",
                        "/frontier_filter/filtered_frontiers",
                        "robosar_messages/PointArray",
                    ),
                ),
            ),
        ],
        "rescue.gazebo.tepper-response": [
            (
                180,
                (
                    ("ros1_topic", "/clock", "rosgraph_msgs/Clock"),
                    ("ros1_topic", "/sim_map", "nav_msgs/OccupancyGrid"),
                    ("ros1_topic", "/response_group/status", "robosar_messages/ResponseGroupStatus"),
                ),
            )
        ],
    }

    actual = {
        manifest.scenario_id: [
            (
                step.start_timeout_seconds,
                tuple((probe.type, probe.name, probe.message_type) for probe in step.readiness.all),
            )
            for step in manifest.steps
        ]
        for manifest in catalog
    }
    assert actual == expected


def test_depends_on_is_required_by_runtime_model_and_json_schema(tmp_path: Path) -> None:
    source = ROOT / "scenarios" / "control.gazebo.response-deform.yaml"
    bad = tmp_path / "missing-dependency-field.yaml"
    bad.write_text(source.read_text(encoding="utf-8").replace("    depends_on: []\n", ""), encoding="utf-8")
    with pytest.raises(CatalogError, match="depends_on|required|missing"):
        ScenarioCatalog.from_manifest(bad)

    file_schema = json.loads((ROOT / "scenarios" / "schema.json").read_text(encoding="utf-8"))
    model_schema = ScenarioManifest.model_json_schema()
    assert set(file_schema["$defs"]["step"]["required"]) == set(
        model_schema["$defs"]["ScenarioStep"]["required"]
    )


@pytest.mark.parametrize(
    ("parameter", "mutation"),
    (
        ("pattern", lambda definition: definition.pop("enum")),
        ("number_robots", lambda definition: definition.pop("minimum")),
        ("pattern", lambda definition: definition.update(enum=["roundflocking_pattern"] * 2)),
        ("number_robots", lambda definition: definition.update(minimum=True)),
    ),
)
def test_parameter_definition_failures_match_json_schema_and_runtime(
    parameter: str,
    mutation: Callable[[dict[str, object]], object],
) -> None:
    payload = yaml.safe_load((ROOT / "scenarios" / "swarm.webots.ros2swarm.yaml").read_text(encoding="utf-8"))
    invalid = deepcopy(payload)
    mutation(invalid["parameters"][parameter])

    schema = json.loads((ROOT / "scenarios" / "schema.json").read_text(encoding="utf-8"))
    assert list(Draft202012Validator(schema).iter_errors(invalid))
    with pytest.raises((ValueError, TypeError)):
        ScenarioManifest.model_validate(invalid)


def test_catalog_parameter_contract_cannot_be_mutated() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    manifest = catalog.get("swarm.webots.ros2swarm")

    with pytest.raises(TypeError):
        manifest.parameters["number_robots"] = manifest.parameters["total_robots"]  # type: ignore[index]

    copied = manifest.model_copy(update={"parameters": dict(manifest.parameters)})
    with pytest.raises(TypeError):
        copied.parameters["number_robots"] = copied.parameters["total_robots"]  # type: ignore[index]

    serialized = json.loads(copied.model_dump_json())
    assert serialized["parameters"]["number_robots"]["default"] == 5


def test_all_platform_scenarios_share_the_ros2_web_domain() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")

    assert {manifest.runtime.ros2_domain_id for manifest in catalog} == {42}


def test_unknown_manifest_fields_are_rejected(tmp_path: Path) -> None:
    source = ROOT / "scenarios" / "swarm.webots.ros2swarm.yaml"
    payload = source.read_text(encoding="utf-8") + "\nunknown_field: true\n"
    bad = tmp_path / "bad.yaml"
    bad.write_text(payload, encoding="utf-8")
    with pytest.raises(CatalogError, match="unknown_field|extra|forbidden"):
        ScenarioCatalog.from_manifest(bad)


def test_invalid_parameters_and_incompatible_combinations_are_rejected(tmp_path: Path) -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    swarm = catalog.get("swarm.webots.ros2swarm")
    with pytest.raises(CatalogError):
        catalog.validate_parameters(swarm.scenario_id, {"number_robots": 0})
    controller = catalog.get("control.gazebo.response-controller")
    with pytest.raises(CatalogError):
        catalog.validate_parameters(controller.scenario_id, {"mode": "unsupported"})

    with pytest.raises(ValueError, match="webots|supported"):
        catalog.get("rescue.gazebo.tepper-response").model_copy(update={"simulator": "webots"})


def test_swarm_world_is_fixed_while_robot_type_remains_selectable() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    swarm = catalog.get("swarm.webots.ros2swarm")

    assert swarm.parameters["webots_world"].enum == ("command_world.wbt",)
    assert swarm.parameters["robot"].enum == ("e_puck", "burger")
    with pytest.raises(CatalogError):
        catalog.validate_parameters(swarm.scenario_id, {"webots_world": "obstacle_world.wbt"})


def test_dependency_cycles_and_unknown_references_are_rejected(tmp_path: Path) -> None:
    source = ROOT / "scenarios" / "control.gazebo.response-deform.yaml"
    text = source.read_text(encoding="utf-8").replace("depends_on: []", "depends_on: [response_demo]")
    text = text.replace("id: response_demo", "id: response_demo\n    depends_on: [response_demo]")
    bad = tmp_path / "cycle.yaml"
    bad.write_text(text, encoding="utf-8")
    with pytest.raises(CatalogError, match="cycle|depend"):
        ScenarioCatalog.from_manifest(bad)
