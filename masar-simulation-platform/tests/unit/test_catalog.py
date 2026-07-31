import json
from pathlib import Path

import pytest

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


def test_manifests_have_required_runtime_contract() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    for manifest in catalog:
        assert manifest.minimum_free_bytes == 2_147_483_648
        assert manifest.display_profile
        assert manifest.data_profile
        assert manifest.artifact_profile
        assert manifest.stop_order
        assert isinstance(manifest.parameters, dict)
        assert manifest.steps
        for step in manifest.steps:
            assert step.grace_seconds.model_dump() == {"sigint": 10, "sigterm": 5, "sigkill": 5}


def test_readiness_contract_matches_spec_19_4_exactly() -> None:
    catalog = ScenarioCatalog.from_directory(ROOT / "scenarios")
    expected = {
        "swarm.webots.ros2swarm": [
            (180, (("tcp", "127.0.0.1:18141"), ("ros2_topic", "/clock"), ("ros2_topic", "/robot_0/odom")))
        ],
        "control.gazebo.response-deform": [
            (120, (("ros1_topic", "/clock"), ("ros1_topic", "/tb_0/odom"), ("ros1_topic", "/response_group/status")))
        ],
        "control.gazebo.response-controller": [
            (120, (("ros1_topic", "/clock"), ("ros1_topic", "/tb_0/odom"), ("ros1_topic", "/response_group/status")))
        ],
        "exploration.gazebo.tepper": [
            (180, (("ros1_topic", "/clock"), ("ros1_topic", "/sim_map"), ("ros1_topic", "/percent_completed")))
        ],
        "exploration.webots.robosar": [
            (180, (("ros2_topic", "/clock"), ("ros2_topic", "/robot_0/odom"))),
            (180, (("ros1_topic", "/sim_map"), ("ros1_topic", "/percent_completed"))),
        ],
        "rescue.gazebo.tepper-response": [
            (180, (("ros1_topic", "/clock"), ("ros1_topic", "/sim_map"), ("ros1_topic", "/response_group/status")))
        ],
    }

    actual = {
        manifest.scenario_id: [
            (
                step.start_timeout_seconds,
                tuple((probe.type, probe.name) for probe in step.readiness.all),
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

    rescue = catalog.get("rescue.gazebo.tepper-response").model_copy(update={"simulator": "webots"})
    with pytest.raises(CatalogError, match="webots|incompatible"):
        catalog.validate_manifest(rescue)


def test_dependency_cycles_and_unknown_references_are_rejected(tmp_path: Path) -> None:
    source = ROOT / "scenarios" / "control.gazebo.response-deform.yaml"
    text = source.read_text(encoding="utf-8").replace("depends_on: []", "depends_on: [response_demo]")
    text = text.replace("id: response_demo", "id: response_demo\n    depends_on: [response_demo]")
    bad = tmp_path / "cycle.yaml"
    bad.write_text(text, encoding="utf-8")
    with pytest.raises(CatalogError, match="cycle|depend"):
        ScenarioCatalog.from_manifest(bad)
