from __future__ import annotations

import json
from pathlib import Path
import re

import pytest

from masar_platform.profiles import TOPIC_PROFILES, get_display_profile
from masar_platform.state_machine import RunState
from tests.platform_fakes import build_test_orchestrator


ROOT = Path(__file__).resolve().parents[2]
FIXTURES = ROOT / "tests" / "fixtures"
SCENARIO_EXPECTATIONS = FIXTURES / "processes" / "scenarios.json"
GRAPH_FIXTURES = FIXTURES / "ros_graph"


def _json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def _topic_name_matches(pattern: str, name: str) -> bool:
    expression = re.escape(pattern)
    expression = expression.replace(re.escape("<robot>"), r"[^/]+")
    expression = expression.replace(re.escape("<agent>"), r"[^/]+")
    expression = expression.replace(re.escape("tb_i"), r"tb_[0-9]+")
    return re.fullmatch(expression, name) is not None


@pytest.mark.parametrize("profile_name", sorted(TOPIC_PROFILES))
def test_ros_graph_fixtures_cover_every_required_profile_topic(profile_name: str) -> None:
    fixture = _json(GRAPH_FIXTURES / f"{profile_name}.json")
    profile = TOPIC_PROFILES[profile_name]

    assert fixture["schema_version"] == 1
    assert fixture["profile"] == profile_name
    assert set(fixture["graphs"]) <= {"ros1", "ros2"}
    observed = [topic for graph in fixture["graphs"].values() for topic in graph["topics"]]
    assert all(topic["last_message_age_seconds"] >= 0 for topic in observed)
    for required in (topic for topic in profile.topics if topic.required):
        assert any(required.matches(topic["name"], topic["message_type"]) for topic in observed), required


@pytest.mark.parametrize(
    ("case_id", "expected"),
    (
        ("missing_camera", "UNAVAILABLE"),
        ("stale_odom", "STALE"),
        ("wrong_odom_type", "TYPE_MISMATCH"),
        ("disconnected_rosbridge", "DISCONNECTED"),
    ),
)
def test_ros_graph_failure_fixtures_are_truthful(case_id: str, expected: str) -> None:
    cases = _json(GRAPH_FIXTURES / "failure_cases.json")["cases"]
    case = next(item for item in cases if item["case_id"] == case_id)
    profile = TOPIC_PROFILES[case["profile"]]
    declared = next(item for item in profile.topics if item.capability == case["capability"])

    if not case["connected"]:
        availability = "DISCONNECTED"
    else:
        named = [item for item in case["topics"] if _topic_name_matches(declared.pattern, item["name"])]
        typed = [item for item in named if item["message_type"] in declared.message_types]
        if not named:
            availability = "UNAVAILABLE"
        elif not typed:
            availability = "TYPE_MISMATCH"
        elif typed[0]["last_message_age_seconds"] > case["stale_after_seconds"]:
            availability = "STALE"
        else:
            availability = "ONLINE"
    assert availability == expected


@pytest.mark.parametrize(
    "scenario_id",
    (
        "swarm.webots.ros2swarm",
        "control.gazebo.response-deform",
        "control.gazebo.response-controller",
        "exploration.gazebo.tepper",
        "exploration.webots.robosar",
        "rescue.gazebo.tepper-response",
    ),
)
def test_each_scenario_smoke_reaches_final_archive_without_invented_scores(
    tmp_path: Path,
    scenario_id: str,
) -> None:
    expected = _json(SCENARIO_EXPECTATIONS)["scenarios"][scenario_id]
    orchestrator, _, trace = build_test_orchestrator(tmp_path)
    manifest = orchestrator.catalog.get(scenario_id)

    run = orchestrator.create_run(scenario_id, {})
    resolved_steps = run.resolved_manifest["manifest"]["steps"]
    assert [step["command_ref"] for step in resolved_steps] == expected["command_refs"]
    assert [[probe["name"] for probe in step["readiness"]["all"]] for step in resolved_steps] == expected[
        "readiness"
    ]
    assert list(get_display_profile(manifest.display_profile).panels) == expected["panels"]

    assert orchestrator.start_run(run.run_id).state is RunState.RUNNING
    assert [item for item in trace if item.startswith("start:")] == expected["start_trace"]
    completed = orchestrator.stop_run(run.run_id)

    assert completed.state is RunState.COMPLETED
    assert [item for item in trace if item.startswith("stop:")] == expected["stop_trace"]
    run_dir = orchestrator.artifacts.run_dir(run.run_id)
    for relative in (
        "manifest.json",
        "summary.json",
        "events.jsonl",
        "resolved-scenario.yaml",
        "ros/topics-snapshot.json",
        "ros/nodes-snapshot.json",
        "checksums.sha256",
    ):
        assert (run_dir / relative).is_file(), relative
    archived_manifest = _json(run_dir / "manifest.json")
    assert archived_manifest["scenario_id"] == scenario_id
    assert archived_manifest["final_state"] == "COMPLETED"
    assert archived_manifest["recording"] == {
        "enabled": False,
        "profile": "default",
        "recorders": [],
    }
    serialized = json.dumps(archived_manifest, sort_keys=True).lower()
    for forbidden in ("aggregation_score", "cohesion_score", "alignment_score", "average_spacing"):
        assert forbidden not in serialized


def test_real_ros_topic_readiness_allows_cli_discovery_overhead() -> None:
    orchestrator, _, _ = build_test_orchestrator(Path("/tmp/unused-task11-artifacts"))

    for scenario in orchestrator.catalog:
        for step in scenario.steps:
            for probe in step.readiness.all:
                if probe.type in {"ros1_topic", "ros2_topic"}:
                    assert probe.max_age_seconds >= 5.0, (
                        f"{scenario.scenario_id}:{step.id}:{probe.name} must allow the ROS CLI "
                        "enough time to discover and receive a message under simulator load"
                    )
