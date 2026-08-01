from __future__ import annotations

import hashlib
import json
from pathlib import Path
import sqlite3

import pytest

from agents.runtime_agent.processes import RECORDER_REGEXES
from masar_platform.artifacts import ArtifactNotFound
from masar_platform.db import Database
from masar_platform.recording import build_recorder_plans
from masar_platform.state_machine import ReasonCode, RunState
from tests.platform_fakes import build_test_orchestrator


MANIFEST_KEYS = {
    "run_id",
    "scenario_id",
    "parameters",
    "platform_version",
    "agent_versions",
    "started_at",
    "ended_at",
    "final_state",
    "failure_reason",
    "step_results",
    "artifact_ids",
    "recording",
}

SUMMARY_KEYS = {
    "run_id",
    "duration_seconds",
    "final_state",
    "missing_data",
    "topic_health",
    "task_counts",
    "exploration_progress_final",
    "response_group",
}


def test_legacy_artifact_primary_key_schema_migrates_without_data_loss(tmp_path: Path) -> None:
    database = Database(tmp_path / "legacy.sqlite3")
    database.initialize()
    with database.connect() as connection:
        connection.execute(
            """
            INSERT INTO runs (run_id, scenario_id, state, created_at, updated_at)
            VALUES ('run-legacy', 'control.gazebo.response-deform', 'COMPLETED',
                    '2026-01-01T00:00:00Z', '2026-01-01T00:00:01Z')
            """
        )
        connection.execute("DROP INDEX idx_artifacts_run_content")
        connection.execute("DROP INDEX idx_artifacts_run_id")
        connection.execute("ALTER TABLE artifacts RENAME TO artifacts_new")
        connection.execute(
            """
            CREATE TABLE artifacts (
                artifact_id TEXT PRIMARY KEY,
                run_id TEXT NOT NULL,
                kind TEXT NOT NULL,
                path TEXT NOT NULL,
                size_bytes INTEGER,
                checksum TEXT,
                created_at TEXT NOT NULL
            )
            """
        )
        connection.execute(
            """
            INSERT INTO artifacts VALUES
            ('artifact-old', 'run-legacy', 'log', '/tmp/legacy.log', 3, 'sha256:old', '2026-01-01T00:00:01Z')
            """
        )
        connection.execute("DROP TABLE artifacts_new")

    database.initialize()

    with sqlite3.connect(database.path) as connection:
        columns = {row[1] for row in connection.execute("PRAGMA table_info(artifacts)")}
        row = connection.execute("SELECT artifact_id, run_id, path FROM artifacts").fetchone()
    assert "artifact_key" in columns
    assert row == ("artifact-old", "run-legacy", "/tmp/legacy.log")


def test_interrupted_legacy_artifact_migration_is_recovered(tmp_path: Path) -> None:
    database = Database(tmp_path / "interrupted.sqlite3")
    database.initialize()
    with database.connect() as connection:
        connection.execute(
            """
            INSERT INTO runs (run_id, scenario_id, state, created_at, updated_at)
            VALUES ('run-legacy', 'control.gazebo.response-deform', 'COMPLETED',
                    '2026-01-01T00:00:00Z', '2026-01-01T00:00:01Z')
            """
        )
        connection.execute(
            """
            CREATE TABLE artifacts_legacy (
                artifact_id TEXT PRIMARY KEY, run_id TEXT NOT NULL, kind TEXT NOT NULL,
                path TEXT NOT NULL, size_bytes INTEGER, checksum TEXT, created_at TEXT NOT NULL
            )
            """
        )
        connection.execute(
            """
            INSERT INTO artifacts_legacy VALUES
            ('artifact-old', 'run-legacy', 'log', '/tmp/legacy.log', 3, 'sha256:old', '2026-01-01T00:00:01Z')
            """
        )

    database.initialize()

    with database.connect() as connection:
        assert connection.execute("SELECT artifact_id FROM artifacts").fetchone()[0] == "artifact-old"
        assert connection.execute(
            "SELECT name FROM sqlite_master WHERE type='table' AND name='artifacts_legacy'"
        ).fetchone() is None


def test_legacy_duplicate_paths_abort_migration_without_dropping_rows(tmp_path: Path) -> None:
    database = Database(tmp_path / "duplicates.sqlite3")
    database.initialize()
    with database.connect() as connection:
        connection.execute(
            """
            INSERT INTO runs (run_id, scenario_id, state, created_at, updated_at)
            VALUES ('run-legacy', 'control.gazebo.response-deform', 'COMPLETED',
                    '2026-01-01T00:00:00Z', '2026-01-01T00:00:01Z')
            """
        )
        connection.execute("DROP INDEX idx_artifacts_run_content")
        connection.execute("DROP INDEX idx_artifacts_run_id")
        connection.execute("ALTER TABLE artifacts RENAME TO artifacts_new")
        connection.execute(
            """
            CREATE TABLE artifacts (
                artifact_id TEXT PRIMARY KEY, run_id TEXT NOT NULL, kind TEXT NOT NULL,
                path TEXT NOT NULL, size_bytes INTEGER, checksum TEXT, created_at TEXT NOT NULL
            )
            """
        )
        for artifact_id in ("artifact-one", "artifact-two"):
            connection.execute(
                "INSERT INTO artifacts VALUES (?, 'run-legacy', 'log', '/tmp/same.log', 3, 'sha256:old', "
                "'2026-01-01T00:00:01Z')",
                (artifact_id,),
            )
        connection.execute("DROP TABLE artifacts_new")

    with pytest.raises(sqlite3.IntegrityError, match="duplicate run paths"):
        database.initialize()

    with database.connect() as connection:
        columns = {row[1] for row in connection.execute("PRAGMA table_info(artifacts)")}
        assert connection.execute("SELECT COUNT(*) FROM artifacts").fetchone()[0] == 2
    assert "artifact_key" not in columns


def test_artifact_ids_are_content_addressed_and_duplicate_content_is_listed(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    run_dir = orchestrator.artifacts.run_dir(run.run_id)
    first = run_dir / "first.log"
    second = run_dir / "second.log"
    first.write_bytes(b"same bytes\n")
    second.write_bytes(b"same bytes\n")

    first_artifact = orchestrator.artifacts.register(run.run_id, "log", first)
    second_artifact = orchestrator.artifacts.register(run.run_id, "log", second)

    digest = hashlib.sha256(b"same bytes\n").hexdigest()
    assert first_artifact.artifact_id == f"sha256:{digest[:32]}"
    assert second_artifact.artifact_id == first_artifact.artifact_id
    assert first_artifact.checksum == f"sha256:{digest}"
    assert [item.path.name for item in orchestrator.artifacts.list(run.run_id)].count("first.log") == 1
    assert [item.path.name for item in orchestrator.artifacts.list(run.run_id)].count("second.log") == 1


def test_checksums_use_atomic_partial_and_gnu_coreutils_format(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    run_dir = orchestrator.artifacts.run_dir(run.run_id)
    payload = run_dir / "payload.bin"
    payload.write_bytes(b"payload")
    orchestrator.artifacts.register(run.run_id, "payload", payload)

    checksum_artifact = orchestrator.artifacts.write_checksums(run.run_id)

    lines = checksum_artifact.path.read_text(encoding="utf-8").splitlines()
    expected = f"{hashlib.sha256(b'payload').hexdigest()}  payload.bin"
    assert expected in lines
    assert all(not path.name.endswith(".partial") for path in run_dir.rglob("*"))


def test_registration_and_lookup_reject_symlinks_and_outside_paths(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    run_dir = orchestrator.artifacts.run_dir(run.run_id)
    outside = tmp_path / "outside.bin"
    outside.write_bytes(b"outside")
    link = run_dir / "linked.bin"
    link.symlink_to(outside)

    with pytest.raises(ValueError):
        orchestrator.artifacts.register(run.run_id, "unsafe", outside)
    with pytest.raises(ValueError):
        orchestrator.artifacts.register(run.run_id, "unsafe", link)

    safe = run_dir / "safe.bin"
    safe.write_bytes(b"safe")
    artifact = orchestrator.artifacts.register(run.run_id, "safe", safe)
    safe.unlink()
    safe.symlink_to(outside)
    with pytest.raises(ArtifactNotFound):
        orchestrator.artifacts.get(run.run_id, artifact.artifact_id)


def test_user_stop_finalizes_fixed_manifest_summary_events_logs_graph_and_checksums(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    assert orchestrator.start_run(run.run_id).state is RunState.RUNNING

    completed = orchestrator.stop_run(run.run_id)

    assert completed.state is RunState.COMPLETED
    run_dir = orchestrator.artifacts.run_dir(run.run_id)
    manifest = json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))
    summary = json.loads((run_dir / "summary.json").read_text(encoding="utf-8"))
    assert set(manifest) == MANIFEST_KEYS
    assert set(summary) == SUMMARY_KEYS
    assert manifest["final_state"] == "COMPLETED"
    assert manifest["failure_reason"] == "USER_STOP"
    assert manifest["recording"] == {"enabled": False, "profile": "default", "recorders": []}
    assert summary["final_state"] == "COMPLETED"
    assert summary["exploration_progress_final"] is None
    assert summary["response_group"] is None
    assert (run_dir / "events.jsonl").read_text(encoding="utf-8").count("\n") >= 1
    assert (run_dir / "process-logs" / "ros1_response_demo.stdout.log").is_file()
    assert (run_dir / "process-logs" / "ros1_response_demo.stderr.log").is_file()
    assert (run_dir / "ros" / "ros1-graph.json").is_file()
    assert (run_dir / "ros" / "topics-snapshot.json").is_file()
    assert (run_dir / "ros" / "nodes-snapshot.json").is_file()
    assert (run_dir / "resolved-scenario.yaml").is_file()
    assert (run_dir / "checksums.sha256").is_file()
    assert not any(path.name.endswith(".partial") for path in run_dir.rglob("*"))


def test_finalization_retry_repairs_a_checksum_file_left_unindexed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    orchestrator.start_run(run.run_id)
    original_register = orchestrator.artifacts.register
    failed_once = False

    def fail_checksum_registration(run_id, kind, path):
        nonlocal failed_once
        if Path(path).name == "checksums.sha256" and not failed_once:
            failed_once = True
            raise OSError("injected registration failure")
        return original_register(run_id, kind, path)

    monkeypatch.setattr(orchestrator.artifacts, "register", fail_checksum_registration)
    completed = orchestrator.stop_run(run.run_id)
    assert completed.state is RunState.COMPLETED
    assert any(event.kind == "ARCHIVE_FAILED" for event in orchestrator.events.list(run.run_id))

    monkeypatch.setattr(orchestrator.artifacts, "register", original_register)
    orchestrator.stop_run(run.run_id)

    assert any(item.path.name == "checksums.sha256" for item in orchestrator.artifacts.list(run.run_id))


def test_archive_completeness_requires_exact_checksum_coverage(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run("control.gazebo.response-deform", {})
    orchestrator.start_run(run.run_id)
    orchestrator.stop_run(run.run_id)
    checksums = orchestrator.artifacts.run_dir(run.run_id) / "checksums.sha256"
    complete_lines = checksums.read_text(encoding="utf-8").splitlines()
    checksums.write_text("\n".join(complete_lines[1:]) + "\n", encoding="utf-8")

    assert orchestrator._archive_is_complete(run.run_id) is False

    orchestrator.stop_run(run.run_id)
    assert len(checksums.read_text(encoding="utf-8").splitlines()) == len(complete_lines)


def test_recovery_archive_retry_keeps_unproven_bags_unindexed(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    original, _, _ = build_test_orchestrator(tmp_path, owner_id="old-api")
    run = original.create_run(
        "swarm.webots.ros2swarm",
        {},
        {"enabled": True, "profile": "default"},
    )
    bag = original.artifacts.run_dir(run.run_id) / "bags" / "ros2"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("possibly live\n", encoding="utf-8")

    restarted, agents, _ = build_test_orchestrator(tmp_path, owner_id="new-api")
    agents["ros2"].health_ok = False
    original_register = restarted.artifacts.register
    failed_once = False

    def fail_checksum_registration(run_id, kind, path):
        nonlocal failed_once
        if Path(path).name == "checksums.sha256" and not failed_once:
            failed_once = True
            raise OSError("injected recovery archive failure")
        return original_register(run_id, kind, path)

    monkeypatch.setattr(restarted.artifacts, "register", fail_checksum_registration)
    restarted.recover_active_runs()
    monkeypatch.setattr(restarted.artifacts, "register", original_register)
    restarted.finalize_incomplete_runs()

    summary = (restarted.artifacts.run_dir(run.run_id) / "summary.json").read_text(encoding="utf-8")
    assert "ownership_unproven" in summary
    assert not any(item.kind == "ros2_bag" for item in restarted.artifacts.list(run.run_id))


def test_failed_start_archive_preserves_original_reason(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path, ros1_ready=False)
    run = orchestrator.create_run("exploration.gazebo.tepper", {})

    failed = orchestrator.start_run(run.run_id)

    assert failed.reason_code is ReasonCode.READINESS_TIMEOUT
    manifest = json.loads(
        (orchestrator.artifacts.run_dir(run.run_id) / "manifest.json").read_text(encoding="utf-8")
    )
    assert manifest["final_state"] == "FAILED"
    assert manifest["failure_reason"] == "READINESS_TIMEOUT"


def test_recording_plans_are_manifest_derived_and_never_record_all_topics(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    manifest = orchestrator.catalog.get("exploration.webots.robosar")

    plans = build_recorder_plans(
        manifest,
        {"enabled": True, "profile": "default"},
        tmp_path / "run-recording",
    )

    assert [plan.agent for plan in plans] == ["ros1", "ros2"]
    assert [plan.process_id for plan in plans] == ["recorder_ros1", "recorder_ros2"]
    assert all("-a" not in plan.argv and "--all" not in plan.argv for plan in plans)
    assert plans[0].argv[:3] == ("rosbag", "record", "--regex")
    assert plans[1].argv[:3] == ("ros2", "bag", "record")
    assert plans[0].argv[4] == "@run/bags/ros1/recording"
    assert plans[1].argv[-1] == "@run/bags/ros2"
    assert all(plan.maximum_bytes_per_second == 2_000_000 for plan in plans)
    assert plans[0].command_ref == "record.ros1.exploration_webots"
    assert plans[1].command_ref == "record.ros2.exploration_webots"

    with pytest.raises(ValueError):
        build_recorder_plans(manifest, {"enabled": True, "profile": "arbitrary"}, tmp_path)
    with pytest.raises(ValueError):
        build_recorder_plans(manifest, {"enabled": True, "profile": "default", "topics": ["/tf"]}, tmp_path)


def test_every_recordable_scenario_matches_the_agent_exact_regex_contract(tmp_path: Path) -> None:
    orchestrator, _, _ = build_test_orchestrator(tmp_path)
    for manifest in orchestrator.catalog:
        if manifest.artifact_profile == "none":
            continue
        plans = build_recorder_plans(
            manifest,
            {"enabled": True, "profile": "default"},
            tmp_path / manifest.scenario_id,
        )
        for plan in plans:
            regex_index = 5 if plan.agent == "ros1" else 4
            assert plan.command_ref in RECORDER_REGEXES
            assert plan.argv[regex_index] == RECORDER_REGEXES[plan.command_ref]


def test_recorders_start_from_static_plans_and_stop_before_business_processes(tmp_path: Path) -> None:
    orchestrator, _, trace = build_test_orchestrator(tmp_path)
    run = orchestrator.create_run(
        "swarm.webots.ros2swarm",
        {},
        {"enabled": True, "profile": "default"},
    )

    assert orchestrator.start_run(run.run_id).state is RunState.RUNNING
    run_dir = orchestrator.artifacts.run_dir(run.run_id)
    bag = run_dir / "bags" / "ros2"
    bag.mkdir(parents=True)
    (bag / "metadata.yaml").write_text("rosbag2_bagfile_information: {}\n", encoding="utf-8")
    completed = orchestrator.stop_run(run.run_id)

    assert completed.state is RunState.COMPLETED
    starts = [item for item in trace if item.startswith("start:")]
    stops = [item for item in trace if item.startswith("stop:")]
    assert starts[-1] == "start:ros2:recorder_ros2"
    assert stops[0] == "stop:ros2:recorder_ros2"
    assert trace.index("graph:ros2") < trace.index("stop:ros2:swarm_simulation")
    assert any(item.path.name == "metadata.yaml" and item.kind == "ros2_bag" for item in orchestrator.artifacts.list(run.run_id))
