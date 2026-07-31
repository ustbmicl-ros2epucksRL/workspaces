from __future__ import annotations

import json
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
from pathlib import Path
from threading import Barrier

import pytest

from masar_platform.db import Database
from masar_platform.events import EventStore, EventValidationError
from masar_platform.leases import LeaseStore


NOW = datetime(2026, 8, 1, 3, 4, 5, 678901, tzinfo=timezone.utc)


def _store(
    tmp_path: Path,
    *,
    retention_days: int = 30,
    retention_count: int = 10_000,
) -> tuple[Database, EventStore, LeaseStore]:
    database = Database(tmp_path / "platform.sqlite3")
    database.initialize()
    events = EventStore(
        database,
        tmp_path / "runs",
        retention_days=retention_days,
        retention_count=retention_count,
    )
    leases = LeaseStore(database, events)
    return database, events, leases


def test_database_initializes_five_indexed_wal_tables(tmp_path: Path) -> None:
    database, _, _ = _store(tmp_path)

    with database.connect() as connection:
        assert connection.execute("PRAGMA journal_mode").fetchone()[0].lower() == "wal"
        assert connection.execute("PRAGMA foreign_keys").fetchone()[0] == 1
        tables = {
            row[0]
            for row in connection.execute(
                "SELECT name FROM sqlite_master WHERE type = 'table' AND name NOT LIKE 'sqlite_%'"
            )
        }
        assert tables == {"runs", "run_steps", "leases", "events", "artifacts"}
        for table in tables:
            columns = connection.execute(f"PRAGMA table_info({table})").fetchall()
            assert any(column[5] for column in columns), f"{table} has no primary key"
            indexes = connection.execute(f"PRAGMA index_list({table})").fetchall()
            indexed_columns = {
                tuple(
                    column[2]
                    for column in connection.execute(f"PRAGMA index_info({index[1]})").fetchall()
                )
                for index in indexes
            }
            assert any(columns and columns[0] == "run_id" for columns in indexed_columns), table
        event_indexes = connection.execute("PRAGMA index_list(events)").fetchall()
        unique_event_indexes = {
            tuple(
                column[2]
                for column in connection.execute(f"PRAGMA index_info({index[1]})").fetchall()
            )
            for index in event_indexes
            if index[2]
        }
        assert ("run_id", "sequence") in unique_event_indexes


def test_event_contract_and_jsonl_mirror(tmp_path: Path) -> None:
    _, events, leases = _store(tmp_path)
    leases.acquire("run-1", "scenario-1", "owner-1", at=NOW)

    event = events.append(
        "run-1",
        source="ros1-agent",
        kind="TOPIC_STALE",
        severity="warning",
        payload={"topic": "/robot_5/odom", "age_ms": 800},
        wall_time=NOW + timedelta(seconds=1),
        sim_time_ns=950_000_000,
    )

    assert event.schema_version == 1
    assert event.event_id
    assert event.run_id == "run-1"
    assert event.sequence == 2
    assert event.wall_time == NOW + timedelta(seconds=1)
    assert event.sim_time_ns == 950_000_000
    assert event.source == "ros1-agent"
    assert event.kind == "TOPIC_STALE"
    assert event.severity == "warning"
    assert event.payload == {"topic": "/robot_5/odom", "age_ms": 800}

    line = events.archive_path("run-1").read_text(encoding="utf-8").splitlines()[-1]
    assert json.loads(line) == event.as_dict()
    assert json.loads(line)["wall_time"] == "2026-08-01T03:04:06.678901Z"


def test_event_payload_must_be_a_json_object(tmp_path: Path) -> None:
    _, events, leases = _store(tmp_path)
    leases.acquire("run-1", "scenario-1", "owner-1", at=NOW)

    with pytest.raises(EventValidationError, match="payload"):
        events.append(
            "run-1",
            source="api",
            kind="INVALID",
            severity="error",
            payload=["not", "an", "object"],  # type: ignore[arg-type]
            wall_time=NOW,
        )


def test_concurrent_connections_allocate_strictly_increasing_sequences(tmp_path: Path) -> None:
    _, events, leases = _store(tmp_path)
    leases.acquire("run-1", "scenario-1", "owner-1", at=NOW)
    ready = Barrier(8)

    def append(index: int) -> int:
        ready.wait()
        return events.append(
            "run-1",
            source="test",
            kind="CONCURRENT",
            severity="info",
            payload={"index": index},
            wall_time=NOW + timedelta(microseconds=index + 1),
        ).sequence

    with ThreadPoolExecutor(max_workers=8) as pool:
        allocated = list(pool.map(append, range(8)))

    assert sorted(allocated) == list(range(2, 10))
    persisted = events.list("run-1")
    assert [event.sequence for event in persisted] == list(range(1, 10))
    assert len({event.event_id for event in persisted}) == len(persisted)
    mirrored = [
        json.loads(line)
        for line in events.archive_path("run-1").read_text(encoding="utf-8").splitlines()
    ]
    assert sorted(event["sequence"] for event in mirrored) == list(range(1, 10))


def test_failed_jsonl_mirror_happens_after_database_commit(tmp_path: Path) -> None:
    database, events, leases = _store(tmp_path)
    leases.acquire("run-1", "scenario-1", "owner-1", at=NOW)
    archive = events.archive_path("run-1")
    archive.unlink()
    archive.mkdir()

    with pytest.raises(IsADirectoryError):
        events.append(
            "run-1",
            source="api",
            kind="COMMITTED",
            severity="info",
            payload={},
            wall_time=NOW + timedelta(seconds=1),
        )

    with database.connect() as connection:
        row = connection.execute(
            "SELECT sequence, kind FROM events WHERE run_id = ? ORDER BY sequence DESC LIMIT 1",
            ("run-1",),
        ).fetchone()
    assert (row["sequence"], row["kind"]) == (2, "COMMITTED")


def test_live_retention_uses_age_or_count_without_touching_archive(tmp_path: Path) -> None:
    _, events, leases = _store(tmp_path, retention_days=30, retention_count=3)
    leases.acquire("run-1", "scenario-1", "owner-1", at=NOW - timedelta(days=40))
    for index, age_days in enumerate((35, 20, 10, 1), start=1):
        events.append(
            "run-1",
            source="test",
            kind="SAMPLE",
            severity="info",
            payload={"index": index},
            wall_time=NOW - timedelta(days=age_days),
        )
    archive = events.archive_path("run-1")
    archive_before = archive.read_bytes()

    deleted = events.prune(at=NOW)

    assert deleted == 2
    assert [event.sequence for event in events.list("run-1")] == [3, 4, 5]
    assert archive.read_bytes() == archive_before
    assert events.archive_path("run-1") == tmp_path / "runs" / "run-1" / "events.jsonl"


def test_list_supports_reconnect_cursor(tmp_path: Path) -> None:
    _, events, leases = _store(tmp_path)
    leases.acquire("run-1", "scenario-1", "owner-1", at=NOW)
    for index in range(3):
        events.append(
            "run-1",
            source="test",
            kind="CURSOR",
            severity="info",
            payload={"index": index},
            wall_time=NOW + timedelta(seconds=index + 1),
        )

    assert [event.sequence for event in events.list("run-1", after_sequence=2)] == [3, 4]
