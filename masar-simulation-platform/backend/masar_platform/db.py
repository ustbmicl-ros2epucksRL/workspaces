"""SQLite schema and transaction primitives for platform run state."""

from __future__ import annotations

import sqlite3
from collections.abc import Iterator
from contextlib import contextmanager
from datetime import datetime, timezone
from pathlib import Path


UTC = timezone.utc


class Database:
    """Own SQLite connections without retaining thread-bound connection state."""

    def __init__(self, path: str | Path, *, timeout_seconds: float = 30.0) -> None:
        self.path = Path(path)
        self.timeout_seconds = timeout_seconds

    def connect(self) -> sqlite3.Connection:
        connection = sqlite3.connect(
            self.path,
            timeout=self.timeout_seconds,
            isolation_level=None,
        )
        connection.row_factory = sqlite3.Row
        connection.execute("PRAGMA foreign_keys = ON")
        connection.execute("PRAGMA busy_timeout = 30000")
        connection.execute("PRAGMA journal_mode = WAL")
        return connection

    def initialize(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with self.connect() as connection:
            connection.executescript(_SCHEMA)

    @contextmanager
    def transaction(self, *, immediate: bool = False) -> Iterator[sqlite3.Connection]:
        connection = self.connect()
        try:
            connection.execute("BEGIN IMMEDIATE" if immediate else "BEGIN")
            yield connection
            connection.commit()
        except BaseException:
            connection.rollback()
            raise
        finally:
            connection.close()


def utc_datetime(value: datetime, *, field: str = "time") -> datetime:
    """Normalize an aware datetime to UTC while keeping time injectable in tests."""

    if value.tzinfo is None or value.utcoffset() is None:
        raise ValueError(f"{field} must be timezone-aware")
    return value.astimezone(UTC)


def datetime_to_text(value: datetime, *, field: str = "time") -> str:
    return utc_datetime(value, field=field).isoformat().replace("+00:00", "Z")


def datetime_from_text(value: str) -> datetime:
    return datetime.fromisoformat(value.replace("Z", "+00:00")).astimezone(UTC)


_SCHEMA = """
CREATE TABLE IF NOT EXISTS runs (
    run_id TEXT PRIMARY KEY,
    scenario_id TEXT NOT NULL,
    state TEXT NOT NULL CHECK (
        state IN ('PREFLIGHT', 'STARTING', 'RUNNING', 'STOPPING', 'COMPLETED', 'FAILED')
    ),
    reason_code TEXT,
    created_at TEXT NOT NULL,
    updated_at TEXT NOT NULL,
    ended_at TEXT,
    next_event_sequence INTEGER NOT NULL DEFAULT 1 CHECK (next_event_sequence > 0)
);

CREATE TABLE IF NOT EXISTS run_steps (
    run_step_id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id TEXT NOT NULL REFERENCES runs(run_id) ON DELETE CASCADE,
    step_id TEXT NOT NULL,
    state TEXT NOT NULL,
    started_at TEXT,
    ended_at TEXT,
    detail TEXT,
    UNIQUE (run_id, step_id)
);

CREATE TABLE IF NOT EXISTS leases (
    lease_id INTEGER PRIMARY KEY AUTOINCREMENT,
    run_id TEXT NOT NULL UNIQUE REFERENCES runs(run_id) ON DELETE CASCADE,
    owner_id TEXT NOT NULL,
    acquired_at TEXT NOT NULL,
    heartbeat_at TEXT NOT NULL,
    expires_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS events (
    schema_version INTEGER NOT NULL CHECK (schema_version = 1),
    event_id TEXT PRIMARY KEY,
    run_id TEXT NOT NULL REFERENCES runs(run_id) ON DELETE CASCADE,
    sequence INTEGER NOT NULL CHECK (sequence > 0),
    wall_time TEXT NOT NULL,
    sim_time_ns INTEGER,
    source TEXT NOT NULL,
    kind TEXT NOT NULL,
    severity TEXT NOT NULL,
    payload TEXT NOT NULL CHECK (json_valid(payload) AND json_type(payload) = 'object'),
    UNIQUE (run_id, sequence)
);

CREATE TABLE IF NOT EXISTS artifacts (
    artifact_id TEXT PRIMARY KEY,
    run_id TEXT NOT NULL REFERENCES runs(run_id) ON DELETE CASCADE,
    kind TEXT NOT NULL,
    path TEXT NOT NULL,
    size_bytes INTEGER,
    checksum TEXT,
    created_at TEXT NOT NULL
);

CREATE INDEX IF NOT EXISTS idx_runs_run_id ON runs(run_id);
CREATE INDEX IF NOT EXISTS idx_run_steps_run_id ON run_steps(run_id);
CREATE INDEX IF NOT EXISTS idx_leases_run_id ON leases(run_id);
CREATE INDEX IF NOT EXISTS idx_events_run_id ON events(run_id);
CREATE UNIQUE INDEX IF NOT EXISTS idx_events_run_sequence ON events(run_id, sequence);
CREATE INDEX IF NOT EXISTS idx_artifacts_run_id ON artifacts(run_id);
CREATE UNIQUE INDEX IF NOT EXISTS idx_runs_single_active
    ON runs ((1))
    WHERE state IN ('PREFLIGHT', 'STARTING', 'RUNNING', 'STOPPING');
"""


__all__ = [
    "Database",
    "datetime_from_text",
    "datetime_to_text",
    "utc_datetime",
]
