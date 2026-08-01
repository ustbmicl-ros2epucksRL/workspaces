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
            _migrate_runs_contract(connection)

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
    request_json TEXT NOT NULL DEFAULT '{}' CHECK (json_valid(request_json) AND json_type(request_json) = 'object'),
    resolved_manifest_json TEXT NOT NULL DEFAULT '{}' CHECK (
        json_valid(resolved_manifest_json) AND json_type(resolved_manifest_json) = 'object'
    ),
    archive_conditions_json TEXT NOT NULL DEFAULT '[]' CHECK (
        json_valid(archive_conditions_json) AND json_type(archive_conditions_json) = 'array'
    ),
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
    artifact_key INTEGER PRIMARY KEY AUTOINCREMENT,
    artifact_id TEXT NOT NULL,
    run_id TEXT NOT NULL REFERENCES runs(run_id) ON DELETE CASCADE,
    kind TEXT NOT NULL,
    path TEXT NOT NULL,
    size_bytes INTEGER,
    checksum TEXT,
    created_at TEXT NOT NULL,
    UNIQUE (run_id, path)
);

CREATE INDEX IF NOT EXISTS idx_runs_run_id ON runs(run_id);
CREATE INDEX IF NOT EXISTS idx_run_steps_run_id ON run_steps(run_id);
CREATE INDEX IF NOT EXISTS idx_leases_run_id ON leases(run_id);
CREATE INDEX IF NOT EXISTS idx_events_run_id ON events(run_id);
CREATE UNIQUE INDEX IF NOT EXISTS idx_events_run_sequence ON events(run_id, sequence);
CREATE INDEX IF NOT EXISTS idx_artifacts_run_id ON artifacts(run_id);
CREATE INDEX IF NOT EXISTS idx_artifacts_run_content ON artifacts(run_id, artifact_id);
CREATE UNIQUE INDEX IF NOT EXISTS idx_runs_single_active
    ON runs ((1))
    WHERE state IN ('PREFLIGHT', 'STARTING', 'RUNNING', 'STOPPING');
"""


def _migrate_runs_contract(connection: sqlite3.Connection) -> None:
    """Add pre-release Task 4 columns to databases created by earlier tasks."""

    columns = {row[1] for row in connection.execute("PRAGMA table_info(runs)")}
    if "request_json" not in columns:
        connection.execute(
            "ALTER TABLE runs ADD COLUMN request_json TEXT NOT NULL DEFAULT '{}' "
            "CHECK (json_valid(request_json) AND json_type(request_json) = 'object')"
        )
    if "resolved_manifest_json" not in columns:
        connection.execute(
            "ALTER TABLE runs ADD COLUMN resolved_manifest_json TEXT NOT NULL DEFAULT '{}' "
            "CHECK (json_valid(resolved_manifest_json) AND json_type(resolved_manifest_json) = 'object')"
        )
    if "archive_conditions_json" not in columns:
        connection.execute(
            "ALTER TABLE runs ADD COLUMN archive_conditions_json TEXT NOT NULL DEFAULT '[]' "
            "CHECK (json_valid(archive_conditions_json) AND json_type(archive_conditions_json) = 'array')"
        )

    _migrate_artifacts_contract(connection)


def _migrate_artifacts_contract(connection: sqlite3.Connection) -> None:
    tables = {
        row[0]
        for row in connection.execute(
            "SELECT name FROM sqlite_master WHERE type = 'table' AND name IN ('artifacts', 'artifacts_legacy')"
        )
    }
    artifact_columns = {row[1] for row in connection.execute("PRAGMA table_info(artifacts)")}
    legacy_exists = "artifacts_legacy" in tables
    legacy_schema = bool(artifact_columns) and "artifact_key" not in artifact_columns
    if not legacy_exists and not legacy_schema:
        return
    if legacy_exists and legacy_schema:
        raise sqlite3.IntegrityError("both artifacts and artifacts_legacy use the legacy schema")

    connection.execute("BEGIN IMMEDIATE")
    try:
        if legacy_schema:
            connection.execute("DROP INDEX IF EXISTS idx_artifacts_run_id")
            connection.execute("DROP INDEX IF EXISTS idx_artifacts_run_content")
            connection.execute("ALTER TABLE artifacts RENAME TO artifacts_legacy")
            _create_artifacts_table(connection)
        duplicates = connection.execute(
            """
            SELECT run_id, path FROM artifacts_legacy
            GROUP BY run_id, path HAVING COUNT(*) > 1 LIMIT 1
            """
        ).fetchone()
        if duplicates is not None:
            raise sqlite3.IntegrityError("legacy artifacts contain duplicate run paths")
        connection.execute(
            """
            INSERT INTO artifacts (artifact_id, run_id, kind, path, size_bytes, checksum, created_at)
            SELECT artifact_id, run_id, kind, path, size_bytes, checksum, created_at
            FROM artifacts_legacy WHERE true
            ON CONFLICT(run_id, path) DO UPDATE SET
                artifact_id = excluded.artifact_id,
                kind = excluded.kind,
                size_bytes = excluded.size_bytes,
                checksum = excluded.checksum,
                created_at = excluded.created_at
            """
        )
        missing = connection.execute(
            """
            SELECT COUNT(*) FROM artifacts_legacy AS legacy
            LEFT JOIN artifacts AS current
              ON current.run_id = legacy.run_id AND current.path = legacy.path
            WHERE current.artifact_key IS NULL
            """
        ).fetchone()[0]
        if missing:
            raise sqlite3.IntegrityError("legacy artifact migration lost rows")
        foreign_key_errors = connection.execute("PRAGMA foreign_key_check(artifacts)").fetchall()
        if foreign_key_errors:
            raise sqlite3.IntegrityError("legacy artifacts violate run foreign keys")
        connection.execute("DROP TABLE artifacts_legacy")
        connection.execute("CREATE INDEX IF NOT EXISTS idx_artifacts_run_id ON artifacts(run_id)")
        connection.execute(
            "CREATE INDEX IF NOT EXISTS idx_artifacts_run_content ON artifacts(run_id, artifact_id)"
        )
        connection.commit()
    except BaseException:
        connection.rollback()
        raise


def _create_artifacts_table(connection: sqlite3.Connection) -> None:
    connection.execute(
        """
        CREATE TABLE artifacts (
            artifact_key INTEGER PRIMARY KEY AUTOINCREMENT,
            artifact_id TEXT NOT NULL,
            run_id TEXT NOT NULL REFERENCES runs(run_id) ON DELETE CASCADE,
            kind TEXT NOT NULL,
            path TEXT NOT NULL,
            size_bytes INTEGER,
            checksum TEXT,
            created_at TEXT NOT NULL,
            UNIQUE (run_id, path)
        )
        """
    )


__all__ = [
    "Database",
    "datetime_from_text",
    "datetime_to_text",
    "utc_datetime",
]
