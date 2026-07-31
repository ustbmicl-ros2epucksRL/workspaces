"""Durable per-run event allocation, JSONL mirroring, and live retention."""

from __future__ import annotations

import json
import os
import sqlite3
import threading
import uuid
from collections.abc import Mapping
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

from .db import Database, datetime_from_text, datetime_to_text, utc_datetime


_ARCHIVE_WRITE_LOCK = threading.Lock()


class EventValidationError(ValueError):
    """Raised when an event does not satisfy the persisted event contract."""


@dataclass(frozen=True, slots=True)
class Event:
    schema_version: int
    event_id: str
    run_id: str
    sequence: int
    wall_time: datetime
    sim_time_ns: int | None
    source: str
    kind: str
    severity: str
    payload: dict[str, Any]

    def as_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "event_id": self.event_id,
            "run_id": self.run_id,
            "sequence": self.sequence,
            "wall_time": datetime_to_text(self.wall_time, field="wall_time"),
            "sim_time_ns": self.sim_time_ns,
            "source": self.source,
            "kind": self.kind,
            "severity": self.severity,
            "payload": self.payload,
        }


class EventStore:
    """Allocate events transactionally and retain an append-only run archive."""

    def __init__(
        self,
        database: Database,
        archive_root: str | Path,
        *,
        retention_days: int = 30,
        retention_count: int = 10_000,
    ) -> None:
        if retention_days <= 0 or retention_count <= 0:
            raise ValueError("event retention values must be greater than zero")
        self.database = database
        self.archive_root = Path(archive_root)
        self.retention_days = retention_days
        self.retention_count = retention_count

    def append(
        self,
        run_id: str,
        *,
        source: str,
        kind: str,
        severity: str,
        payload: Mapping[str, Any],
        wall_time: datetime,
        sim_time_ns: int | None = None,
        event_id: str | None = None,
    ) -> Event:
        with self.database.transaction(immediate=True) as connection:
            event = self.append_in_transaction(
                connection,
                run_id,
                source=source,
                kind=kind,
                severity=severity,
                payload=payload,
                wall_time=wall_time,
                sim_time_ns=sim_time_ns,
                event_id=event_id,
            )
        self.mirror(event)
        return event

    def append_in_transaction(
        self,
        connection: sqlite3.Connection,
        run_id: str,
        *,
        source: str,
        kind: str,
        severity: str,
        payload: Mapping[str, Any],
        wall_time: datetime,
        sim_time_ns: int | None = None,
        event_id: str | None = None,
    ) -> Event:
        normalized_payload, encoded_payload = _encode_payload(payload)
        normalized_time = utc_datetime(wall_time, field="wall_time")
        if sim_time_ns is not None and (not isinstance(sim_time_ns, int) or isinstance(sim_time_ns, bool)):
            raise EventValidationError("sim_time_ns must be an integer or null")
        for field_name, value in (("run_id", run_id), ("source", source), ("kind", kind), ("severity", severity)):
            if not isinstance(value, str) or not value:
                raise EventValidationError(f"{field_name} must be a non-empty string")

        run_row = connection.execute(
            "SELECT next_event_sequence FROM runs WHERE run_id = ?",
            (run_id,),
        ).fetchone()
        if run_row is None:
            raise EventValidationError(f"unknown run_id: {run_id}")
        sequence = run_row[0]
        connection.execute(
            "UPDATE runs SET next_event_sequence = next_event_sequence + 1 WHERE run_id = ?",
            (run_id,),
        )
        stable_event_id = event_id or str(uuid.uuid4())
        connection.execute(
            """
            INSERT INTO events (
                schema_version, event_id, run_id, sequence, wall_time, sim_time_ns, source, kind, severity, payload
            ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """,
            (
                1,
                stable_event_id,
                run_id,
                sequence,
                datetime_to_text(normalized_time, field="wall_time"),
                sim_time_ns,
                source,
                kind,
                severity,
                encoded_payload,
            ),
        )
        return Event(
            schema_version=1,
            event_id=stable_event_id,
            run_id=run_id,
            sequence=sequence,
            wall_time=normalized_time,
            sim_time_ns=sim_time_ns,
            source=source,
            kind=kind,
            severity=severity,
            payload=normalized_payload,
        )

    def mirror(self, event: Event) -> None:
        """Mirror committed events and repair any preceding committed gap."""

        path = self.archive_path(event.run_id)
        with _ARCHIVE_WRITE_LOCK:
            path.parent.mkdir(parents=True, exist_ok=True)
            last_sequence = _last_archived_sequence(path)
            if event.sequence <= last_sequence:
                return
            with self.database.connect() as connection:
                rows = connection.execute(
                    """
                    SELECT * FROM events
                    WHERE run_id = ? AND sequence > ? AND sequence <= ?
                    ORDER BY sequence
                    """,
                    (event.run_id, last_sequence, event.sequence),
                ).fetchall()
            with path.open("a", encoding="utf-8") as stream:
                for row in rows:
                    stream.write(json.dumps(_event_from_row(row).as_dict(), separators=(",", ":"), sort_keys=True))
                    stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())

    def archive_path(self, run_id: str) -> Path:
        """Return the handoff path consumed by later run finalization."""

        return self.archive_root / run_id / "events.jsonl"

    def prepare_archive(self, run_id: str) -> Path:
        """Flush the append-only JSONL boundary before checksums are generated."""

        path = self.archive_path(run_id)
        with _ARCHIVE_WRITE_LOCK:
            path.parent.mkdir(parents=True, exist_ok=True)
            last_sequence = _last_archived_sequence(path)
            with self.database.connect() as connection:
                rows = connection.execute(
                    """
                    SELECT * FROM events
                    WHERE run_id = ? AND sequence > ?
                    ORDER BY sequence
                    """,
                    (run_id, last_sequence),
                ).fetchall()
            with path.open("a", encoding="utf-8") as stream:
                for row in rows:
                    stream.write(json.dumps(_event_from_row(row).as_dict(), separators=(",", ":"), sort_keys=True))
                    stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
        return path

    def list(self, run_id: str, *, after_sequence: int = 0) -> list[Event]:
        with self.database.connect() as connection:
            rows = connection.execute(
                "SELECT * FROM events WHERE run_id = ? AND sequence > ? ORDER BY sequence",
                (run_id, after_sequence),
            ).fetchall()
        return [_event_from_row(row) for row in rows]

    def prune(self, *, at: datetime | None = None) -> int:
        """Prune live rows by age or count without opening archive files."""

        current_time = utc_datetime(at or datetime.now(timezone.utc), field="at")
        cutoff = datetime_to_text(current_time - timedelta(days=self.retention_days))
        deleted = 0
        with self.database.transaction(immediate=True) as connection:
            cursor = connection.execute("DELETE FROM events WHERE wall_time < ?", (cutoff,))
            deleted += cursor.rowcount
            run_ids = connection.execute("SELECT DISTINCT run_id FROM events").fetchall()
            for row in run_ids:
                boundary = connection.execute(
                    """
                    SELECT sequence FROM events
                    WHERE run_id = ?
                    ORDER BY sequence DESC
                    LIMIT 1 OFFSET ?
                    """,
                    (row["run_id"], self.retention_count - 1),
                ).fetchone()
                if boundary is None:
                    continue
                cursor = connection.execute(
                    "DELETE FROM events WHERE run_id = ? AND sequence < ?",
                    (row["run_id"], boundary["sequence"]),
                )
                deleted += cursor.rowcount
        return deleted


def _encode_payload(payload: Mapping[str, Any]) -> tuple[dict[str, Any], str]:
    if not isinstance(payload, Mapping):
        raise EventValidationError("payload must be a JSON object")
    normalized = dict(payload)
    try:
        encoded = json.dumps(normalized, separators=(",", ":"), sort_keys=True)
    except (TypeError, ValueError) as exc:
        raise EventValidationError("payload must be a JSON object") from exc
    return json.loads(encoded), encoded


def _event_from_row(row: sqlite3.Row) -> Event:
    return Event(
        schema_version=row["schema_version"],
        event_id=row["event_id"],
        run_id=row["run_id"],
        sequence=row["sequence"],
        wall_time=datetime_from_text(row["wall_time"]),
        sim_time_ns=row["sim_time_ns"],
        source=row["source"],
        kind=row["kind"],
        severity=row["severity"],
        payload=json.loads(row["payload"]),
    )


def _last_archived_sequence(path: Path) -> int:
    if not path.exists():
        return 0
    last_line = ""
    with path.open("r", encoding="utf-8") as stream:
        for line in stream:
            if line.strip():
                last_line = line
    return int(json.loads(last_line)["sequence"]) if last_line else 0


__all__ = ["Event", "EventStore", "EventValidationError"]
