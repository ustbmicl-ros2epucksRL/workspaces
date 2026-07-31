"""Single-active-run lease acquisition and heartbeat persistence."""

from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone

from .db import Database, datetime_from_text, datetime_to_text, utc_datetime
from .events import EventStore
from .state_machine import ACTIVE_STATES, RunState


HEARTBEAT_INTERVAL_SECONDS = 2
LEASE_EXPIRY_SECONDS = 10


class LeaseError(RuntimeError):
    """Base class for typed lease failures."""


class LeaseConflict(LeaseError):
    def __init__(self, active_run_id: str) -> None:
        self.active_run_id = active_run_id
        super().__init__(f"an active run already exists: {active_run_id}")


class LeaseNotFound(LeaseError):
    pass


class LeaseOwnershipError(LeaseError):
    pass


class LeaseExpired(LeaseError):
    pass


@dataclass(frozen=True, slots=True)
class Lease:
    lease_id: int
    run_id: str
    owner_id: str
    acquired_at: datetime
    heartbeat_at: datetime
    expires_at: datetime


class LeaseStore:
    def __init__(self, database: Database, events: EventStore) -> None:
        self.database = database
        self.events = events

    def acquire(
        self,
        run_id: str,
        scenario_id: str,
        owner_id: str,
        *,
        at: datetime | None = None,
    ) -> Lease:
        """Atomically create a PREFLIGHT run and acquire the sole active lease."""

        acquired_at = utc_datetime(at or datetime.now(timezone.utc), field="at")
        _require_identifier("run_id", run_id)
        _require_identifier("scenario_id", scenario_id)
        _require_identifier("owner_id", owner_id)
        expires_at = acquired_at + timedelta(seconds=LEASE_EXPIRY_SECONDS)
        active_values = tuple(state.value for state in ACTIVE_STATES)
        placeholders = ", ".join("?" for _ in active_values)

        with self.database.transaction(immediate=True) as connection:
            active = connection.execute(
                f"SELECT run_id FROM runs WHERE state IN ({placeholders}) LIMIT 1",  # noqa: S608
                active_values,
            ).fetchone()
            if active is not None:
                raise LeaseConflict(active["run_id"])
            existing = connection.execute("SELECT run_id FROM runs WHERE run_id = ?", (run_id,)).fetchone()
            if existing is not None:
                raise LeaseConflict(run_id)

            timestamp = datetime_to_text(acquired_at)
            connection.execute(
                """
                INSERT INTO runs (run_id, scenario_id, state, reason_code, created_at, updated_at, ended_at)
                VALUES (?, ?, ?, NULL, ?, ?, NULL)
                """,
                (run_id, scenario_id, RunState.PREFLIGHT.value, timestamp, timestamp),
            )
            cursor = connection.execute(
                """
                INSERT INTO leases (run_id, owner_id, acquired_at, heartbeat_at, expires_at)
                VALUES (?, ?, ?, ?, ?)
                """,
                (run_id, owner_id, timestamp, timestamp, datetime_to_text(expires_at)),
            )
            event = self.events.append_in_transaction(
                connection,
                run_id,
                source="control-plane",
                kind="RUN_STATE_CHANGED",
                severity="info",
                payload={"previous_state": None, "state": RunState.PREFLIGHT.value, "reason_code": None},
                wall_time=acquired_at,
            )
            lease = Lease(
                lease_id=cursor.lastrowid,
                run_id=run_id,
                owner_id=owner_id,
                acquired_at=acquired_at,
                heartbeat_at=acquired_at,
                expires_at=expires_at,
            )
        self.events.mirror(event)
        return lease

    def get(self, run_id: str) -> Lease:
        with self.database.connect() as connection:
            row = connection.execute("SELECT * FROM leases WHERE run_id = ?", (run_id,)).fetchone()
        if row is None:
            raise LeaseNotFound(run_id)
        return _lease_from_row(row)

    def heartbeat(
        self,
        run_id: str,
        owner_id: str,
        *,
        at: datetime | None = None,
    ) -> Lease:
        heartbeat_at = utc_datetime(at or datetime.now(timezone.utc), field="at")
        with self.database.transaction(immediate=True) as connection:
            row = connection.execute("SELECT * FROM leases WHERE run_id = ?", (run_id,)).fetchone()
            if row is None:
                raise LeaseNotFound(run_id)
            lease = _lease_from_row(row)
            if lease.owner_id != owner_id:
                raise LeaseOwnershipError(f"lease for {run_id} belongs to another owner")
            if heartbeat_at >= lease.expires_at:
                raise LeaseExpired(f"lease for {run_id} expired at {datetime_to_text(lease.expires_at)}")
            expires_at = heartbeat_at + timedelta(seconds=LEASE_EXPIRY_SECONDS)
            connection.execute(
                "UPDATE leases SET heartbeat_at = ?, expires_at = ? WHERE run_id = ?",
                (datetime_to_text(heartbeat_at), datetime_to_text(expires_at), run_id),
            )
        return Lease(
            lease_id=lease.lease_id,
            run_id=lease.run_id,
            owner_id=lease.owner_id,
            acquired_at=lease.acquired_at,
            heartbeat_at=heartbeat_at,
            expires_at=expires_at,
        )

    def is_expired(self, run_id: str, *, at: datetime | None = None) -> bool:
        checked_at = utc_datetime(at or datetime.now(timezone.utc), field="at")
        return checked_at >= self.get(run_id).expires_at

    def release(self, run_id: str, owner_id: str) -> None:
        with self.database.transaction(immediate=True) as connection:
            row = connection.execute("SELECT owner_id FROM leases WHERE run_id = ?", (run_id,)).fetchone()
            if row is None:
                raise LeaseNotFound(run_id)
            if row["owner_id"] != owner_id:
                raise LeaseOwnershipError(f"lease for {run_id} belongs to another owner")
            connection.execute("DELETE FROM leases WHERE run_id = ?", (run_id,))


def _lease_from_row(row: sqlite3.Row) -> Lease:
    return Lease(
        lease_id=row["lease_id"],
        run_id=row["run_id"],
        owner_id=row["owner_id"],
        acquired_at=datetime_from_text(row["acquired_at"]),
        heartbeat_at=datetime_from_text(row["heartbeat_at"]),
        expires_at=datetime_from_text(row["expires_at"]),
    )


def _require_identifier(name: str, value: str) -> None:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{name} must be a non-empty string")


__all__ = [
    "HEARTBEAT_INTERVAL_SECONDS",
    "LEASE_EXPIRY_SECONDS",
    "Lease",
    "LeaseConflict",
    "LeaseError",
    "LeaseExpired",
    "LeaseNotFound",
    "LeaseOwnershipError",
    "LeaseStore",
]
