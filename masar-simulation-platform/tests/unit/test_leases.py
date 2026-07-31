from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timedelta, timezone
from pathlib import Path
from threading import Barrier

import pytest

from masar_platform.db import Database
from masar_platform.events import EventStore
from masar_platform.leases import (
    HEARTBEAT_INTERVAL_SECONDS,
    LEASE_EXPIRY_SECONDS,
    LeaseConflict,
    LeaseExpired,
    LeaseOwnershipError,
    LeaseStore,
)
from masar_platform.state_machine import ReasonCode, RunState, RunStateMachine


NOW = datetime(2026, 8, 1, 2, 0, 0, tzinfo=timezone.utc)


def _store(tmp_path: Path) -> tuple[Database, EventStore, LeaseStore, RunStateMachine]:
    database = Database(tmp_path / "platform.sqlite3")
    database.initialize()
    events = EventStore(database, tmp_path / "runs")
    return database, events, LeaseStore(database, events), RunStateMachine(database, events)


def test_duplicate_run_acquisition_has_exactly_one_winner(tmp_path: Path) -> None:
    database, events, leases, _ = _store(tmp_path)
    ready = Barrier(2)

    def acquire(run_id: str) -> object:
        ready.wait()
        try:
            return leases.acquire(run_id, "scenario-1", f"owner-{run_id}", at=NOW)
        except LeaseConflict as exc:
            return exc

    with ThreadPoolExecutor(max_workers=2) as pool:
        results = list(pool.map(acquire, ("run-1", "run-2")))

    winners = [result for result in results if not isinstance(result, LeaseConflict)]
    conflicts = [result for result in results if isinstance(result, LeaseConflict)]
    assert len(winners) == 1
    assert len(conflicts) == 1
    assert conflicts[0].active_run_id == winners[0].run_id
    with database.connect() as connection:
        assert connection.execute("SELECT count(*) FROM runs").fetchone()[0] == 1
        assert connection.execute("SELECT count(*) FROM leases").fetchone()[0] == 1
    assert len(events.list(winners[0].run_id)) == 1


def test_acquisition_rejects_a_second_claim_for_the_same_run(tmp_path: Path) -> None:
    _, _, leases, _ = _store(tmp_path)
    leases.acquire("run-1", "scenario-1", "owner-1", at=NOW)

    with pytest.raises(LeaseConflict) as caught:
        leases.acquire("run-1", "scenario-1", "owner-2", at=NOW)

    assert caught.value.active_run_id == "run-1"


def test_heartbeat_renews_for_ten_seconds_and_expiry_is_testable(tmp_path: Path) -> None:
    _, _, leases, _ = _store(tmp_path)
    lease = leases.acquire("run-1", "scenario-1", "owner-1", at=NOW)

    assert HEARTBEAT_INTERVAL_SECONDS == 2
    assert LEASE_EXPIRY_SECONDS == 10
    assert lease.heartbeat_at == NOW
    assert lease.expires_at == NOW + timedelta(seconds=10)

    renewed = leases.heartbeat("run-1", "owner-1", at=NOW + timedelta(seconds=2))

    assert renewed.heartbeat_at == NOW + timedelta(seconds=2)
    assert renewed.expires_at == NOW + timedelta(seconds=12)
    assert not leases.is_expired("run-1", at=NOW + timedelta(seconds=11, milliseconds=999))
    assert leases.is_expired("run-1", at=NOW + timedelta(seconds=12))
    with pytest.raises(LeaseExpired):
        leases.heartbeat("run-1", "owner-1", at=NOW + timedelta(seconds=12))


def test_heartbeat_requires_the_current_owner(tmp_path: Path) -> None:
    _, _, leases, _ = _store(tmp_path)
    leases.acquire("run-1", "scenario-1", "owner-1", at=NOW)

    with pytest.raises(LeaseOwnershipError):
        leases.heartbeat("run-1", "owner-2", at=NOW + timedelta(seconds=2))


def test_terminal_run_releases_the_single_active_run_constraint(tmp_path: Path) -> None:
    _, _, leases, states = _store(tmp_path)
    leases.acquire("run-1", "scenario-1", "owner-1", at=NOW)
    states.transition("run-1", RunState.STOPPING, reason_code=ReasonCode.USER_STOP, at=NOW)
    states.transition("run-1", RunState.COMPLETED, reason_code=ReasonCode.USER_STOP, at=NOW)
    leases.release("run-1", "owner-1")

    second = leases.acquire("run-2", "scenario-1", "owner-2", at=NOW)

    assert second.run_id == "run-2"


def test_lease_foreign_key_prevents_orphan_ownership_rows(tmp_path: Path) -> None:
    database, _, _, _ = _store(tmp_path)

    with database.connect() as connection, pytest.raises(Exception):
        connection.execute(
            """
            INSERT INTO leases (run_id, owner_id, acquired_at, heartbeat_at, expires_at)
            VALUES (?, ?, ?, ?, ?)
            """,
            ("missing-run", "owner-1", "2026-08-01T02:00:00Z", "2026-08-01T02:00:00Z", "2026-08-01T02:00:10Z"),
        )
