"""Frozen public run states and transactional transition persistence."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
from enum import Enum
import json
import sqlite3
from typing import Any

from .db import Database, datetime_from_text, datetime_to_text, utc_datetime
from .events import EventStore


class RunState(str, Enum):
    PREFLIGHT = "PREFLIGHT"
    STARTING = "STARTING"
    RUNNING = "RUNNING"
    STOPPING = "STOPPING"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


class ReasonCode(str, Enum):
    USER_STOP = "USER_STOP"
    NORMAL_EXIT = "NORMAL_EXIT"
    UNEXPECTED_PROCESS_EXIT = "UNEXPECTED_PROCESS_EXIT"
    READINESS_TIMEOUT = "READINESS_TIMEOUT"
    PREFLIGHT_FAILED = "PREFLIGHT_FAILED"
    NEEDS_INSPECTION = "NEEDS_INSPECTION"


ACTIVE_STATES = frozenset(
    {RunState.PREFLIGHT, RunState.STARTING, RunState.RUNNING, RunState.STOPPING}
)
TERMINAL_STATES = frozenset({RunState.COMPLETED, RunState.FAILED})
ALLOWED_TRANSITIONS: dict[RunState, frozenset[RunState]] = {
    RunState.PREFLIGHT: frozenset({RunState.STARTING, RunState.STOPPING, RunState.FAILED}),
    RunState.STARTING: frozenset({RunState.RUNNING, RunState.STOPPING, RunState.FAILED}),
    RunState.RUNNING: frozenset({RunState.STOPPING, RunState.FAILED}),
    RunState.STOPPING: frozenset({RunState.COMPLETED, RunState.FAILED}),
    RunState.COMPLETED: frozenset(),
    RunState.FAILED: frozenset(),
}


class InvalidTransition(ValueError):
    """Raised when a requested state edge is outside the frozen graph."""

    def __init__(self, run_id: str, current: RunState, target: RunState) -> None:
        self.run_id = run_id
        self.current = current
        self.target = target
        super().__init__(f"invalid run transition for {run_id}: {current.value} -> {target.value}")


class InvalidReasonCode(ValueError):
    """Raised when a terminal state and reason code disagree."""


class RunNotFound(LookupError):
    """Raised when no persisted run has the requested ID."""


@dataclass(frozen=True, slots=True)
class RunRecord:
    run_id: str
    scenario_id: str
    state: RunState
    reason_code: ReasonCode | None
    created_at: datetime
    updated_at: datetime
    ended_at: datetime | None
    request: dict[str, Any]
    resolved_manifest: dict[str, Any]


class RunStateMachine:
    def __init__(self, database: Database, events: EventStore) -> None:
        self.database = database
        self.events = events

    def get(self, run_id: str) -> RunRecord:
        with self.database.connect() as connection:
            row = connection.execute("SELECT * FROM runs WHERE run_id = ?", (run_id,)).fetchone()
        if row is None:
            raise RunNotFound(run_id)
        return _run_from_row(row)

    def current(self) -> RunRecord | None:
        active = tuple(state.value for state in ACTIVE_STATES)
        placeholders = ",".join("?" for _ in active)
        with self.database.connect() as connection:
            row = connection.execute(
                f"SELECT * FROM runs WHERE state IN ({placeholders}) ORDER BY created_at DESC LIMIT 1",  # noqa: S608
                active,
            ).fetchone()
        return _run_from_row(row) if row is not None else None

    def list_active(self) -> tuple[RunRecord, ...]:
        active = tuple(state.value for state in ACTIVE_STATES)
        placeholders = ",".join("?" for _ in active)
        with self.database.connect() as connection:
            rows = connection.execute(
                f"SELECT * FROM runs WHERE state IN ({placeholders}) ORDER BY created_at",  # noqa: S608
                active,
            ).fetchall()
        return tuple(_run_from_row(row) for row in rows)

    def list(self, *, limit: int = 100) -> tuple[RunRecord, ...]:
        if isinstance(limit, bool) or not isinstance(limit, int) or not 1 <= limit <= 1000:
            raise ValueError("limit must be between 1 and 1000")
        with self.database.connect() as connection:
            rows = connection.execute(
                "SELECT * FROM runs ORDER BY created_at DESC LIMIT ?",
                (limit,),
            ).fetchall()
        return tuple(_run_from_row(row) for row in rows)

    def transition(
        self,
        run_id: str,
        target: RunState | str,
        *,
        reason_code: ReasonCode | str | None = None,
        at: datetime | None = None,
        source: str = "control-plane",
    ) -> RunRecord:
        target_state = RunState(target)
        reason = ReasonCode(reason_code) if reason_code is not None else None
        changed_at = utc_datetime(at or datetime.now(timezone.utc), field="at")

        with self.database.transaction(immediate=True) as connection:
            row = connection.execute("SELECT * FROM runs WHERE run_id = ?", (run_id,)).fetchone()
            if row is None:
                raise RunNotFound(run_id)
            current = RunState(row["state"])
            if target_state not in ALLOWED_TRANSITIONS[current]:
                raise InvalidTransition(run_id, current, target_state)
            _validate_terminal_reason(target_state, reason)
            ended_at = changed_at if target_state in TERMINAL_STATES else None
            connection.execute(
                """
                UPDATE runs
                SET state = ?, reason_code = ?, updated_at = ?, ended_at = ?
                WHERE run_id = ?
                """,
                (
                    target_state.value,
                    reason.value if reason is not None else None,
                    datetime_to_text(changed_at),
                    datetime_to_text(ended_at) if ended_at is not None else None,
                    run_id,
                ),
            )
            event = self.events.append_in_transaction(
                connection,
                run_id,
                source=source,
                kind="RUN_STATE_CHANGED",
                severity="error" if target_state is RunState.FAILED else "info",
                payload={
                    "previous_state": current.value,
                    "state": target_state.value,
                    "reason_code": reason.value if reason is not None else None,
                },
                wall_time=changed_at,
            )
        self.events.mirror(event)
        return self.get(run_id)


def _validate_terminal_reason(target: RunState, reason: ReasonCode | None) -> None:
    if target is RunState.COMPLETED and reason not in {ReasonCode.NORMAL_EXIT, ReasonCode.USER_STOP}:
        raise InvalidReasonCode("COMPLETED requires NORMAL_EXIT or USER_STOP")
    failure_reasons = {
        ReasonCode.UNEXPECTED_PROCESS_EXIT,
        ReasonCode.READINESS_TIMEOUT,
        ReasonCode.PREFLIGHT_FAILED,
        ReasonCode.NEEDS_INSPECTION,
    }
    if target is RunState.FAILED and reason not in failure_reasons:
        raise InvalidReasonCode("FAILED requires a failure reason code")


def _run_from_row(row: sqlite3.Row) -> RunRecord:
    return RunRecord(
        run_id=row["run_id"],
        scenario_id=row["scenario_id"],
        state=RunState(row["state"]),
        reason_code=ReasonCode(row["reason_code"]) if row["reason_code"] is not None else None,
        created_at=datetime_from_text(row["created_at"]),
        updated_at=datetime_from_text(row["updated_at"]),
        ended_at=datetime_from_text(row["ended_at"]) if row["ended_at"] is not None else None,
        request=json.loads(row["request_json"]),
        resolved_manifest=json.loads(row["resolved_manifest_json"]),
    )


__all__ = [
    "ACTIVE_STATES",
    "ALLOWED_TRANSITIONS",
    "TERMINAL_STATES",
    "InvalidReasonCode",
    "InvalidTransition",
    "ReasonCode",
    "RunNotFound",
    "RunRecord",
    "RunState",
    "RunStateMachine",
]
