from __future__ import annotations

import json
import sqlite3
from datetime import datetime, timezone
from pathlib import Path

import pytest

from masar_platform.db import Database
from masar_platform.events import EventStore
from masar_platform.leases import LeaseStore
from masar_platform.state_machine import (
    ALLOWED_TRANSITIONS,
    InvalidTransition,
    ReasonCode,
    RunState,
    RunStateMachine,
)


NOW = datetime(2026, 8, 1, 1, 2, 3, tzinfo=timezone.utc)


def _stores(tmp_path: Path, name: str = "platform") -> tuple[Database, EventStore, LeaseStore, RunStateMachine]:
    database = Database(tmp_path / f"{name}.sqlite3")
    database.initialize()
    events = EventStore(database, tmp_path / "runs")
    leases = LeaseStore(database, events)
    states = RunStateMachine(database, events)
    return database, events, leases, states


def _advance_to(states: RunStateMachine, run_id: str, state: RunState) -> None:
    paths = {
        RunState.PREFLIGHT: (),
        RunState.STARTING: (RunState.STARTING,),
        RunState.RUNNING: (RunState.STARTING, RunState.RUNNING),
        RunState.STOPPING: (RunState.STOPPING,),
        RunState.COMPLETED: (RunState.STOPPING, RunState.COMPLETED),
        RunState.FAILED: (RunState.FAILED,),
    }
    for target in paths[state]:
        reason = None
        if target is RunState.COMPLETED:
            reason = ReasonCode.USER_STOP
        elif target is RunState.FAILED:
            reason = ReasonCode.PREFLIGHT_FAILED
        states.transition(run_id, target, reason_code=reason, at=NOW)


@pytest.mark.parametrize(
    ("source", "target"),
    tuple(
        (source, target)
        for source, targets in ALLOWED_TRANSITIONS.items()
        for target in targets
    ),
)
def test_every_approved_transition_is_persisted_with_an_event(
    tmp_path: Path,
    source: RunState,
    target: RunState,
) -> None:
    database, events, leases, states = _stores(tmp_path)
    lease = leases.acquire("run-1", "scenario-1", "api-1", at=NOW)
    assert lease.run_id == "run-1"
    _advance_to(states, "run-1", source)

    reason = None
    if target is RunState.COMPLETED:
        reason = ReasonCode.USER_STOP
    elif target is RunState.FAILED:
        reason = ReasonCode.PREFLIGHT_FAILED
    changed = states.transition("run-1", target, reason_code=reason, at=NOW)

    assert changed.state is target
    persisted = states.get("run-1")
    assert persisted.state is target
    state_event = events.list("run-1")[-1]
    assert state_event.kind == "RUN_STATE_CHANGED"
    assert state_event.payload == {
        "previous_state": source.value,
        "state": target.value,
        "reason_code": reason.value if reason is not None else None,
    }
    with database.connect() as connection:
        row = connection.execute("SELECT state FROM runs WHERE run_id = ?", ("run-1",)).fetchone()
    assert row["state"] == target.value


def test_creation_is_absence_to_preflight_and_emits_the_first_event(tmp_path: Path) -> None:
    _, events, leases, states = _stores(tmp_path)

    leases.acquire("run-1", "scenario-1", "api-1", at=NOW)

    run = states.get("run-1")
    assert run.state is RunState.PREFLIGHT
    assert run.reason_code is None
    assert events.list("run-1")[0].payload == {
        "previous_state": None,
        "state": "PREFLIGHT",
        "reason_code": None,
    }


def test_every_unapproved_transition_raises_typed_error(tmp_path: Path) -> None:
    all_states = set(RunState)
    for index, source in enumerate(RunState):
        invalid_targets = all_states - set(ALLOWED_TRANSITIONS[source])
        for target in invalid_targets:
            _, _, leases, states = _stores(tmp_path, f"invalid-{index}-{target.value}")
            leases.acquire("run-1", "scenario-1", "api-1", at=NOW)
            _advance_to(states, "run-1", source)

            with pytest.raises(InvalidTransition) as caught:
                states.transition("run-1", target, reason_code=ReasonCode.PREFLIGHT_FAILED, at=NOW)

            assert caught.value.run_id == "run-1"
            assert caught.value.current is source
            assert caught.value.target is target
            assert states.get("run-1").state is source


@pytest.mark.parametrize(
    ("path", "final_state", "reason"),
    (
        ((RunState.STARTING, RunState.RUNNING, RunState.STOPPING), RunState.COMPLETED, ReasonCode.USER_STOP),
        ((RunState.STARTING, RunState.RUNNING, RunState.STOPPING), RunState.COMPLETED, ReasonCode.NORMAL_EXIT),
        ((RunState.STARTING, RunState.RUNNING), RunState.FAILED, ReasonCode.UNEXPECTED_PROCESS_EXIT),
        ((RunState.STARTING,), RunState.FAILED, ReasonCode.READINESS_TIMEOUT),
        ((), RunState.FAILED, ReasonCode.PREFLIGHT_FAILED),
        ((RunState.STARTING,), RunState.FAILED, ReasonCode.NEEDS_INSPECTION),
    ),
)
def test_frozen_terminal_reason_semantics(
    tmp_path: Path,
    path: tuple[RunState, ...],
    final_state: RunState,
    reason: ReasonCode,
) -> None:
    _, events, leases, states = _stores(tmp_path)
    leases.acquire("run-1", "scenario-1", "api-1", at=NOW)
    for target in path:
        stop_reason = reason if target is RunState.STOPPING else None
        states.transition("run-1", target, reason_code=stop_reason, at=NOW)

    result = states.transition("run-1", final_state, reason_code=reason, at=NOW)

    assert result.state is final_state
    assert result.reason_code is reason
    assert events.list("run-1")[-1].payload["reason_code"] == reason.value
    assert result.state.value != "FAILED_NEEDS_INSPECTION"


def test_state_and_event_write_roll_back_together(tmp_path: Path) -> None:
    database, events, leases, states = _stores(tmp_path)
    leases.acquire("run-1", "scenario-1", "api-1", at=NOW)
    archive_path = events.archive_path("run-1")
    original_archive = archive_path.read_text(encoding="utf-8")
    with database.connect() as connection:
        connection.execute(
            """
            CREATE TRIGGER reject_starting_event
            BEFORE INSERT ON events
            WHEN NEW.kind = 'RUN_STATE_CHANGED'
                 AND json_extract(NEW.payload, '$.state') = 'STARTING'
            BEGIN
                SELECT RAISE(ABORT, 'event rejected');
            END
            """
        )

    with pytest.raises(sqlite3.IntegrityError, match="event rejected"):
        states.transition("run-1", RunState.STARTING, at=NOW)

    assert states.get("run-1").state is RunState.PREFLIGHT
    assert len(events.list("run-1")) == 1
    assert archive_path.read_text(encoding="utf-8") == original_archive


def test_committed_state_event_is_mirrored_as_jsonl(tmp_path: Path) -> None:
    _, events, leases, states = _stores(tmp_path)
    leases.acquire("run-1", "scenario-1", "api-1", at=NOW)

    states.transition("run-1", RunState.STARTING, at=NOW)

    lines = events.archive_path("run-1").read_text(encoding="utf-8").splitlines()
    mirrored = [json.loads(line) for line in lines]
    assert [event["sequence"] for event in mirrored] == [1, 2]
    assert mirrored[-1] == events.list("run-1")[-1].as_dict()
