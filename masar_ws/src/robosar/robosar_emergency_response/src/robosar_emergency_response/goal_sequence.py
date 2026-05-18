"""Stage-0 goal sequence state machine."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, List, Optional


NEW = "NEW"
ACTIVE = "ACTIVE"
DONE = "DONE"
FAILED = "FAILED"


@dataclass(frozen=True)
class GoalSpec:
    goal_id: int
    x: float
    y: float
    yaw: float = 0.0
    frame_id: str = "map"


class GoalSequence:
    def __init__(self, goals: Iterable[GoalSpec]):
        self._goals: List[GoalSpec] = list(goals)
        if not self._goals:
            raise ValueError("at least one goal is required")
        self._index = 0
        self._completed_ids = set()

    @property
    def complete(self) -> bool:
        return self._index >= len(self._goals)

    def current(self) -> Optional[GoalSpec]:
        if self.complete:
            return None
        return self._goals[self._index]

    def handle_status(self, goal_id: int, state: str) -> Optional[GoalSpec]:
        current = self.current()
        if current is None or goal_id != current.goal_id or state != DONE:
            return None
        if goal_id in self._completed_ids:
            return None

        self._completed_ids.add(goal_id)
        self._index += 1
        return self.current()


def goals_from_config(config: dict) -> List[GoalSpec]:
    raw_goals = config.get("goals", [])
    goals = []
    for index, raw_goal in enumerate(raw_goals):
        goals.append(
            GoalSpec(
                goal_id=int(raw_goal.get("id", index)),
                x=float(raw_goal["x"]),
                y=float(raw_goal["y"]),
                yaw=float(raw_goal.get("yaw", 0.0)),
                frame_id=str(raw_goal.get("frame_id", config.get("frame_id", "map"))),
            )
        )
    return goals


def as_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)
