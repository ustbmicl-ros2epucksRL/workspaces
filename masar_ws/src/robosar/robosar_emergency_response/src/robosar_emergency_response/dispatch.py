"""Goal dispatch helpers for the DEFORM bridge."""

from __future__ import annotations

from dataclasses import dataclass

from .formation import Pose2D
from .goal_sequence import GoalSpec


@dataclass(frozen=True)
class NavGoal:
    goal_id: int
    frame_id: str
    pose: Pose2D


def prepare_nav_goal(goal: GoalSpec) -> NavGoal:
    return NavGoal(
        goal_id=goal.goal_id,
        frame_id=goal.frame_id,
        pose=Pose2D(goal.x, goal.y, goal.yaw),
    )


def resolve_goal_id(explicit_goal_id: int | None, header_seq: int) -> int:
    """Prefer application-level task ids over ROS Header.seq transport counters."""
    if explicit_goal_id is not None:
        return int(explicit_goal_id)
    return int(header_seq)
