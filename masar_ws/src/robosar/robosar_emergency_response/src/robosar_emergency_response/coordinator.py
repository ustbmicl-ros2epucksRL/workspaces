"""Arrival barrier logic for the stage-0 response group."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, List, Mapping, Sequence

from .formation import (
    FormationSlot,
    Pose2D,
    assigned_slot_targets,
    formation_center,
    normalize_angle,
)


@dataclass(frozen=True)
class RobotState:
    name: str
    x: float
    y: float
    yaw: float = 0.0

    @property
    def pose(self) -> Pose2D:
        return Pose2D(self.x, self.y, self.yaw)


@dataclass(frozen=True)
class ArrivalResult:
    arrived: bool
    all_inside: bool
    max_distance: float
    targets: Mapping[str, Pose2D]
    center: Pose2D


@dataclass(frozen=True)
class GoalRegionResult:
    arrived: bool
    all_inside: bool
    max_robot_distance: float
    center_distance: float
    center: Pose2D


class ArrivalBarrier:
    def __init__(
        self,
        robot_names: Sequence[str],
        slots: Sequence[FormationSlot],
        arrival_radius: float,
        yaw_tolerance: float,
        hold_seconds: float,
        use_yaw: bool = False,
    ):
        if len(robot_names) != len(slots):
            raise ValueError("robot_names and slots must have the same length")
        self.robot_names = list(robot_names)
        self.slots = list(slots)
        self.arrival_radius = float(arrival_radius)
        self.yaw_tolerance = float(yaw_tolerance)
        self.hold_seconds = float(hold_seconds)
        self.use_yaw = bool(use_yaw)
        self._inside_since = None

    def reset(self) -> None:
        self._inside_since = None

    def set_slots(self, slots: Sequence[FormationSlot]) -> None:
        if len(slots) != len(self.robot_names):
            raise ValueError("slots length must match robot_names length")
        self.slots = list(slots)
        self.reset()

    def evaluate(
        self,
        goal: Pose2D,
        robot_states: Mapping[str, RobotState],
        now: float,
        remap_ids: Sequence[int] | None = None,
        slot_rotation_yaw: float = 0.0,
    ) -> ArrivalResult:
        rotated_goal = Pose2D(goal.x, goal.y, slot_rotation_yaw)
        targets_list = assigned_slot_targets(
            rotated_goal,
            self.slots,
            remap_ids,
            rotate_by_goal_yaw=True,
        )
        targets = dict(zip(self.robot_names, targets_list))
        distances: List[float] = []
        inside = True
        poses = []

        for robot_name in self.robot_names:
            state = robot_states.get(robot_name)
            if state is None:
                inside = False
                distances.append(float("inf"))
                continue
            target = targets[robot_name]
            poses.append(state.pose)
            distance = math.hypot(state.x - target.x, state.y - target.y)
            distances.append(distance)
            yaw_ok = True
            if self.use_yaw:
                yaw_ok = abs(normalize_angle(state.yaw - target.yaw)) <= self.yaw_tolerance
            if distance > self.arrival_radius or not yaw_ok:
                inside = False

        if inside:
            if self._inside_since is None:
                self._inside_since = now
            arrived = (now - self._inside_since) >= self.hold_seconds
        else:
            self._inside_since = None
            arrived = False

        return ArrivalResult(
            arrived=arrived,
            all_inside=inside,
            max_distance=max(distances) if distances else float("inf"),
            targets=targets,
            center=formation_center(poses),
        )


class GoalRegionBarrier:
    """Stage-0 success gate when exact task-point formation is intentionally loose."""

    def __init__(
        self,
        robot_names: Sequence[str],
        robot_radius: float,
        center_radius: float,
        hold_seconds: float,
    ):
        self.robot_names = list(robot_names)
        self.robot_radius = float(robot_radius)
        self.center_radius = float(center_radius)
        self.hold_seconds = float(hold_seconds)
        self._inside_since = None

    def reset(self) -> None:
        self._inside_since = None

    def evaluate(
        self,
        goal: Pose2D,
        robot_states: Mapping[str, RobotState],
        now: float,
    ) -> GoalRegionResult:
        distances: List[float] = []
        poses = []
        inside = True

        for robot_name in self.robot_names:
            state = robot_states.get(robot_name)
            if state is None:
                inside = False
                distances.append(float("inf"))
                continue
            poses.append(state.pose)
            distance = math.hypot(state.x - goal.x, state.y - goal.y)
            distances.append(distance)
            if distance > self.robot_radius:
                inside = False

        center = formation_center(poses)
        center_distance = math.hypot(center.x - goal.x, center.y - goal.y)
        if center_distance > self.center_radius:
            inside = False

        if inside:
            if self._inside_since is None:
                self._inside_since = now
            arrived = (now - self._inside_since) >= self.hold_seconds
        else:
            self._inside_since = None
            arrived = False

        return GoalRegionResult(
            arrived=arrived,
            all_inside=inside,
            max_robot_distance=max(distances) if distances else float("inf"),
            center_distance=center_distance,
            center=center,
        )
