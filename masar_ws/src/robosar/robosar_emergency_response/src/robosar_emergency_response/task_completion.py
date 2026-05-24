"""Physical completion evidence for emergency task controllers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping, Sequence

from .checkpoint_controller import object_info_from_global_info, robot_info_from_global_info


@dataclass(frozen=True)
class RoundflockingCompletionEvidence:
    complete: bool
    completed_robot_count: int
    required_robot_count: int
    elapsed: float
    progress_by_robot: Mapping[str, float]
    radius_by_robot: Mapping[str, float]
    missing_robot_names: tuple[str, ...]


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class RoundflockingCompletionMonitor:
    """Accumulate target-relative angular progress as evidence of orbiting."""

    def __init__(
        self,
        robot_names: Sequence[str],
        min_duration: float,
        min_angle_progress: float,
        max_radius: float,
        required_robot_fraction: float = 0.5,
        required_robot_count: int = 0,
        max_angle_step: float = 1.2,
    ):
        self.robot_names = list(robot_names)
        self.min_duration = max(0.0, float(min_duration))
        self.min_angle_progress = max(0.0, float(min_angle_progress))
        self.max_radius = max(0.0, float(max_radius))
        self.required_robot_fraction = max(0.0, min(1.0, float(required_robot_fraction)))
        self.required_robot_count = int(required_robot_count)
        self.max_angle_step = max(0.0, float(max_angle_step))
        self.reset()

    def reset(self) -> None:
        self._last_angles = {}
        self._last_angle_was_valid = {}
        self._signed_progress = {name: 0.0 for name in self.robot_names}
        self._radius = {}

    def update(
        self,
        global_info: Mapping | None,
        now: float,
        elapsed: float,
    ) -> RoundflockingCompletionEvidence:
        if global_info is None:
            return self._evidence(elapsed, ())

        target = object_info_from_global_info(global_info, "flocking_target")
        if target is None:
            return self._evidence(elapsed, tuple(self.robot_names))

        missing = []
        for robot_name in self.robot_names:
            robot = robot_info_from_global_info(global_info, robot_name)
            if robot is None:
                missing.append(robot_name)
                self._last_angle_was_valid[robot_name] = False
                continue

            dx = robot.pose.x - target.x
            dy = robot.pose.y - target.y
            radius = math.hypot(dx, dy)
            angle = math.atan2(dy, dx)
            valid_radius = self.max_radius <= 0.0 or radius <= self.max_radius
            last_angle = self._last_angles.get(robot_name)
            last_valid = self._last_angle_was_valid.get(robot_name, False)

            self._last_angles[robot_name] = angle
            self._last_angle_was_valid[robot_name] = valid_radius
            self._radius[robot_name] = radius

            if not valid_radius or last_angle is None or not last_valid:
                continue

            delta = _normalize_angle(angle - last_angle)
            if self.max_angle_step > 0.0 and abs(delta) > self.max_angle_step:
                continue
            self._signed_progress[robot_name] = (
                self._signed_progress.get(robot_name, 0.0) + delta
            )

        return self._evidence(elapsed, tuple(missing))

    def _required_count(self) -> int:
        if not self.robot_names:
            return 0
        if self.required_robot_count > 0:
            return min(len(self.robot_names), self.required_robot_count)
        return max(1, int(math.ceil(len(self.robot_names) * self.required_robot_fraction)))

    def _evidence(
        self,
        elapsed: float,
        missing_robot_names: tuple[str, ...],
    ) -> RoundflockingCompletionEvidence:
        required = self._required_count()
        completed = sum(
            1
            for robot_name in self.robot_names
            if abs(self._signed_progress.get(robot_name, 0.0)) >= self.min_angle_progress
        )
        progress_by_robot = {
            robot_name: abs(self._signed_progress.get(robot_name, 0.0))
            for robot_name in self.robot_names
        }
        return RoundflockingCompletionEvidence(
            complete=elapsed >= self.min_duration and completed >= required,
            completed_robot_count=completed,
            required_robot_count=required,
            elapsed=float(elapsed),
            progress_by_robot=progress_by_robot,
            radius_by_robot=dict(self._radius),
            missing_robot_names=missing_robot_names,
        )
