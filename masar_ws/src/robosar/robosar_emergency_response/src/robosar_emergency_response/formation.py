"""Formation geometry shared by the stage-0 response nodes."""

from __future__ import annotations

from dataclasses import dataclass
import itertools
import math
from typing import Dict, Iterable, List, Mapping, Sequence


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass(frozen=True)
class FormationSlot:
    x: float
    y: float


def normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def generate_square_slots(robot_num: int, interval: float) -> List[FormationSlot]:
    """Return fixed, centroid-aligned response-group slots.

    The response group now keeps one shape only. For the normal four-TB3
    response group, slot order is front-left, front-right, rear-right,
    rear-left around the goal center.
    """
    if robot_num <= 0:
        raise ValueError("robot_num must be positive")
    if interval <= 0.0:
        raise ValueError("interval must be positive")

    if robot_num == 4:
        half = 0.5 * interval
        return [
            FormationSlot(half, half),
            FormationSlot(half, -half),
            FormationSlot(-half, -half),
            FormationSlot(-half, half),
        ]

    columns = int(math.ceil(math.sqrt(robot_num)))
    rows = int(math.ceil(robot_num / columns))
    x_origin = 0.5 * (columns - 1) * interval
    y_origin = 0.5 * (rows - 1) * interval
    return [
        FormationSlot(
            x=x_origin - (index % columns) * interval,
            y=y_origin - (index // columns) * interval,
        )
        for index in range(robot_num)
    ]


def rotate_slot(slot: FormationSlot, yaw: float) -> FormationSlot:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return FormationSlot(x=c * slot.x - s * slot.y, y=s * slot.x + c * slot.y)


def assign_slots_by_current_geometry(
    robot_names: Sequence[str],
    robot_poses: Mapping[str, Pose2D],
    slots: Sequence[FormationSlot],
    yaw: float = 0.0,
) -> Dict[str, int]:
    """Bind each robot to the nearest current formation corner.

    This keeps the visible square from asking robots to swap corners when the
    nominal robot-id order does not match the spawned/current geometry.
    """
    if len(robot_names) != len(slots):
        raise ValueError("robot_names and slots must have the same length")

    poses = [robot_poses.get(name) for name in robot_names]
    if any(pose is None for pose in poses):
        return {name: index for index, name in enumerate(robot_names)}

    center = formation_center(poses)
    rotated_slots = [rotate_slot(slot, yaw) for slot in slots]

    best_assignment = None
    best_cost = float("inf")
    for assignment in itertools.permutations(range(len(slots))):
        cost = 0.0
        for robot_index, slot_index in enumerate(assignment):
            pose = poses[robot_index]
            slot = rotated_slots[slot_index]
            dx = pose.x - (center.x + slot.x)
            dy = pose.y - (center.y + slot.y)
            cost += dx * dx + dy * dy
        if cost < best_cost:
            best_cost = cost
            best_assignment = assignment

    if best_assignment is None:
        return {name: index for index, name in enumerate(robot_names)}
    return {name: int(best_assignment[index]) for index, name in enumerate(robot_names)}


def assigned_slot_targets(
    goal: Pose2D,
    slots: Sequence[FormationSlot],
    rotate_by_goal_yaw: bool = False,
) -> List[Pose2D]:
    targets = []
    for slot in slots:
        if rotate_by_goal_yaw:
            slot = rotate_slot(slot, goal.yaw)
        targets.append(Pose2D(goal.x + slot.x, goal.y + slot.y, goal.yaw))
    return targets


def formation_center(poses: Iterable[Pose2D]) -> Pose2D:
    poses = list(poses)
    if not poses:
        return Pose2D(0.0, 0.0, 0.0)
    return Pose2D(
        x=sum(pose.x for pose in poses) / len(poses),
        y=sum(pose.y for pose in poses) / len(poses),
        yaw=0.0,
    )
