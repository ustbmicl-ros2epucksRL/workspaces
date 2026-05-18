"""Formation geometry shared by the stage-0 response nodes."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, List, Sequence


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


def generate_deform_slots(
    robot_num: int,
    one_row_num: int,
    interval: float,
) -> List[FormationSlot]:
    """Mirror DEFORM PlanManager::generateFormationPositions."""
    if robot_num <= 0:
        raise ValueError("robot_num must be positive")
    if one_row_num <= 0 or robot_num < one_row_num:
        raise ValueError("one_row_num must be in [1, robot_num]")

    positions = []
    leader_pos = None
    leader_index = 0
    rows = robot_num // one_row_num
    remaining = robot_num % one_row_num

    for row in range(rows):
        for col in range(one_row_num):
            positions.append((row * interval, col * interval))

    if remaining > 0:
        base_x = rows * interval
        center_y = (one_row_num - 1) * interval / 2.0
        if remaining % 2 == 1:
            leader_pos = (base_x, center_y)
            leader_index = len(positions)
            positions.append(leader_pos)
            remaining -= 1
            for idx in range(1, remaining // 2 + 1):
                positions.append((base_x, leader_pos[1] + idx * interval))
                positions.append((base_x, leader_pos[1] - idx * interval))
        else:
            leader_pos = (base_x, center_y + interval / 2.0)
            leader_index = len(positions)
            positions.append(leader_pos)
            right_pos = (base_x, center_y - interval / 2.0)
            positions.append(right_pos)
            remaining -= 2
            for idx in range(1, remaining // 2 + 1):
                positions.append((leader_pos[0], leader_pos[1] + idx * interval))
                positions.append((right_pos[0], right_pos[1] - idx * interval))
    else:
        leader_pos = ((rows - 1) * interval, (one_row_num // 2) * interval)
        leader_index = (rows - 1) * one_row_num + (one_row_num // 2)

    centered = [
        FormationSlot(x=pos[0] - leader_pos[0], y=pos[1] - leader_pos[1])
        for pos in positions
    ]
    centered[0], centered[leader_index] = centered[leader_index], centered[0]
    return centered


def rotate_slot(slot: FormationSlot, yaw: float) -> FormationSlot:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return FormationSlot(x=c * slot.x - s * slot.y, y=s * slot.x + c * slot.y)


def assigned_slot_targets(
    goal: Pose2D,
    slots: Sequence[FormationSlot],
    remap_ids: Sequence[int] | None = None,
    rotate_by_goal_yaw: bool = False,
) -> List[Pose2D]:
    if remap_ids is None:
        remap_ids = list(range(len(slots)))
    if len(remap_ids) != len(slots):
        raise ValueError("remap_ids length must match slots length")

    targets = []
    for slot_id in remap_ids:
        if slot_id < 0 or slot_id >= len(slots):
            raise ValueError("remap_ids contains an invalid slot id")
        slot = slots[slot_id]
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

