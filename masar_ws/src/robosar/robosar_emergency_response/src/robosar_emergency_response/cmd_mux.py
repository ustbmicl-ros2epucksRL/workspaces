"""Pure command selection for response-group velocity muxing."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence


MODE_NAV_FORMATION = "NAV_FORMATION"
MODE_TASK_CONTROL = "TASK_CONTROL"
MODE_STOP = "STOP"
MODE_FAILED = "FAILED"


@dataclass(frozen=True)
class Twist2D:
    linear_x: float = 0.0
    angular_z: float = 0.0


@dataclass(frozen=True)
class ScanSectorRanges:
    front: float | None = None
    left: float | None = None
    right: float | None = None
    rear: float | None = None
    all_min: float | None = None


ZERO_TWIST = Twist2D(0.0, 0.0)


def _fresh(last_time: float | None, now: float, timeout: float) -> bool:
    return last_time is not None and (now - last_time) <= timeout


def _apply_safety(
    command: Twist2D,
    safety_min_range: float | None,
    stop_distance: float,
    slowdown_distance: float,
) -> Twist2D:
    if safety_min_range is None:
        return command
    if safety_min_range <= stop_distance:
        return Twist2D(0.0, command.angular_z)
    if safety_min_range < slowdown_distance and slowdown_distance > stop_distance:
        scale = (safety_min_range - stop_distance) / (slowdown_distance - stop_distance)
        return Twist2D(command.linear_x * max(0.0, min(1.0, scale)), command.angular_z)
    return command


def min_range_in_front(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    front_angle: float,
) -> float | None:
    half_angle = abs(front_angle) / 2.0
    values = []
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0.0:
            continue
        angle = _normalize_angle(angle_min + index * angle_increment)
        if abs(angle) <= half_angle:
            values.append(value)
    return min(values) if values else None


def min_valid_range(ranges: Sequence[float]) -> float | None:
    values = [value for value in ranges if math.isfinite(value) and value > 0.0]
    return min(values) if values else None


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def min_range_excluding_angles(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    excluded_angles: Sequence[float],
    excluded_half_angle: float,
) -> float | None:
    values = []
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0.0:
            continue
        angle = angle_min + index * angle_increment
        if any(
            abs(_normalize_angle(angle - excluded_angle)) <= excluded_half_angle
            for excluded_angle in excluded_angles
        ):
            continue
        values.append(value)
    return min(values) if values else None


def min_range_in_sector(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    center_angle: float,
    half_angle: float,
    excluded_angles: Sequence[float] | None = None,
    excluded_half_angle: float = 0.0,
) -> float | None:
    excluded_angles = excluded_angles or []
    values = []
    for index, value in enumerate(ranges):
        if not math.isfinite(value) or value <= 0.0:
            continue
        angle = _normalize_angle(angle_min + index * angle_increment)
        if abs(_normalize_angle(angle - center_angle)) > abs(half_angle):
            continue
        if any(
            abs(_normalize_angle(angle - excluded_angle)) <= excluded_half_angle
            for excluded_angle in excluded_angles
        ):
            continue
        values.append(value)
    return min(values) if values else None


def scan_sector_ranges(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    front_angle: float,
    side_angle: float,
    rear_angle: float,
    excluded_angles: Sequence[float] | None = None,
    excluded_half_angle: float = 0.0,
) -> ScanSectorRanges:
    excluded_angles = excluded_angles or []
    return ScanSectorRanges(
        front=min_range_in_sector(
            ranges,
            angle_min,
            angle_increment,
            center_angle=0.0,
            half_angle=front_angle / 2.0,
            excluded_angles=excluded_angles,
            excluded_half_angle=excluded_half_angle,
        ),
        left=min_range_in_sector(
            ranges,
            angle_min,
            angle_increment,
            center_angle=math.pi / 2.0,
            half_angle=side_angle / 2.0,
            excluded_angles=excluded_angles,
            excluded_half_angle=excluded_half_angle,
        ),
        right=min_range_in_sector(
            ranges,
            angle_min,
            angle_increment,
            center_angle=-math.pi / 2.0,
            half_angle=side_angle / 2.0,
            excluded_angles=excluded_angles,
            excluded_half_angle=excluded_half_angle,
        ),
        rear=min_range_in_sector(
            ranges,
            angle_min,
            angle_increment,
            center_angle=math.pi,
            half_angle=rear_angle / 2.0,
            excluded_angles=excluded_angles,
            excluded_half_angle=excluded_half_angle,
        ),
        all_min=min_range_excluding_angles(
            ranges,
            angle_min,
            angle_increment,
            excluded_angles,
            excluded_half_angle,
        )
        if excluded_angles
        else min_valid_range(ranges),
    )


def _open_side_turn(left_range: float | None, right_range: float | None, speed: float) -> float:
    left_clearance = left_range if left_range is not None else float("inf")
    right_clearance = right_range if right_range is not None else float("inf")
    return abs(speed) if left_clearance >= right_clearance else -abs(speed)


def _scale_forward(command: Twist2D, scale: float) -> Twist2D:
    if command.linear_x <= 0.0:
        return command
    return Twist2D(command.linear_x * max(0.0, min(1.0, scale)), command.angular_z)


def apply_scan_safety(
    command: Twist2D,
    sectors: ScanSectorRanges | None,
    stop_distance: float,
    slowdown_distance: float,
    escape_turn_speed: float,
    side_stop_distance: float = 0.0,
    side_crawl_speed: float = 0.08,
) -> Twist2D:
    if sectors is None:
        return command

    if sectors.front is not None and sectors.front <= stop_distance:
        return Twist2D(0.0, _open_side_turn(sectors.left, sectors.right, escape_turn_speed))

    safe = command
    if (
        sectors.front is not None
        and sectors.front < slowdown_distance
        and slowdown_distance > stop_distance
    ):
        scale = (sectors.front - stop_distance) / (slowdown_distance - stop_distance)
        safe = _scale_forward(safe, scale)

    side_ranges = [value for value in (sectors.left, sectors.right) if value is not None]
    if side_stop_distance > 0.0 and side_ranges and min(side_ranges) <= side_stop_distance:
        turn = _open_side_turn(sectors.left, sectors.right, escape_turn_speed)
        if safe.linear_x > 0.0:
            safe = Twist2D(min(safe.linear_x, max(0.0, side_crawl_speed)), turn)
        else:
            safe = Twist2D(safe.linear_x, turn)

    return safe


def is_stuck_candidate(
    raw_command: Twist2D,
    sectors: ScanSectorRanges | None,
    linear_speed: float,
    angular_speed: float,
    linear_command_threshold: float,
    angular_command_threshold: float,
    linear_speed_threshold: float,
    angular_speed_threshold: float,
    obstacle_distance: float,
) -> bool:
    if sectors is None:
        return False

    near_obstacle = (
        sectors.all_min is not None and sectors.all_min <= obstacle_distance
    )
    if not near_obstacle:
        return False

    commanded_forward = raw_command.linear_x >= linear_command_threshold
    commanded_turn = abs(raw_command.angular_z) >= angular_command_threshold
    linear_stalled = linear_speed <= linear_speed_threshold
    angular_stalled = angular_speed <= angular_speed_threshold

    return linear_stalled and (
        commanded_forward or (commanded_turn and angular_stalled)
    )


def apply_stuck_recovery(
    command: Twist2D,
    sectors: ScanSectorRanges | None,
    is_stuck: bool,
    reverse_speed: float,
    reverse_clearance: float,
    escape_turn_speed: float,
) -> Twist2D:
    if not is_stuck or sectors is None:
        return command
    turn = _open_side_turn(sectors.left, sectors.right, escape_turn_speed)
    rear_clear = sectors.rear is None or sectors.rear >= reverse_clearance
    linear = -abs(reverse_speed) if rear_clear else 0.0
    return Twist2D(linear, turn)


def hysteresis_avoid_state(
    previous_state: bool,
    min_range: float | None,
    reduce_distance: float,
    clear_distance: float,
) -> bool:
    if min_range is None:
        return False
    clear_distance = max(clear_distance, reduce_distance)
    if previous_state:
        return min_range < clear_distance
    return min_range < reduce_distance


def select_command(
    mode: str,
    deform_cmd: Twist2D,
    task_cmd: Twist2D,
    now: float,
    last_deform_time: float | None,
    last_task_time: float | None,
    command_timeout: float,
    safety_min_range: float | None = None,
    stop_distance: float = 0.25,
    slowdown_distance: float = 0.55,
) -> Twist2D:
    if mode in (MODE_STOP, MODE_FAILED):
        return ZERO_TWIST
    if mode == MODE_NAV_FORMATION:
        command = deform_cmd if _fresh(last_deform_time, now, command_timeout) else ZERO_TWIST
    elif mode == MODE_TASK_CONTROL:
        command = task_cmd if _fresh(last_task_time, now, command_timeout) else ZERO_TWIST
    else:
        command = ZERO_TWIST
    return _apply_safety(command, safety_min_range, stop_distance, slowdown_distance)
