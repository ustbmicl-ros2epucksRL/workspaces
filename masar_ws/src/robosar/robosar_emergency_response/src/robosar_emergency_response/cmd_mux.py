"""Pure command selection and LiDAR safety logic for response robots."""

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
class RangeExclusion:
    bearing: float
    center_distance: float
    radius: float
    extra_half_angle: float = 0.0
    range_padding: float = 0.03


ZERO_TWIST = Twist2D(0.0, 0.0)


def _fresh(last_time: float | None, now: float, timeout: float) -> bool:
    return last_time is not None and (now - last_time) <= timeout


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _range_excluded_by_distance(
    value: float,
    angle: float,
    range_exclusions: Sequence[RangeExclusion],
) -> bool:
    if not math.isfinite(value) or value <= 0.0:
        return False

    for exclusion in range_exclusions:
        center_distance = float(exclusion.center_distance)
        radius = max(0.0, float(exclusion.radius))
        if center_distance <= 0.0 or radius <= 0.0:
            continue

        angular_radius = math.asin(min(1.0, radius / max(center_distance, 1e-6)))
        half_angle = min(math.pi, angular_radius + abs(float(exclusion.extra_half_angle)))
        if abs(_normalize_angle(angle - exclusion.bearing)) > half_angle:
            continue

        padding = max(0.0, float(exclusion.range_padding))
        min_distance = max(0.0, center_distance - radius - padding)
        max_distance = center_distance + radius + padding
        if min_distance <= value <= max_distance:
            return True

    return False


def _range_inside_self_footprint(value: float, self_filter_radius: float) -> bool:
    if not math.isfinite(value) or value <= 0.0:
        return False
    return value <= max(0.0, float(self_filter_radius))


def clamp_command(
    command: Twist2D,
    max_translational_velocity: float,
    max_rotational_velocity: float,
) -> Twist2D:
    linear_limit = abs(float(max_translational_velocity))
    angular_limit = abs(float(max_rotational_velocity))
    linear = max(-linear_limit, min(linear_limit, float(command.linear_x)))
    angular = max(-angular_limit, min(angular_limit, float(command.angular_z)))
    if not math.isfinite(linear) or not math.isfinite(angular):
        return ZERO_TWIST
    return Twist2D(linear, angular)


def apply_hard_scan_safety(
    command: Twist2D,
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    hard_stop_distance: float,
    motion_half_width: float,
    scan_age: float | None,
    scan_timeout: float,
    range_exclusions: Sequence[RangeExclusion] | None = None,
    self_filter_radius: float = 0.0,
) -> Twist2D:
    """Enforce a swept-footprint stop without synthesizing steering.

    Only returns inside the finite body-width corridor swept in the commanded
    translation direction block motion. A stale or missing scan fails closed.
    """
    if command == ZERO_TWIST:
        return ZERO_TWIST
    timeout = max(0.0, float(scan_timeout))
    if scan_age is None or not math.isfinite(scan_age) or scan_age > timeout:
        return ZERO_TWIST
    if abs(command.linear_x) <= 1e-9:
        return command

    distance_limit = max(0.0, float(hard_stop_distance))
    half_width = max(0.0, float(motion_half_width))
    direction = 0.0 if command.linear_x > 0.0 else math.pi
    exclusions = range_exclusions or []
    for index, raw_range in enumerate(ranges):
        if not math.isfinite(raw_range) or raw_range <= 0.0:
            continue
        angle = _normalize_angle(float(angle_min) + index * float(angle_increment))
        relative_angle = _normalize_angle(angle - direction)
        longitudinal_distance = raw_range * math.cos(relative_angle)
        lateral_distance = abs(raw_range * math.sin(relative_angle))
        if longitudinal_distance <= 0.0 or longitudinal_distance > distance_limit:
            continue
        if lateral_distance > half_width:
            continue
        if _range_inside_self_footprint(raw_range, self_filter_radius):
            continue
        if _range_excluded_by_distance(raw_range, angle, exclusions):
            continue
        return Twist2D(0.0, command.angular_z)
    return command


def select_command(
    mode: str,
    motion_enabled: bool,
    deform_cmd: Twist2D,
    task_cmd: Twist2D,
    now: float,
    last_deform_time: float | None,
    last_task_time: float | None,
    command_timeout: float,
) -> Twist2D:
    if not motion_enabled:
        return ZERO_TWIST
    if mode in (MODE_STOP, MODE_FAILED):
        return ZERO_TWIST
    if mode == MODE_NAV_FORMATION:
        return deform_cmd if _fresh(last_deform_time, now, command_timeout) else ZERO_TWIST
    if mode == MODE_TASK_CONTROL:
        return task_cmd if _fresh(last_task_time, now, command_timeout) else ZERO_TWIST
    return ZERO_TWIST
