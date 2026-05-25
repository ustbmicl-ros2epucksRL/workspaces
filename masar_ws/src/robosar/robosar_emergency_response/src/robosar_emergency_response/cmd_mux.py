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
class PotentialFieldDecision:
    avoid_needed: bool
    command: Twist2D
    min_range: float | None = None
    open_angle: float | None = None
    open_range: float | None = None
    obstacle_count: int = 0


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


def min_valid_range(ranges: Sequence[float]) -> float | None:
    values = [value for value in ranges if math.isfinite(value) and value > 0.0]
    return min(values) if values else None


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
        angle = _normalize_angle(angle_min + index * angle_increment)
        if any(
            abs(_normalize_angle(angle - excluded_angle)) <= excluded_half_angle
            for excluded_angle in excluded_angles
        ):
            continue
        values.append(value)
    return min(values) if values else None


def _range_excluded(angle: float, excluded_angles: Sequence[float], excluded_half_angle: float) -> bool:
    return any(
        abs(_normalize_angle(angle - excluded_angle)) <= excluded_half_angle
        for excluded_angle in excluded_angles
    )


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


def expand_excluded_angle_window(
    center_angle: float,
    angular_radius: float,
    excluded_half_angle: float,
) -> list[float]:
    half_angle = abs(float(excluded_half_angle))
    radius = abs(float(angular_radius))
    if half_angle <= 0.0 or radius <= half_angle:
        return [_normalize_angle(center_angle)]

    spacing = max(half_angle * 1.8, 1e-6)
    segments = max(1, int(math.ceil((2.0 * radius) / spacing)))
    return [
        _normalize_angle(center_angle - radius + (2.0 * radius * index / segments))
        for index in range(segments + 1)
    ]


def _adjust_range(value: float, min_range: float, max_range: float) -> float:
    if not math.isfinite(value) or value <= 0.0:
        return max_range
    if value < min_range:
        return 0.0
    if value > max_range:
        return max_range
    return float(value)


def _twist_from_field_vector(
    vector_x: float,
    vector_y: float,
    max_translational_velocity: float,
    max_rotational_velocity: float,
) -> Twist2D:
    if not math.isfinite(vector_x) or not math.isfinite(vector_y):
        return ZERO_TWIST
    norm = math.hypot(vector_x, vector_y)
    if norm <= 1e-9:
        return ZERO_TWIST

    unit_x = vector_x / norm
    unit_y = vector_y / norm
    linear = float(max_translational_velocity) * unit_x
    angular = float(max_rotational_velocity) * math.asin(max(-1.0, min(1.0, unit_y)))
    angular = max(-abs(float(max_rotational_velocity)), min(abs(float(max_rotational_velocity)), angular))
    return Twist2D(float(linear), float(angular))


def potential_field_safety(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    max_range: float,
    min_range: float,
    front_attraction: float,
    threshold: int,
    max_translational_velocity: float,
    max_rotational_velocity: float,
    excluded_angles: Sequence[float] | None = None,
    excluded_half_angle: float = 0.0,
    range_exclusions: Sequence[RangeExclusion] | None = None,
    self_filter_radius: float = 0.0,
) -> PotentialFieldDecision:
    """Compute ROS2swarm-style potential-field avoidance from LiDAR samples.

    Teammate bearings can be excluded before both obstacle counting and field
    vector construction, so nearby formation partners do not trigger avoidance.
    """
    if max_range <= 0.0 or not ranges:
        return PotentialFieldDecision(False, ZERO_TWIST)

    excluded_angles = excluded_angles or []
    range_exclusions = range_exclusions or []
    adjusted_ranges: list[float] = []
    visible_ranges: list[float] = []
    angles: list[float] = []
    obstacle_count = 0
    open_angle = None
    open_range = None

    for index, value in enumerate(ranges):
        angle = _normalize_angle(angle_min + index * angle_increment)
        excluded = _range_inside_self_footprint(value, self_filter_radius)
        excluded = excluded or _range_excluded(angle, excluded_angles, excluded_half_angle)
        excluded = excluded or _range_excluded_by_distance(value, angle, range_exclusions)
        adjusted = max_range if excluded else _adjust_range(value, min_range, max_range)
        adjusted_ranges.append(adjusted)
        angles.append(angle)

        if excluded:
            continue
        if math.isfinite(value) and value > 0.0:
            visible_ranges.append(float(value))
        if 0.0 < adjusted < max_range:
            obstacle_count += 1
        if adjusted <= 0.0:
            continue
        if open_range is None or adjusted > open_range or (
            math.isclose(adjusted, open_range)
            and open_angle is not None
            and abs(angle) < abs(open_angle)
        ):
            open_range = adjusted
            open_angle = angle

    vector_x = float(front_attraction)
    vector_y = 0.0
    for adjusted, angle in zip(adjusted_ranges, angles):
        if adjusted >= max_range:
            continue
        weight = 1.0 - (adjusted / max_range)
        if weight <= 0.0:
            continue
        vector_x -= weight * math.cos(angle)
        vector_y -= weight * math.sin(angle)

    return PotentialFieldDecision(
        avoid_needed=obstacle_count > int(threshold),
        command=_twist_from_field_vector(
            vector_x,
            vector_y,
            max_translational_velocity=max_translational_velocity,
            max_rotational_velocity=max_rotational_velocity,
        ),
        min_range=min_valid_range(visible_ranges),
        open_angle=open_angle,
        open_range=open_range,
        obstacle_count=obstacle_count,
    )


def apply_potential_field_safety(command: Twist2D, decision: PotentialFieldDecision | None) -> Twist2D:
    if decision is None or not decision.avoid_needed:
        return command
    if abs(command.linear_x) <= 1e-3:
        return Twist2D(0.0, command.angular_z)
    if decision.min_range is not None and decision.min_range < 0.20 and command.linear_x > 0.0:
        return Twist2D(min(0.0, decision.command.linear_x), decision.command.angular_z)
    if (
        decision.min_range is not None
        and decision.min_range >= 0.22
        and command.linear_x > 0.0
    ):
        crawl_speed = min(abs(command.linear_x), 0.06)
        if decision.command.linear_x > 0.0:
            crawl_speed = min(crawl_speed, decision.command.linear_x)
        else:
            crawl_speed = max(0.0, crawl_speed)
        if crawl_speed > 0.0:
            return Twist2D(crawl_speed, decision.command.angular_z)
    return decision.command


def apply_soft_potential_field_safety(
    command: Twist2D,
    decision: PotentialFieldDecision | None,
    blend: float = 0.4,
    max_angular_delta: float = 0.55,
    min_forward_scale: float = 0.35,
    critical_stop_distance: float = 0.10,
) -> Twist2D:
    if decision is None or not decision.avoid_needed:
        return command

    blend = max(0.0, min(1.0, float(blend)))
    max_angular_delta = abs(float(max_angular_delta))
    min_forward_scale = max(0.0, min(1.0, float(min_forward_scale)))
    critical_stop_distance = max(0.0, float(critical_stop_distance))

    if (
        critical_stop_distance > 0.0
        and decision.min_range is not None
        and decision.min_range < critical_stop_distance
        and command.linear_x > 0.0
    ):
        return Twist2D(min(0.0, decision.command.linear_x), decision.command.angular_z)

    avoid_linear = decision.command.linear_x
    if command.linear_x > 1e-3:
        minimum_forward = command.linear_x * min_forward_scale
        avoid_linear = max(minimum_forward, min(command.linear_x, avoid_linear))
    elif command.linear_x < -1e-3:
        avoid_linear = min(0.0, max(command.linear_x, avoid_linear))

    linear = command.linear_x + blend * (avoid_linear - command.linear_x)
    angular_delta = decision.command.angular_z - command.angular_z
    angular_delta = max(-max_angular_delta, min(max_angular_delta, angular_delta))
    angular = command.angular_z + blend * angular_delta
    return Twist2D(float(linear), float(angular))


def should_bypass_transport_safety(
    mode: str,
    active_task_type: int,
    box_task_type: int,
    bypass_enabled: bool,
) -> bool:
    if not (mode == MODE_TASK_CONTROL and int(active_task_type) == int(box_task_type)):
        return False
    return bool(bypass_enabled)


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
) -> Twist2D:
    if mode in (MODE_STOP, MODE_FAILED):
        return ZERO_TWIST
    if mode == MODE_NAV_FORMATION:
        return deform_cmd if _fresh(last_deform_time, now, command_timeout) else ZERO_TWIST
    if mode == MODE_TASK_CONTROL:
        return task_cmd if _fresh(last_task_time, now, command_timeout) else ZERO_TWIST
    return ZERO_TWIST
