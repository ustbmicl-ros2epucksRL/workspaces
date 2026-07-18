"""Pose-based hard-safety helpers for response robots."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Collection, Mapping

from .cmd_mux import RangeExclusion, Twist2D


@dataclass(frozen=True)
class AgentSafetyPose:
    x: float
    y: float
    yaw: float = 0.0


def _normalize_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def relative_bearing(from_pose: AgentSafetyPose, to_pose: AgentSafetyPose) -> float:
    return _normalize_angle(math.atan2(to_pose.y - from_pose.y, to_pose.x - from_pose.x) - from_pose.yaw)


def circle_range_exclusion(
    observer_pose: AgentSafetyPose,
    circle_pose: AgentSafetyPose,
    radius: float,
    range_padding: float = 0.03,
) -> RangeExclusion:
    return RangeExclusion(
        bearing=relative_bearing(observer_pose, circle_pose),
        center_distance=math.hypot(
            circle_pose.x - observer_pose.x,
            circle_pose.y - observer_pose.y,
        ),
        radius=max(0.0, float(radius)),
        range_padding=max(0.0, float(range_padding)),
    )


def apply_teammate_hard_safety(
    command: Twist2D,
    robot_name: str,
    robot_poses: Mapping[str, AgentSafetyPose],
    minimum_center_distance: float,
    allowed_controlled_contact_names: Collection[str] = (),
    controlled_contact_min_center_distance: float = 0.0,
) -> Twist2D:
    agent_pose = robot_poses.get(robot_name)
    if agent_pose is None or abs(command.linear_x) <= 1e-9:
        return command
    normal_distance_limit = max(0.0, float(minimum_center_distance))
    controlled_distance_limit = max(
        0.0, float(controlled_contact_min_center_distance),
    )
    controlled_names = set(allowed_controlled_contact_names)
    for name, teammate_pose in robot_poses.items():
        if name == robot_name:
            continue
        distance_limit = (
            controlled_distance_limit
            if name in controlled_names
            else normal_distance_limit
        )
        distance = math.hypot(
            teammate_pose.x - agent_pose.x,
            teammate_pose.y - agent_pose.y,
        )
        if distance >= distance_limit:
            continue
        bearing = relative_bearing(agent_pose, teammate_pose)
        if command.linear_x * math.cos(bearing) > 0.0:
            return Twist2D(0.0, command.angular_z)
    return command


def apply_external_priority_safety(
    command: Twist2D,
    robot_name: str,
    robot_poses: Mapping[str, AgentSafetyPose],
    protected_names: Collection[str],
    minimum_center_distance: float,
    reaction_horizon: float,
    maximum_other_speed: float,
) -> Twist2D:
    """Limit approach speed so a lower-priority mobile robot has time to yield.

    The normal task controller remains responsible for steering.  This final
    velocity-level barrier only consumes the available center-distance margin:
    over ``reaction_horizon`` it reserves enough distance for the other robot
    to move at the configured system speed bound.  It therefore scales or
    stops forward translation without inventing reverse motion or modifying
    the task controller's angular command.
    """
    agent_pose = robot_poses.get(robot_name)
    if agent_pose is None or command.linear_x <= 1e-9:
        return command

    horizon = max(1e-6, float(reaction_horizon))
    distance_limit = max(0.0, float(minimum_center_distance))
    other_speed = max(0.0, float(maximum_other_speed))
    protected = set(protected_names)
    limited_linear = float(command.linear_x)

    for name, other_pose in robot_poses.items():
        if name not in protected:
            continue
        dx = other_pose.x - agent_pose.x
        dy = other_pose.y - agent_pose.y
        distance = math.hypot(dx, dy)
        if distance <= 1e-9:
            limited_linear = 0.0
            continue

        bearing = relative_bearing(agent_pose, other_pose)
        forward_projection = math.cos(bearing)
        if forward_projection <= 1e-9:
            continue

        distance_margin = max(0.0, distance - distance_limit)
        allowed_closing_speed = max(0.0, distance_margin / horizon - other_speed)
        allowed_linear = allowed_closing_speed / forward_projection
        limited_linear = min(limited_linear, allowed_linear)

    return Twist2D(max(0.0, limited_linear), command.angular_z)
