"""Small dynamic-obstacle safety helpers for Tepper search robots."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .cmd_mux import Twist2D


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


def closest_obstacle(
    agent_pose: AgentSafetyPose,
    obstacle_poses: Sequence[AgentSafetyPose],
) -> tuple[float, AgentSafetyPose | None]:
    closest_distance = float("inf")
    closest_pose = None
    for obstacle_pose in obstacle_poses:
        distance = math.hypot(obstacle_pose.x - agent_pose.x, obstacle_pose.y - agent_pose.y)
        if distance < closest_distance:
            closest_distance = distance
            closest_pose = obstacle_pose
    return closest_distance, closest_pose


def apply_dynamic_obstacle_pose_safety(
    command: Twist2D,
    agent_pose: AgentSafetyPose | None,
    obstacle_poses: Sequence[AgentSafetyPose],
    trigger_distance: float,
    stop_distance: float,
    max_rotational_velocity: float,
    reverse_velocity: float = 0.0,
) -> Twist2D:
    if agent_pose is None or trigger_distance <= 0.0 or not obstacle_poses:
        return command

    distance, closest_pose = closest_obstacle(agent_pose, obstacle_poses)
    if closest_pose is None or distance > trigger_distance:
        return command

    bearing = relative_bearing(agent_pose, closest_pose)

    speed = abs(float(max_rotational_velocity))
    if speed <= 0.0:
        return Twist2D(min(command.linear_x, 0.0), 0.0)

    closeness = 1.0 - max(0.0, min(distance, trigger_distance)) / trigger_distance
    turn_speed = speed * max(0.35, closeness)
    angular = -turn_speed if bearing >= 0.0 else turn_speed
    escape_speed = abs(float(reverse_velocity))
    stop_closeness = 1.0 - max(0.0, min(distance, stop_distance)) / max(stop_distance, 1e-6)
    escape_speed *= max(0.5, stop_closeness) if escape_speed > 0.0 else 0.0

    if abs(bearing) >= (0.75 * math.pi):
        linear = max(command.linear_x, escape_speed)
    elif abs(bearing) <= (0.5 * math.pi):
        linear = min(command.linear_x, 0.0)
        if distance <= stop_distance and escape_speed > 0.0:
            linear = -escape_speed
    else:
        linear = min(command.linear_x, 0.0)
    return Twist2D(float(linear), float(angular))
