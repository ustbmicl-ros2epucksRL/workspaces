"""Transport observation and action adapters for VMAS-trained policies."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np


@dataclass(frozen=True)
class TransportObservation:
    """VMAS-compatible transport observation plus useful diagnostics."""

    values: np.ndarray
    robot_yaw: float
    distance_to_goal: float
    robot_position: tuple[float, float]
    box_position: tuple[float, float]
    target_position: tuple[float, float]
    on_goal: float


@dataclass(frozen=True)
class DiffDriveCommand:
    """Diff-drive command converted from a VMAS planar action."""

    linear: float
    angular: float
    action_x: float
    action_y: float
    action_norm: float
    heading_error: float
    body_x: float
    body_y: float


def build_transport_observation(
    global_info: dict[str, Any],
    *,
    robot_name: str,
    box_def_name: str,
    target_def_name: str,
    world_scale: float,
    velocity_scale: float | None,
    goal_threshold: float,
) -> TransportObservation:
    """Build the 11D observation used by VMAS ``transport.py``."""
    robots = global_info.get("robots", {})
    objects = global_info.get("objects", {})

    robot_info = robots.get(robot_name)
    if robot_info is None:
        raise KeyError(f"Robot {robot_name!r} not present in /global_info")

    box_info = objects.get(box_def_name)
    if box_info is None:
        raise KeyError(
            f"Transport box {box_def_name!r} not present in /global_info"
        )

    target_info = objects.get(target_def_name)
    if target_info is None:
        raise KeyError(
            f"Target {target_def_name!r} not present in /global_info"
        )

    pos_scale = (
        float(world_scale)
        if abs(float(world_scale)) > 1e-9
        else 1.0
    )
    vel_scale = (
        pos_scale
        if velocity_scale is None or float(velocity_scale) <= 0.0
        else float(velocity_scale)
    )

    robot_x = float(robot_info["position"]["x"])
    robot_y = float(robot_info["position"]["y"])
    robot_vx = float(robot_info["velocity"]["x"])
    robot_vy = float(robot_info["velocity"]["y"])
    robot_yaw = yaw_from_orientation(robot_info.get("orientation"))

    box_x = float(box_info["position"]["x"])
    box_y = float(box_info["position"]["y"])
    box_vx = float(box_info["velocity"]["x"])
    box_vy = float(box_info["velocity"]["y"])

    target_x = float(target_info["position"]["x"])
    target_y = float(target_info["position"]["y"])

    box_to_target_x = box_x - target_x
    box_to_target_y = box_y - target_y
    robot_to_box_x = box_x - robot_x
    robot_to_box_y = box_y - robot_y
    distance_to_goal = math.hypot(box_to_target_x, box_to_target_y)
    on_goal = 1.0 if distance_to_goal < float(goal_threshold) else 0.0

    values = np.asarray(
        [
            robot_x / pos_scale,
            robot_y / pos_scale,
            robot_vx / vel_scale,
            robot_vy / vel_scale,
            box_to_target_x / pos_scale,
            box_to_target_y / pos_scale,
            robot_to_box_x / pos_scale,
            robot_to_box_y / pos_scale,
            box_vx / vel_scale,
            box_vy / vel_scale,
            on_goal,
        ],
        dtype=np.float32,
    )

    return TransportObservation(
        values=values,
        robot_yaw=robot_yaw,
        distance_to_goal=distance_to_goal,
        robot_position=(robot_x, robot_y),
        box_position=(box_x, box_y),
        target_position=(target_x, target_y),
        on_goal=on_goal,
    )


def vmas_action_to_diff_drive(
    action: np.ndarray | list[float],
    *,
    robot_yaw: float,
    max_linear_velocity: float,
    max_angular_velocity: float,
    turn_gain: float,
    min_action_norm: float,
    allow_reverse: bool,
    mapping: str = "heading",
) -> DiffDriveCommand:
    """Map a normalized 2D VMAS action vector to a differential-drive twist."""
    flat = np.asarray(action, dtype=np.float32).reshape(-1)
    if flat.size < 2:
        raise ValueError(f"Expected at least 2 action values, got {flat.size}")

    action_x = float(np.clip(flat[0], -1.0, 1.0))
    action_y = float(np.clip(flat[1], -1.0, 1.0))
    action_norm = min(1.0, math.hypot(action_x, action_y))
    if action_norm < float(min_action_norm):
        return DiffDriveCommand(
            0.0,
            0.0,
            action_x,
            action_y,
            action_norm,
            0.0,
            0.0,
            0.0,
        )

    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    body_x = cos_yaw * action_x + sin_yaw * action_y
    body_y = -sin_yaw * action_x + cos_yaw * action_y
    mapping = str(mapping).strip().lower()

    if mapping == "direct":
        linear = action_x * max_linear_velocity
        angular = action_y * max_angular_velocity
        heading_error = math.atan2(action_y, action_x)
    elif mapping == "projection":
        linear_factor = body_x
        if not allow_reverse:
            linear_factor = max(0.0, linear_factor)
        linear = (
            float(np.clip(linear_factor, -1.0, 1.0))
            * max_linear_velocity
        )
        angular = (
            float(np.clip(turn_gain * body_y, -1.0, 1.0))
            * max_angular_velocity
        )
        heading_error = math.atan2(body_y, body_x)
    else:
        desired_heading = math.atan2(action_y, action_x)
        heading_error = wrap_to_pi(desired_heading - robot_yaw)
        linear_factor = action_norm * math.cos(heading_error)
        if not allow_reverse:
            linear_factor = max(0.0, linear_factor)
        linear = (
            float(np.clip(linear_factor, -1.0, 1.0))
            * max_linear_velocity
        )
        angular = float(
            np.clip(
                turn_gain * heading_error,
                -max_angular_velocity,
                max_angular_velocity,
            )
        )

    return DiffDriveCommand(
        linear=float(linear),
        angular=float(angular),
        action_x=action_x,
        action_y=action_y,
        action_norm=action_norm,
        heading_error=float(heading_error),
        body_x=float(body_x),
        body_y=float(body_y),
    )


def yaw_from_orientation(orientation: Any) -> float:
    """Extract yaw from the row-major 3x3 Webots orientation matrix."""
    if orientation is None:
        return 0.0
    values = list(orientation)
    if len(values) < 4:
        return 0.0
    return math.atan2(float(values[3]), float(values[0]))


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi]``."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi
