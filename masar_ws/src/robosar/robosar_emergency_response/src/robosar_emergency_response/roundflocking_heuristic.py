"""VMAS-style heuristic controller for roundflocking tasks."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Sequence

from .checkpoint_controller import global_action_to_twist
from .cmd_mux import Twist2D


@dataclass(frozen=True)
class RoundflockingHeuristicConfig:
    circle_radius: float = 0.30
    normal_offset: float = 0.10
    object_visible_range: float = 0.10
    object_avoid_distance: float = 0.10
    gain: float = 10.0
    u_range: float = 1.0
    speed: float = 1.0


def roundflocking_heuristic_twist_from_observation(
    observation: Sequence[float],
    robot_yaw: float,
    max_linear_velocity: float,
    max_angular_velocity: float,
    config: RoundflockingHeuristicConfig | None = None,
) -> Twist2D:
    cfg = config or RoundflockingHeuristicConfig()
    global_action = roundflocking_heuristic_global_action_from_observation(
        observation,
        config=cfg,
    )
    return global_action_to_twist(
        global_action,
        robot_yaw=robot_yaw,
        max_linear_velocity=max_linear_velocity,
        max_angular_velocity=max_angular_velocity,
        speed_scale=cfg.speed,
    )


def roundflocking_heuristic_global_action_from_observation(
    observation: Sequence[float],
    config: RoundflockingHeuristicConfig | None = None,
) -> tuple[float, float]:
    """Return the VMAS flocking HeuristicPolicy action in global coordinates."""
    if len(observation) < 6:
        raise ValueError("roundflocking observation must have at least 6 values")

    cfg = config or RoundflockingHeuristicConfig()
    current_pos = (float(observation[0]), float(observation[1]))
    lidar = [float(value) for value in observation[6:]]

    if lidar and min(lidar) < cfg.object_visible_range:
        closest_index = min(range(len(lidar)), key=lambda index: lidar[index])
        object_dir = closest_index / float(len(lidar)) * 2.0 * math.pi
        object_vec = (math.cos(object_dir), math.sin(object_dir))
        des_pos = (
            current_pos[0] - object_vec[0] * cfg.object_avoid_distance,
            current_pos[1] - object_vec[1] * cfg.object_avoid_distance,
        )
    else:
        des_pos = _desired_circle_motion_position(current_pos, cfg)

    return (
        _clamp((des_pos[0] - current_pos[0]) * cfg.gain, cfg.u_range),
        _clamp((des_pos[1] - current_pos[1]) * cfg.gain, cfg.u_range),
    )


def _desired_circle_motion_position(
    current_pos: tuple[float, float],
    config: RoundflockingHeuristicConfig,
) -> tuple[float, float]:
    norm = math.hypot(current_pos[0], current_pos[1])
    if norm <= 1e-9:
        closest_point = (config.circle_radius, 0.0)
    else:
        closest_point = (
            current_pos[0] / norm * config.circle_radius,
            current_pos[1] / norm * config.circle_radius,
        )

    normal = (closest_point[1], -closest_point[0])
    normal_norm = math.hypot(normal[0], normal[1])
    if normal_norm <= 1e-9:
        return closest_point
    return (
        closest_point[0] + normal[0] / normal_norm * config.normal_offset,
        closest_point[1] + normal[1] / normal_norm * config.normal_offset,
    )


def _clamp(value: float, limit: float) -> float:
    limit = abs(float(limit))
    if limit <= 0.0:
        return 0.0
    return max(-limit, min(limit, float(value)))
