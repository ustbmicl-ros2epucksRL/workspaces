"""Observation helpers for VMAS planar navigation and flocking actors."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import numpy as np

from ros2swarm.vmas_transport_adapter import yaw_from_orientation


@dataclass(frozen=True)
class PlanarTaskObservation:
    """VMAS-compatible observation plus diagnostics."""

    values: np.ndarray
    robot_yaw: float
    distance_to_target: float
    robot_position: tuple[float, float]
    target_position: tuple[float, float]
    lidar_min: float | None


def build_navigation_observation(
    global_info: dict[str, Any],
    *,
    robot_name: str,
    target_def_name: str,
    lidar_ranges: np.ndarray | None,
    world_scale: float,
    velocity_scale: float | None,
    lidar_range: float,
) -> PlanarTaskObservation:
    """Build the 18D observation used by VMAS ``navigation.py``."""
    return _build_goal_observation(
        global_info,
        robot_name=robot_name,
        target_def_name=target_def_name,
        lidar_ranges=lidar_ranges,
        world_scale=world_scale,
        velocity_scale=velocity_scale,
        lidar_range=lidar_range,
        lidar_mode="navigation_free_space",
    )


def build_flocking_observation(
    global_info: dict[str, Any],
    *,
    robot_name: str,
    target_def_name: str,
    lidar_ranges: np.ndarray | None,
    world_scale: float,
    velocity_scale: float | None,
    lidar_range: float,
) -> PlanarTaskObservation:
    """Build the 18D observation used by VMAS ``flocking.py``.

    The exported flocking actor was trained with ``static_at_origin=True``.
    In that scenario ``agent.state.pos`` and ``agent.state.pos - target.pos``
    are the same vector.  Webots can place the target elsewhere, so feed the
    actor target-centric positions while keeping diagnostics in world frame.
    """
    return _build_goal_observation(
        global_info,
        robot_name=robot_name,
        target_def_name=target_def_name,
        lidar_ranges=lidar_ranges,
        world_scale=world_scale,
        velocity_scale=velocity_scale,
        lidar_range=lidar_range,
        lidar_mode="raw_distance",
        target_centric_position=True,
    )


def sample_lidar_msg(msg: Any, n_rays: int, max_range: float) -> np.ndarray:
    """Sample a ROS LaserScan into VMAS ray order: 0..2pi from robot +x."""
    ranges = np.asarray(msg.ranges, dtype=np.float32)
    if ranges.size == 0:
        return np.full(int(n_rays), float(max_range), dtype=np.float32)

    max_range = float(max_range)
    ranges = np.where(np.isfinite(ranges), ranges, max_range)
    ranges = np.clip(ranges, 0.0, max_range)

    angle_min = float(getattr(msg, "angle_min", 0.0))
    angle_increment = float(getattr(msg, "angle_increment", 0.0))
    if abs(angle_increment) < 1e-12:
        return _sample_lidar_by_sector(ranges, n_rays)

    sampled = []
    for ray_index in range(int(n_rays)):
        desired = 2.0 * math.pi * ray_index / int(n_rays)
        desired = _wrap_into_scan_interval(desired, angle_min, angle_increment,
                                           ranges.size)
        index = int(round((desired - angle_min) / angle_increment))
        index = max(0, min(index, ranges.size - 1))
        sampled.append(ranges[index])
    return np.asarray(sampled, dtype=np.float32)


def synthesize_flocking_lidar(
    global_info: dict[str, Any],
    *,
    robot_name: str,
    n_rays: int,
    max_range: float,
    obstacle_prefix: str = "obstacle_",
    obstacle_radius: float = 0.1,
) -> np.ndarray:
    """Build VMAS-style flocking lidar from known obstacle objects.

    VMAS flocking trains with a lidar filter that excludes all agents, so the
    target and the other robots are intentionally absent from these rays.
    """
    n_rays = int(n_rays)
    if n_rays <= 0:
        return np.zeros(0, dtype=np.float32)

    max_range = float(max_range)
    lidar = np.full(n_rays, max_range, dtype=np.float32)
    robots = global_info.get("robots", {})
    objects = global_info.get("objects", {})

    robot_info = robots.get(robot_name)
    if robot_info is None:
        raise KeyError(f"Robot {robot_name!r} not present in /global_info")

    robot_x = float(robot_info["position"]["x"])
    robot_y = float(robot_info["position"]["y"])
    robot_yaw = yaw_from_orientation(robot_info.get("orientation"))

    for object_name, object_info in objects.items():
        if not str(object_name).startswith(obstacle_prefix):
            continue
        position = object_info.get("position", {})
        obstacle_x = float(position["x"])
        obstacle_y = float(position["y"])
        _cast_circle_into_lidar(
            lidar,
            obstacle_x - robot_x,
            obstacle_y - robot_y,
            robot_yaw,
            float(obstacle_radius),
            max_range,
        )

    return lidar

def _build_goal_observation(
    global_info: dict[str, Any],
    *,
    robot_name: str,
    target_def_name: str,
    lidar_ranges: np.ndarray | None,
    world_scale: float,
    velocity_scale: float | None,
    lidar_range: float,
    lidar_mode: str,
    target_centric_position: bool = False,
) -> PlanarTaskObservation:
    robots = global_info.get("robots", {})
    objects = global_info.get("objects", {})

    robot_info = robots.get(robot_name)
    if robot_info is None:
        raise KeyError(f"Robot {robot_name!r} not present in /global_info")

    target_info = objects.get(target_def_name)
    if target_info is None:
        raise KeyError(
            f"Target {target_def_name!r} not present in /global_info"
        )

    if lidar_ranges is None:
        raise ValueError("Lidar data is required for this actor.")

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

    target_x = float(target_info["position"]["x"])
    target_y = float(target_info["position"]["y"])
    rel_x = robot_x - target_x
    rel_y = robot_y - target_y
    distance_to_target = math.hypot(rel_x, rel_y)
    obs_robot_x = rel_x if target_centric_position else robot_x
    obs_robot_y = rel_y if target_centric_position else robot_y

    lidar_ranges = np.asarray(lidar_ranges, dtype=np.float32)
    lidar_ranges = np.clip(lidar_ranges, 0.0, float(lidar_range))
    if lidar_mode == "navigation_free_space":
        lidar_values = float(lidar_range) - lidar_ranges
    elif lidar_mode == "raw_distance":
        lidar_values = lidar_ranges
    else:
        raise ValueError(f"Unsupported lidar_mode: {lidar_mode}")

    values = np.concatenate(
        [
            np.asarray(
                [
                    obs_robot_x / pos_scale,
                    obs_robot_y / pos_scale,
                    robot_vx / vel_scale,
                    robot_vy / vel_scale,
                    rel_x / pos_scale,
                    rel_y / pos_scale,
                ],
                dtype=np.float32,
            ),
            lidar_values.astype(np.float32),
        ],
        axis=0,
    )

    return PlanarTaskObservation(
        values=values,
        robot_yaw=robot_yaw,
        distance_to_target=distance_to_target,
        robot_position=(robot_x, robot_y),
        target_position=(target_x, target_y),
        lidar_min=float(np.min(lidar_ranges)) if lidar_ranges.size else None,
    )


def _sample_lidar_by_sector(ranges: np.ndarray, n_rays: int) -> np.ndarray:
    sampled = []
    beams_per_sector = max(1, len(ranges) // int(n_rays))
    for ray_index in range(int(n_rays)):
        start_idx = ray_index * beams_per_sector
        mid_idx = min(start_idx + beams_per_sector // 2, len(ranges) - 1)
        sampled.append(ranges[mid_idx])
    return np.asarray(sampled, dtype=np.float32)


def _cast_circle_into_lidar(
    lidar: np.ndarray,
    rel_x: float,
    rel_y: float,
    robot_yaw: float,
    radius: float,
    max_range: float,
) -> None:
    cos_yaw = math.cos(robot_yaw)
    sin_yaw = math.sin(robot_yaw)
    body_x = cos_yaw * rel_x + sin_yaw * rel_y
    body_y = -sin_yaw * rel_x + cos_yaw * rel_y
    center_distance = math.hypot(body_x, body_y)
    radius = max(0.0, float(radius))

    if center_distance <= 1e-9:
        lidar[:] = 0.0
        return
    if center_distance - radius > max_range:
        return

    center_angle = math.atan2(body_y, body_x)
    for ray_index in range(lidar.size):
        ray_angle = 2.0 * math.pi * ray_index / lidar.size
        angle_error = _wrap_to_pi(ray_angle - center_angle)
        projection = center_distance * math.cos(angle_error)
        if projection <= 0.0:
            continue
        perpendicular = abs(center_distance * math.sin(angle_error))
        if perpendicular > radius:
            continue
        surface_distance = projection - math.sqrt(
            max(0.0, radius * radius - perpendicular * perpendicular)
        )
        if 0.0 <= surface_distance <= max_range:
            lidar[ray_index] = min(lidar[ray_index], surface_distance)


def _wrap_to_pi(angle: float) -> float:
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def _wrap_into_scan_interval(
    angle: float,
    angle_min: float,
    angle_increment: float,
    range_count: int,
) -> float:
    angle_max = angle_min + angle_increment * (range_count - 1)
    low = min(angle_min, angle_max)
    high = max(angle_min, angle_max)
    wrapped = angle
    while wrapped < low:
        wrapped += 2.0 * math.pi
    while wrapped > high:
        wrapped -= 2.0 * math.pi
    return wrapped
