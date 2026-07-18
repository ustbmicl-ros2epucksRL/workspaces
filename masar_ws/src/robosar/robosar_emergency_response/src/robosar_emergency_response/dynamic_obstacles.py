"""Dynamic obstacle helpers for response-group local-map navigation."""

from __future__ import annotations

import math
from typing import Iterable, Mapping, Sequence, Tuple


Point3 = Tuple[float, float, float]


def _xy_from_pose(pose) -> tuple[float, float] | None:
    if hasattr(pose, "position"):
        return float(pose.position.x), float(pose.position.y)
    if isinstance(pose, Sequence) and len(pose) >= 2:
        return float(pose[0]), float(pose[1])
    return None


def build_dynamic_obstacle_points(
    model_poses: Mapping[str, object],
    obstacle_prefixes: Iterable[str],
    ignored_names: Iterable[str],
    radius: float,
    z: float,
    samples_per_obstacle: int,
    excluded_prefixes: Iterable[str] = (),
) -> list[Point3]:
    """Sample circular point obstacles for selected Gazebo model names."""
    ignored = set(ignored_names)
    prefixes = tuple(prefix for prefix in obstacle_prefixes if prefix)
    excluded = tuple(prefix for prefix in excluded_prefixes if prefix)
    radius = max(0.0, float(radius))
    z = float(z)
    sample_count = max(0, int(samples_per_obstacle))
    points: list[Point3] = []

    for name in sorted(model_poses.keys()):
        if (
            name in ignored
            or any(name.startswith(prefix) for prefix in excluded)
            or not any(name.startswith(prefix) for prefix in prefixes)
        ):
            continue
        xy = _xy_from_pose(model_poses[name])
        if xy is None:
            continue
        x, y = xy
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        points.append((x, y, z))
        if radius <= 0.0 or sample_count <= 0:
            continue
        for index in range(sample_count):
            angle = 2.0 * math.pi * float(index) / float(sample_count)
            points.append((x + radius * math.cos(angle), y + radius * math.sin(angle), z))

    return points


def build_dynamic_obstacle_points_from_centers(
    centers: Iterable[Sequence[float]],
    radius: float,
    z: float,
    samples_per_obstacle: int,
) -> list[Point3]:
    """Sample circular point obstacles from already-resolved object centers."""
    radius = max(0.0, float(radius))
    z = float(z)
    sample_count = max(0, int(samples_per_obstacle))
    points: list[Point3] = []

    for center in centers:
        if len(center) < 2:
            continue
        x = float(center[0])
        y = float(center[1])
        if not math.isfinite(x) or not math.isfinite(y):
            continue
        points.append((x, y, z))
        if radius <= 0.0 or sample_count <= 0:
            continue
        for index in range(sample_count):
            angle = 2.0 * math.pi * float(index) / float(sample_count)
            points.append((x + radius * math.cos(angle), y + radius * math.sin(angle), z))

    return points


def build_occupancy_grid_points(
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    data: Sequence[int],
    occupied_threshold: int,
    z: float,
    stride: int = 1,
) -> list[Point3]:
    """Convert occupied OccupancyGrid cells into point obstacles."""
    width = int(width)
    height = int(height)
    stride = max(1, int(stride))
    resolution = float(resolution)
    origin_x = float(origin_x)
    origin_y = float(origin_y)
    occupied_threshold = int(occupied_threshold)
    z = float(z)
    points: list[Point3] = []

    for row in range(0, height, stride):
        for col in range(0, width, stride):
            index = row * width + col
            if index >= len(data) or int(data[index]) < occupied_threshold:
                continue
            x = origin_x + (float(col) + 0.5) * resolution
            y = origin_y + (float(row) + 0.5) * resolution
            points.append((x, y, z))

    return points


def filter_points_near_centers(
    points: Iterable[Point3],
    centers: Iterable[Sequence[float]],
    clear_radius: float,
) -> list[Point3]:
    """Remove obstacle points within clear_radius of known free centers."""
    radius = max(0.0, float(clear_radius))
    if radius <= 0.0:
        return list(points)

    clear_centers: list[tuple[float, float]] = []
    for center in centers:
        if len(center) < 2:
            continue
        x = float(center[0])
        y = float(center[1])
        if math.isfinite(x) and math.isfinite(y):
            clear_centers.append((x, y))
    if not clear_centers:
        return list(points)

    radius_sq = radius * radius
    filtered: list[Point3] = []
    for x, y, z in points:
        if any((x - cx) ** 2 + (y - cy) ** 2 <= radius_sq for cx, cy in clear_centers):
            continue
        filtered.append((x, y, z))
    return filtered


def filter_points_near_model_poses(
    points: Iterable[Point3],
    model_poses: Mapping[str, object],
    clear_prefixes: Iterable[str],
    clear_radius: float,
) -> list[Point3]:
    """Remove map obstacle points around live robot poses.

    SLAM maps can temporarily contain robot footprints. Feeding those stale
    cells into a local planner can make the current robot start "inside an
    obstacle", so the static map layer is cleared around live search/response
    robots before live dynamic obstacles are added back explicitly.
    """
    radius = max(0.0, float(clear_radius))
    if radius <= 0.0:
        return list(points)

    prefixes = tuple(prefix for prefix in clear_prefixes if prefix)
    if not prefixes:
        return list(points)

    clear_centers: list[tuple[float, float]] = []
    for name, pose in model_poses.items():
        if not any(name.startswith(prefix) for prefix in prefixes):
            continue
        xy = _xy_from_pose(pose)
        if xy is None:
            continue
        x, y = xy
        if math.isfinite(x) and math.isfinite(y):
            clear_centers.append((x, y))

    if not clear_centers:
        return list(points)

    radius_sq = radius * radius
    filtered: list[Point3] = []
    for x, y, z in points:
        if any((x - cx) ** 2 + (y - cy) ** 2 <= radius_sq for cx, cy in clear_centers):
            continue
        filtered.append((x, y, z))
    return filtered
