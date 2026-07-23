"""Small, dependency-light geometry helpers used by real-robot adapters."""

from __future__ import annotations

import math
from typing import Iterable

import numpy as np


def wrap_to_pi(angle: float) -> float:
    """Wrap an angle to ``[-pi, pi]``."""
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(quaternion) -> float:
    """Return planar yaw from a ROS quaternion-like object."""
    x = float(quaternion.x)
    y = float(quaternion.y)
    z = float(quaternion.z)
    w = float(quaternion.w)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def quaternion_from_yaw(yaw: float):
    """Return a ``geometry_msgs.msg.Quaternion`` for planar yaw."""
    from geometry_msgs.msg import Quaternion

    msg = Quaternion()
    msg.z = math.sin(float(yaw) / 2.0)
    msg.w = math.cos(float(yaw) / 2.0)
    return msg


def transform_planar_pose(
    x: float,
    y: float,
    yaw: float,
    transform: Iterable[float],
) -> tuple[float, float, float]:
    """
    Apply a rigid 3-value or affine 6-value planar transform.

    Three values are ``[tx, ty, tyaw]``.  Six values are the row-major affine
    matrix ``[a, b, tx, c, d, ty]`` and can represent the reflection commonly
    required when camera optical ``+y`` points down in the image.
    """
    values = [float(value) for value in transform]
    if len(values) == 3:
        tx, ty, transform_yaw = values
        a = math.cos(transform_yaw)
        b = -math.sin(transform_yaw)
        c = math.sin(transform_yaw)
        d = math.cos(transform_yaw)
    elif len(values) == 6:
        a, b, tx, c, d, ty = values
    else:
        raise ValueError("planar transform must contain 3 or 6 values")
    world_x = a * float(x) + b * float(y) + tx
    world_y = c * float(x) + d * float(y) + ty
    direction_x = math.cos(float(yaw))
    direction_y = math.sin(float(yaw))
    world_direction_x = a * direction_x + b * direction_y
    world_direction_y = c * direction_x + d * direction_y
    world_yaw = math.atan2(world_direction_y, world_direction_x)
    return world_x, world_y, wrap_to_pi(world_yaw)


def fit_planar_affine(
    camera_points: Iterable[Iterable[float]],
    world_points: Iterable[Iterable[float]],
) -> tuple[list[float], float]:
    """Fit ``[a,b,tx,c,d,ty]`` from camera-plane to world points."""
    camera = np.asarray(list(camera_points), dtype=np.float64)
    world = np.asarray(list(world_points), dtype=np.float64)
    if (
        camera.ndim != 2
        or world.ndim != 2
        or camera.shape[1:] != (2,)
        or world.shape != camera.shape
        or camera.shape[0] < 3
    ):
        raise ValueError("at least three matching planar point pairs required")
    design = np.column_stack([camera, np.ones(camera.shape[0])])
    if np.linalg.matrix_rank(design) < 3:
        raise ValueError("camera calibration points are collinear")
    coefficients, _, _, _ = np.linalg.lstsq(design, world, rcond=None)
    predicted = design @ coefficients
    rmse = float(np.sqrt(np.mean(np.sum((predicted - world) ** 2, axis=1))))
    transform = [
        coefficients[0, 0],
        coefficients[1, 0],
        coefficients[2, 0],
        coefficients[0, 1],
        coefficients[1, 1],
        coefficients[2, 1],
    ]
    return [float(value) for value in transform], rmse


def sample_scan_ranges(
    ranges: Iterable[float],
    *,
    angle_min: float,
    angle_increment: float,
    n_rays: int,
    max_range: float,
    sensor_yaw: float = 0.0,
) -> np.ndarray:
    """
    Sample a scan at base-link angles ``0..2pi``.

    ``sensor_yaw`` is the sensor frame's yaw relative to ``base_link``.
    Invalid readings are treated as no return at ``max_range``.  The function
    accepts a plain sequence so it can be tested without a ROS graph.
    """
    n_rays = int(n_rays)
    max_range = float(max_range)
    if n_rays <= 0:
        raise ValueError("n_rays must be positive")
    if max_range <= 0.0:
        raise ValueError("max_range must be positive")

    scan = np.asarray(list(ranges), dtype=np.float32)
    if scan.size == 0:
        return np.full(n_rays, max_range, dtype=np.float32)
    scan = np.where(np.isfinite(scan), scan, max_range)
    scan = np.clip(scan, 0.0, max_range)

    if abs(float(angle_increment)) < 1e-12:
        sector_width = max(1, scan.size // n_rays)
        output = []
        for ray_index in range(n_rays):
            index = min(
                ray_index * sector_width + sector_width // 2,
                scan.size - 1,
            )
            output.append(scan[index])
        return np.asarray(output, dtype=np.float32)

    desired_angles = 2.0 * math.pi * np.arange(n_rays) / n_rays
    desired_sensor_angles = desired_angles - float(sensor_yaw)
    scan_angles = (
        float(angle_min) + float(angle_increment) * np.arange(scan.size)
    )
    best_indices = []
    for desired in desired_sensor_angles:
        # Choose the nearest equivalent angle, which also handles scans that
        # start at -pi or use a negative increment.
        errors = np.abs(
            np.arctan2(
                np.sin(scan_angles - desired),
                np.cos(scan_angles - desired),
            )
        )
        best_indices.append(int(np.argmin(errors)))
    return scan[np.asarray(best_indices, dtype=np.int64)].astype(np.float32)


def scan_has_blocking_obstacle(
    ranges: Iterable[float],
    angles: Iterable[float],
    *,
    stop_distance: float,
    linear: float,
    angular: float,
    front_cone: float = math.radians(75.0),
) -> bool:
    """Return whether a scan contains an obstacle in the commanded corridor."""
    if stop_distance <= 0.0:
        return False
    ranges_array = np.asarray(list(ranges), dtype=np.float32)
    angles_array = np.asarray(list(angles), dtype=np.float32)
    if ranges_array.size == 0 or angles_array.size != ranges_array.size:
        return False
    valid = np.isfinite(ranges_array) & (ranges_array > 0.0)
    if not np.any(valid):
        return False
    moving_angle = 0.0 if abs(linear) > 1e-6 else (
        math.pi / 2.0 if angular >= 0.0 else -math.pi / 2.0
    )
    if linear < -1e-6:
        moving_angle = math.pi
    errors = np.abs(
        np.arctan2(
            np.sin(angles_array - moving_angle),
            np.cos(angles_array - moving_angle),
        )
    )
    corridor = errors <= float(front_cone)
    blocked = valid & corridor & (ranges_array <= float(stop_distance))
    return bool(np.any(blocked))


def mask_expected_contact_returns(
    ranges: Iterable[float],
    angles: Iterable[float],
    *,
    expected_bearing: float,
    expected_range: float,
    bearing_half_width: float,
    range_tolerance: float,
) -> tuple[list[float], bool]:
    """Mask returns that agree with a tracked contact object's pose."""
    ranges_array = np.asarray(list(ranges), dtype=np.float32)
    angles_array = np.asarray(list(angles), dtype=np.float32)
    if ranges_array.size == 0 or ranges_array.size != angles_array.size:
        return ranges_array.tolist(), False

    bearing_error = np.abs(
        np.arctan2(
            np.sin(angles_array - float(expected_bearing)),
            np.cos(angles_array - float(expected_bearing)),
        )
    )
    valid = np.isfinite(ranges_array) & (ranges_array > 0.0)
    expected_error = np.abs(
        ranges_array - max(0.0, float(expected_range))
    )
    contact = (
        valid
        & (bearing_error <= max(0.0, float(bearing_half_width)))
        & (expected_error <= max(0.0, float(range_tolerance)))
    )
    masked = ranges_array.copy()
    masked[contact] = np.inf
    return masked.tolist(), bool(np.any(contact))
