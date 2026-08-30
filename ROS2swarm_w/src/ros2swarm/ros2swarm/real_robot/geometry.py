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
