#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0

import math

import numpy as np

from ros2swarm.real_robot.geometry import (
    fit_planar_affine,
    transform_planar_pose,
)


def test_affine_camera_transform_can_reflect_image_y_axis():
    x, y, yaw = transform_planar_pose(
        0.2,
        0.3,
        math.pi / 2.0,
        [1.0, 0.0, 1.0, 0.0, -1.0, 2.0],
    )

    assert x == 1.2
    assert y == 1.7
    assert math.isclose(yaw, -math.pi / 2.0)


def test_affine_calibration_recovers_world_transform():
    camera_points = [
        [0.0, 0.0],
        [1.0, 0.0],
        [0.0, 1.0],
        [1.0, 1.0],
    ]
    expected = [2.0, 0.5, 1.0, -0.25, -3.0, 2.0]
    world_points = [
        transform_planar_pose(x, y, 0.0, expected)[:2]
        for x, y in camera_points
    ]

    fitted, rmse = fit_planar_affine(camera_points, world_points)

    np.testing.assert_allclose(fitted, expected, atol=1e-12)
    assert rmse < 1e-12
