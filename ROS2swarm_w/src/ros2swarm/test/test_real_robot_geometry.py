#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0

import math

import numpy as np

from ros2swarm.real_robot.geometry import (
    fit_planar_affine,
    mask_expected_contact_returns,
    sample_scan_ranges,
    scan_has_blocking_obstacle,
    transform_planar_pose,
)


def test_scan_sampling_uses_angles_not_array_origin():
    beam_count = 360
    ranges = np.full(beam_count, 5.0, dtype=np.float32)
    angle_min = -math.pi
    increment = 2.0 * math.pi / beam_count
    front_index = round((0.0 - angle_min) / increment)
    ranges[front_index] = 0.1

    sampled = sample_scan_ranges(
        ranges,
        angle_min=angle_min,
        angle_increment=increment,
        n_rays=12,
        max_range=0.35,
    )

    assert sampled.shape == (12,)
    assert sampled[0] == np.float32(0.1)
    np.testing.assert_allclose(sampled[1:], np.full(11, 0.35))


def test_scan_sampling_handles_clockwise_scan_and_invalid_values():
    ranges = [float("inf")] * 360
    angle_min = math.pi
    increment = -2.0 * math.pi / 360
    front_index = round((0.0 - angle_min) / increment)
    ranges[front_index] = 0.08

    sampled = sample_scan_ranges(
        ranges,
        angle_min=angle_min,
        angle_increment=increment,
        n_rays=12,
        max_range=0.2,
    )

    assert sampled[0] == np.float32(0.08)
    assert max(sampled) == np.float32(0.2)


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


def test_obstacle_check_uses_commanded_corridor():
    angles = [0.0, math.pi]
    ranges = [0.1, 0.1]

    assert scan_has_blocking_obstacle(
        ranges,
        angles,
        stop_distance=0.12,
        linear=0.1,
        angular=0.0,
    )
    assert scan_has_blocking_obstacle(
        ranges,
        angles,
        stop_distance=0.12,
        linear=-0.1,
        angular=0.0,
    )


def test_transport_contact_mask_only_removes_matching_box_return():
    ranges = [0.095, 0.08, 0.30]
    angles = [0.0, 0.5, -0.5]

    masked, contact = mask_expected_contact_returns(
        ranges,
        angles,
        expected_bearing=0.0,
        expected_range=0.10,
        bearing_half_width=0.15,
        range_tolerance=0.02,
    )

    assert contact
    assert math.isinf(masked[0])
    np.testing.assert_allclose(masked[1:], ranges[1:])
