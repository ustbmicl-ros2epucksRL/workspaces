#!/usr/bin/env python3
# Copyright 2026
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from unittest.mock import Mock

import numpy as np

from ros2swarm.hardware_protection_layer import limit_twist_to_wheel_speed
from ros2swarm.movement_pattern.basic.roundflocking_pattern_neat import (
    RoundFlockingPattern,
    sample_lidar_for_neat,
)


def test_physical_lidar_ranges_are_clipped_to_neat_training_range():
    epuck_scan = np.full(24, 12.0, dtype=np.float32)
    burger_scan = np.full(24, 3.5, dtype=np.float32)
    epuck_scan[3] = burger_scan[3] = 0.08

    epuck_sample = sample_lidar_for_neat(epuck_scan, 12, 0.2)
    burger_sample = sample_lidar_for_neat(burger_scan, 12, 0.2)

    np.testing.assert_allclose(epuck_sample, burger_sample)
    assert epuck_sample[1] == np.float32(0.08)
    assert np.max(epuck_sample) == np.float32(0.2)


def test_invalid_lidar_values_mean_no_detection_at_training_limit():
    sample = sample_lidar_for_neat(
        [np.inf, np.nan] * 12,
        n_lidar_rays=12,
        lidar_input_max_range=0.2,
    )

    np.testing.assert_allclose(sample, np.full(12, 0.2, dtype=np.float32))


def test_flocking_stays_stopped_until_observation_is_ready():
    pattern = RoundFlockingPattern.__new__(RoundFlockingPattern)
    pattern.network = Mock()
    pattern._get_observation = Mock(return_value=None)
    pattern.command_publisher = Mock()

    pattern.control_callback()

    pattern.network.activate.assert_not_called()
    stop = pattern.command_publisher.publish.call_args.args[0]
    assert stop.linear.x == 0.0
    assert stop.angular.z == 0.0


def test_wheel_speed_limit_preserves_curvature_and_joint_limit():
    linear, angular = limit_twist_to_wheel_speed(
        linear_velocity=0.22,
        angular_velocity=2.84,
        wheel_separation=0.17792,
        max_wheel_linear_velocity=0.22,
    )

    left = linear - angular * 0.17792 / 2.0
    right = linear + angular * 0.17792 / 2.0
    assert max(abs(left), abs(right)) <= 0.22 + 1e-12
    assert angular / linear == np.float64(2.84 / 0.22)


def test_disabled_wheel_speed_limit_keeps_legacy_platform_command():
    assert limit_twist_to_wheel_speed(
        0.12,
        1.0,
        0.0,
        0.0,
    ) == (0.12, 1.0)
