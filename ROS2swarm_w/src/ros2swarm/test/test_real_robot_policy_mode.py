#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0

import time
from unittest.mock import Mock

import numpy as np

from ros2swarm.movement_pattern.basic.navigation_pattern import (
    NavigationPattern as NavigationPt,
)
from ros2swarm.movement_pattern.basic.navigation_pattern_neat import (
    NavigationPattern as NavigationNeat,
)
from ros2swarm.movement_pattern.basic.roundflocking_pattern import (
    RoundFlockingPattern as RoundFlockingPt,
)
from ros2swarm.movement_pattern.basic.roundflocking_pattern_neat import (
    RoundFlockingPattern as RoundFlockingNeat,
)
from ros2swarm.movement_pattern.basic.transport_pattern import (
    TransportPattern as TransportPt,
)
from ros2swarm.movement_pattern.basic.transport_pattern_neat import (
    TransportPattern as TransportNeat,
)


def _logger():
    return Mock(debug=Mock(), warn=Mock(), error=Mock(), info=Mock())


def test_neat_real_mode_never_synthesizes_missing_observations():
    transport = TransportNeat.__new__(TransportNeat)
    transport.global_info = None
    transport.real_robot_mode = True
    transport.network_input_dim = 11
    transport._distance_to_target = None
    transport.get_logger = Mock(return_value=_logger())

    navigation = NavigationNeat.__new__(NavigationNeat)
    navigation.global_info = None
    navigation.lidar_data = None
    navigation.real_robot_mode = True
    navigation.collisions = True
    navigation.network_input_dim = 18
    navigation.get_logger = Mock(return_value=_logger())

    assert transport._get_observation() is None
    assert navigation._get_observation() is None


def test_neat_simulation_mode_keeps_random_missing_data_fallback():
    pattern = TransportNeat.__new__(TransportNeat)
    pattern.global_info = None
    pattern.real_robot_mode = False
    pattern.network_input_dim = 11
    pattern._distance_to_target = None
    pattern.get_logger = Mock(return_value=_logger())

    observation = pattern._get_observation()

    assert len(observation) == 11


def test_pt_real_mode_stale_inputs_publish_stop_before_inference():
    pattern = RoundFlockingPt.__new__(RoundFlockingPt)
    pattern.actor_policy = Mock()
    pattern.global_info = {"robots": {}, "objects": {}}
    pattern.lidar_data = np.full(12, 0.2, dtype=np.float32)
    pattern.real_robot_mode = True
    pattern.data_timeout = 0.3
    pattern._global_info_received_at = time.monotonic() - 1.0
    pattern._lidar_received_at = time.monotonic()
    pattern._missing_data_warn_counts = {}
    pattern.command_publisher = Mock()
    pattern.get_logger = Mock(return_value=_logger())

    pattern.control_callback()

    pattern.actor_policy.act.assert_not_called()
    stop = pattern.command_publisher.publish.call_args.args[0]
    assert stop.linear.x == 0.0
    assert stop.angular.z == 0.0


def test_navigation_neat_real_mode_stops_at_goal():
    pattern = NavigationNeat.__new__(NavigationNeat)
    pattern.network = Mock()
    pattern._get_observation = Mock(return_value=[0.0] * 18)
    pattern.command_publisher = Mock()
    pattern.real_robot_mode = True
    pattern._goal_reached = False
    pattern._distance_to_target = 0.05
    pattern.goal_threshold = 0.1
    pattern.get_logger = Mock(return_value=_logger())

    pattern.control_callback()

    pattern.network.activate.assert_not_called()
    assert pattern._goal_reached
    stop = pattern.command_publisher.publish.call_args.args[0]
    assert stop.linear.x == 0.0
    assert stop.angular.z == 0.0


def test_recurrent_networks_reset_on_real_start_only():
    cases = [
        (TransportPt, "actor_policy", (1,)),
        (TransportNeat, "network", ()),
        (RoundFlockingPt, "actor_policy", (1,)),
        (RoundFlockingNeat, "network", ()),
        (NavigationPt, "actor_policy", (1,)),
        (NavigationNeat, "network", ()),
    ]
    for pattern_class, network_attribute, reset_args in cases:
        pattern = pattern_class.__new__(pattern_class)
        network = Mock()
        setattr(pattern, network_attribute, network)
        pattern.real_robot_mode = False
        pattern._goal_reached = True
        pattern._distance_to_target = 1.0
        pattern._started_near_goal = True

        pattern.on_swarm_start()
        network.reset.assert_not_called()

        pattern.real_robot_mode = True
        pattern.on_swarm_start()
        network.reset.assert_called_once_with(*reset_args)
