#!/usr/bin/env python3
# Copyright 2026
# Licensed under the Apache License, Version 2.0

import math

from communication_interfaces.msg import TrackedEntity
from geometry_msgs.msg import Twist

from ros2swarm.real_robot.geometry import quaternion_from_yaw
from ros2swarm.real_robot.safety import SafetyLimits, clamp_twist_to_limits
from ros2swarm.real_robot.safety_supervisor import SafetySupervisor


def _limits():
    return SafetyLimits(
        max_linear_velocity=0.12,
        max_angular_velocity=1.0,
        wheel_separation=0.05685,
        max_wheel_linear_velocity=0.1507,
        max_linear_acceleration=0.5,
        max_angular_acceleration=2.0,
    )


def test_safety_limits_velocity_and_acceleration():
    requested = Twist()
    requested.linear.x = 1.0
    requested.angular.z = 5.0

    output = clamp_twist_to_limits(
        requested,
        _limits(),
        previous=Twist(),
        dt=0.1,
    )

    assert output.linear.x == 0.05
    assert output.angular.z == 0.2


def test_safety_joint_wheel_limit_preserves_curvature():
    requested = Twist()
    requested.linear.x = 0.12
    requested.angular.z = 1.0
    limits = _limits()

    output = clamp_twist_to_limits(requested, limits)
    left = output.linear.x - output.angular.z * limits.wheel_separation / 2.0
    right = output.linear.x + output.angular.z * limits.wheel_separation / 2.0

    assert max(abs(left), abs(right)) <= (
        limits.max_wheel_linear_velocity + 1e-12
    )


def test_transport_contact_requires_aligned_tracked_box_and_limits_speed():
    supervisor = SafetySupervisor.__new__(SafetySupervisor)
    supervisor.allow_transport_contact = True
    supervisor.task_mode = "transport"
    supervisor.last_command = Twist()
    supervisor.last_command.linear.x = 0.10
    supervisor.transport_contact_distance = 0.22
    supervisor.transport_box_radius = 0.06
    supervisor.transport_contact_bearing = math.radians(25.0)
    supervisor.max_transport_contact_linear_velocity = 0.05
    supervisor.robot_state = TrackedEntity()
    supervisor.robot_state.pose.pose.orientation = quaternion_from_yaw(0.0)
    supervisor.transport_box_state = TrackedEntity()
    supervisor.transport_box_state.pose.pose.position.x = 0.16

    bearing, expected_range = supervisor._transport_contact_geometry()
    limited = supervisor._limit_transport_contact_speed(
        supervisor.last_command
    )

    assert bearing == 0.0
    assert math.isclose(expected_range, 0.10)
    assert limited.linear.x == 0.05
    assert supervisor.last_command.linear.x == 0.10

    supervisor.transport_box_state.pose.pose.position.y = 0.20
    assert supervisor._transport_contact_geometry() is None
