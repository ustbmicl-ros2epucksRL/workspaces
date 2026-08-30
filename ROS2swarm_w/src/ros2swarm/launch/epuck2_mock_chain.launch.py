#!/usr/bin/env python3
"""Run the physical-style ROS graph with no attached hardware.

The mock deliberately keeps the real deployment topology: one policy and one
driver namespace per robot, with one shared overhead-tracking/global-info
publisher.  It validates ROS contracts before hardware exists.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    package_dir = get_package_share_directory("ros2swarm")
    mock_tracking = os.path.join(
        package_dir, "config", "e_puck", "real_robot", "mock_tracking.yaml"
    )
    physical_launch = os.path.join(package_dir, "epuck2_physical.launch.py")
    pattern = LaunchConfiguration("pattern").perform(context)
    robot_count = int(LaunchConfiguration("robot_count").perform(context))
    if robot_count < 1:
        raise RuntimeError("robot_count must be positive")

    robot_keys = [f"robot_{index}" for index in range(robot_count)]
    initial_poses = [
        (-0.60, -0.60, 0.0),
        (0.60, -0.60, 1.57079632679),
        (0.0, 0.60, 3.14159265359),
    ]
    while len(initial_poses) < robot_count:
        index = len(initial_poses)
        initial_poses.append(
            (-0.75 + 0.25 * ((index - 3) % 7),
             0.75 - 0.25 * ((index - 3) // 7),
             0.0)
        )

    navigation_targets = []
    for index in range(robot_count):
        navigation_targets.extend([
            0.80 - 0.40 * (index % 2),
            0.80 - 0.40 * (index // 2),
            0.0,
        ])
    object_poses = navigation_targets + [
        0.0, 0.0, 0.0,       # flocking_target
        0.30, 0.0, 0.0,      # transport_box
        0.60, 0.0, 0.0,      # target_zone
    ]
    actions = [
        Node(
            package="ros2swarm",
            executable="fake_marker_tracker",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "robot_keys": robot_keys,
                "object_keys": [
                    *[f"navigation_target_{index}" for index in range(robot_count)],
                    "flocking_target",
                    "transport_box",
                    "target_zone",
                ],
                "object_poses": object_poses,
            }],
        ),
        Node(
            package="ros2swarm",
            executable="marker_state_fuser",
            output="screen",
            parameters=[mock_tracking, {"use_sim_time": False}],
        ),
        Node(
            package="ros2swarm",
            executable="global_info_bridge",
            output="screen",
            parameters=[{"use_sim_time": False}],
        ),
    ]
    for robot_id, (x, y, yaw) in enumerate(initial_poses[:robot_count]):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(physical_launch),
                launch_arguments={
                    "robot_id": str(robot_id),
                    "pattern": pattern,
                    "mock_hardware": "true",
                    "start_driver": "false",
                    "initial_x": str(x),
                    "initial_y": str(y),
                    "initial_yaw": str(yaw),
                }.items(),
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("pattern", default_value="navigation_pattern"),
        DeclareLaunchArgument("robot_count", default_value="3"),
        OpaqueFunction(function=_launch_setup),
    ])
