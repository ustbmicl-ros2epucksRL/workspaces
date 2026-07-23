#!/usr/bin/env python3
"""Run the complete physical-style ROS graph with no attached hardware."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("ros2swarm")
    mock_tracking = os.path.join(
        package_dir, "config", "e_puck", "real_robot", "mock_tracking.yaml"
    )
    physical_launch = os.path.join(package_dir, "epuck2_physical.launch.py")
    pattern = LaunchConfiguration("pattern")
    return LaunchDescription([
        DeclareLaunchArgument("pattern", default_value="navigation_pattern"),
        Node(
            package="ros2swarm",
            executable="fake_marker_tracker",
            namespace="robot_0",
            output="screen",
            parameters=[{"use_sim_time": False}],
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(physical_launch),
            launch_arguments={
                "robot_id": "0",
                "pattern": pattern,
                "mock_hardware": "true",
                "start_driver": "false",
            }.items(),
        ),
    ])
