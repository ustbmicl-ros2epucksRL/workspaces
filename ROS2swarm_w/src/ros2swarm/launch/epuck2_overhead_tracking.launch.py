#!/usr/bin/env python3
"""Launch overhead detection and the global-state compatibility bridge."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_dir = get_package_share_directory("ros2swarm")
    default_config = os.path.join(
        package_dir, "config", "e_puck", "real_robot", "marker_tracking.yaml"
    )
    config = LaunchConfiguration("config")
    start_detector = LaunchConfiguration("start_detector")
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("start_detector", default_value="true"),
        Node(
            package="ros2swarm",
            executable="fiducial_detector",
            name="fiducial_detector",
            output="screen",
            parameters=[config, {"use_sim_time": False}],
            condition=IfCondition(start_detector),
        ),
        Node(
            package="ros2swarm",
            executable="marker_state_fuser",
            name="marker_state_fuser",
            output="screen",
            parameters=[config, {"use_sim_time": False}],
        ),
        Node(
            package="ros2swarm",
            executable="global_info_bridge",
            name="global_info_bridge",
            output="screen",
            parameters=[{"use_sim_time": False}],
        ),
    ])
