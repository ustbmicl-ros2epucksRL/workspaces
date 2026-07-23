#!/usr/bin/env python3
"""Launch one e-puck2 policy through the physical safety pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    package_dir = get_package_share_directory("ros2swarm")
    robot_id = int(LaunchConfiguration("robot_id").perform(context))
    robot_name = f"robot_{robot_id}"
    pattern = LaunchConfiguration("pattern").perform(context)
    mock_hardware = (
        LaunchConfiguration("mock_hardware").perform(context).lower()
        == "true"
    )
    start_driver = (
        LaunchConfiguration("start_driver").perform(context).lower()
        == "true"
    )
    start_lidar = (
        LaunchConfiguration("start_lidar").perform(context).lower()
        == "true"
    )
    start_policy = (
        LaunchConfiguration("start_policy").perform(context).lower()
        == "true"
    )
    start_scan_adapter = (
        LaunchConfiguration("start_scan_adapter").perform(context).lower()
        == "true"
    )
    start_safety = (
        LaunchConfiguration("start_safety").perform(context).lower()
        == "true"
    )
    driver_package = LaunchConfiguration("driver_package").perform(context)
    driver_executable = LaunchConfiguration("driver_executable").perform(
        context
    )
    lidar_package = LaunchConfiguration("lidar_package").perform(context)
    lidar_executable = LaunchConfiguration("lidar_executable").perform(context)
    lidar_serial_port = LaunchConfiguration("lidar_serial_port").perform(
        context
    )
    lidar_baudrate = int(
        LaunchConfiguration("lidar_baudrate").perform(context)
    )
    lidar_scan_mode = LaunchConfiguration("lidar_scan_mode").perform(context)
    lidar_sensor_yaw = float(
        LaunchConfiguration("lidar_sensor_yaw").perform(context)
    )
    lidar_frame = f"{robot_name}/laser_frame"
    config_dir = os.path.join(package_dir, "config", "e_puck")
    pattern_config = os.path.join(
        config_dir,
        "movement_pattern",
        "basic",
        f"{pattern}.yaml",
    )
    if not os.path.exists(pattern_config):
        raise RuntimeError(f"Pattern config does not exist: {pattern_config}")
    safety_config = os.path.join(config_dir, "real_robot", "safety.yaml")
    input_max_range = 0.35 if pattern == "navigation_pattern" else 0.20
    task_mode = pattern.replace("_pattern_neat", "").replace("_pattern", "")

    actions = []
    if mock_hardware:
        actions.extend([
            Node(
                package="ros2swarm",
                executable="fake_epuck_driver",
                namespace=robot_name,
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "odom_frame": f"{robot_name}/odom",
                    "base_frame": f"{robot_name}/base_link",
                }],
            ),
            Node(
                package="ros2swarm",
                executable="synthetic_scan_publisher",
                namespace=robot_name,
                output="screen",
                parameters=[{"use_sim_time": False}],
            ),
        ])
    elif start_driver:
        actions.append(Node(
            package=driver_package,
            executable=driver_executable,
            namespace=robot_name,
            output="screen",
            # The e-puck2 driver's scan is IR/ToF, not the added 360 Lidar.
            remappings=[("scan", "proximity_scan")],
            parameters=[{
                "use_sim_time": False,
                "command_timeout": 0.25,
                "odom_frame": f"{robot_name}/odom",
                "base_frame": f"{robot_name}/base_link",
                "proximity_laser_frame": (
                    f"{robot_name}/proximity_laser"
                ),
                "frame_prefix": f"{robot_name}/",
            }],
        ))

    if not mock_hardware and start_lidar:
        actions.extend([
            Node(
                package=lidar_package,
                executable=lidar_executable,
                namespace=robot_name,
                name="lidar_driver",
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "serial_port": lidar_serial_port,
                    "serial_baudrate": lidar_baudrate,
                    "frame_id": lidar_frame,
                    "topic_name": "scan_raw",
                    "scan_mode": lidar_scan_mode,
                    "inverted": False,
                    "angle_compensate": True,
                }],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=robot_name,
                name="lidar_static_tf",
                output="screen",
                arguments=[
                    "--x", LaunchConfiguration("lidar_x"),
                    "--y", LaunchConfiguration("lidar_y"),
                    "--z", LaunchConfiguration("lidar_z"),
                    "--yaw", LaunchConfiguration("lidar_sensor_yaw"),
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", f"{robot_name}/base_link",
                    "--child-frame-id", lidar_frame,
                ],
            ),
        ])

    if start_scan_adapter:
        actions.append(Node(
            package="ros2swarm",
            executable="policy_scan_adapter",
            namespace=robot_name,
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "input_topic": "scan_raw",
                "output_topic": "scan",
                "n_rays": 12,
                "input_max_range": input_max_range,
                "sensor_yaw": lidar_sensor_yaw,
                "output_frame": f"{robot_name}/base_link",
            }],
        ))
    if start_policy:
        actions.append(Node(
            package="ros2swarm",
            executable=pattern,
            namespace=robot_name,
            output="screen",
            parameters=[
                pattern_config,
                {
                    "use_sim_time": False,
                    "bypass_protection": False,
                    "real_robot_mode": True,
                },
            ],
        ))
    if start_safety:
        actions.append(Node(
            package="ros2swarm",
            executable="safety_supervisor",
            namespace=robot_name,
            output="screen",
            parameters=[
                safety_config,
                {
                    "use_sim_time": False,
                    "robot_name": robot_name,
                    "task_mode": task_mode,
                    "sensor_yaw": lidar_sensor_yaw,
                },
            ],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="0"),
        DeclareLaunchArgument("pattern", default_value="navigation_pattern"),
        DeclareLaunchArgument("mock_hardware", default_value="false"),
        DeclareLaunchArgument("start_driver", default_value="false"),
        DeclareLaunchArgument("start_policy", default_value="true"),
        DeclareLaunchArgument("start_scan_adapter", default_value="true"),
        DeclareLaunchArgument("start_safety", default_value="true"),
        DeclareLaunchArgument(
            "driver_package",
            default_value="epuck_ros2_driver",
        ),
        DeclareLaunchArgument("driver_executable", default_value="driver"),
        DeclareLaunchArgument("start_lidar", default_value="false"),
        DeclareLaunchArgument("lidar_package", default_value="rplidar_ros"),
        DeclareLaunchArgument(
            "lidar_executable",
            default_value="rplidar_composition",
        ),
        DeclareLaunchArgument(
            "lidar_serial_port",
            default_value="/dev/ttyUSB0",
        ),
        DeclareLaunchArgument("lidar_baudrate", default_value="115200"),
        DeclareLaunchArgument("lidar_scan_mode", default_value=""),
        DeclareLaunchArgument("lidar_x", default_value="0.0"),
        DeclareLaunchArgument("lidar_y", default_value="0.0"),
        DeclareLaunchArgument("lidar_z", default_value="0.08"),
        DeclareLaunchArgument("lidar_sensor_yaw", default_value="0.0"),
        OpaqueFunction(function=_launch_setup),
    ])
