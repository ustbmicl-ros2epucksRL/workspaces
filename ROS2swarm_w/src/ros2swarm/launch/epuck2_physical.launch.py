#!/usr/bin/env python3
"""Launch one physical e-puck2 with the same control chain as Webots."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(context, name):
    return LaunchConfiguration(name).perform(context).lower() == "true"


def _launch_setup(context):
    package_dir = get_package_share_directory("ros2swarm")
    config_dir = os.path.join(package_dir, "config", "e_puck")
    robot_id = int(LaunchConfiguration("robot_id").perform(context))
    robot_name = f"robot_{robot_id}"
    pattern = LaunchConfiguration("pattern").perform(context)
    is_transport = "transport_pattern" in pattern

    pattern_config = os.path.join(
        config_dir,
        "movement_pattern",
        "basic",
        f"{pattern}.yaml",
    )
    if not os.path.exists(pattern_config):
        raise RuntimeError(f"Pattern config does not exist: {pattern_config}")

    mock_hardware = _as_bool(context, "mock_hardware")
    start_driver = _as_bool(context, "start_driver")
    start_lidar = _as_bool(context, "start_lidar")
    start_lidar_layer = _as_bool(context, "start_lidar_layer")
    start_policy = _as_bool(context, "start_policy")
    start_protection = _as_bool(context, "start_hardware_protection")

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
    lidar_frame = f"{robot_name}/laser_frame"

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
                    "initial_x": float(LaunchConfiguration("initial_x").perform(context)),
                    "initial_y": float(LaunchConfiguration("initial_y").perform(context)),
                    "initial_yaw": float(LaunchConfiguration("initial_yaw").perform(context)),
                }],
            ),
            Node(
                package="ros2swarm",
                executable="synthetic_scan_publisher",
                namespace=robot_name,
                output="screen",
                parameters=[{
                    "use_sim_time": False,
                    "topic": "scan",
                    "frame_id": lidar_frame,
                }],
            ),
        ])
    elif start_driver:
        actions.append(Node(
            package=driver_package,
            executable=driver_executable,
            namespace=robot_name,
            output="screen",
            # The e-puck driver's scan is IR/ToF. Keep /scan for the added
            # 360-degree lidar, matching the Webots topic contract.
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
                    "topic_name": "scan",
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
                    "--yaw", LaunchConfiguration("lidar_yaw"),
                    "--pitch", "0.0",
                    "--roll", "0.0",
                    "--frame-id", f"{robot_name}/base_link",
                    "--child-frame-id", lidar_frame,
                ],
            ),
        ])

    # Reuse the same LaserScan -> RangeData layer as Webots. The policy also
    # consumes /scan directly and performs its existing 12-ray sampling.
    if start_lidar_layer:
        actions.append(Node(
            package="ros2swarm",
            executable="lidar_layer",
            namespace=robot_name,
            output="screen",
            parameters=[
                os.path.join(config_dir, "sensor_specification.yaml"),
                {"use_sim_time": False},
            ],
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
                    # Keep the policy and velocity mapping used by Webots.
                    # This flag only disables simulation fallbacks and enables
                    # stale-input handling for physical ROS communication.
                    "real_robot_mode": True,
                },
            ],
        ))

    # bringup_patterns.launch.py bypasses this node for Transport, whose
    # validated config publishes directly to cmd_vel. Preserve that behavior.
    if start_protection and not is_transport:
        actions.append(Node(
            package="ros2swarm",
            executable="hardware_protection_layer",
            namespace=robot_name,
            output="screen",
            parameters=[
                os.path.join(config_dir, "hardware_protection_layer.yaml"),
                {"use_sim_time": False},
            ],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="0"),
        DeclareLaunchArgument("pattern", default_value="navigation_pattern"),
        DeclareLaunchArgument("mock_hardware", default_value="false"),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument("start_driver", default_value="false"),
        DeclareLaunchArgument("start_policy", default_value="true"),
        DeclareLaunchArgument("start_lidar_layer", default_value="true"),
        DeclareLaunchArgument(
            "start_hardware_protection",
            default_value="true",
        ),
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
        DeclareLaunchArgument("lidar_yaw", default_value="0.0"),
        OpaqueFunction(function=_launch_setup),
    ])
