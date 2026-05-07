#!/usr/bin/env python3
"""
RoboSAR Webots simulation launch file.

Starts Webots with robots + controller nodes + bridge adapter + map_server.
Supports both E-puck and TurtleBot3 Burger, same as create_enviroment_fast.
Does NOT start ROS1 nodes or ros1_bridge (those are separate).

Usage:
  ros2 launch launch_webots robosar_webots_sim.launch.py \
      webots_world:=command_world.wbt number_robots:=3 robot:=e_puck \
      static_map:=tepper.yaml
"""
import os
import sys
import random
import math

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch_webots.robot_nodes import create_epuck_nodes, create_burger_nodes
from launch_webots.environment_generator import (
    parse_world_file_objects,
    generate_random_positions_with_obstacles,
    generate_world_file,
)


def generate_launch_description():
    launch_file_dir = get_package_share_directory('launch_webots')

    ld = LaunchDescription()

    # --- defaults ---
    webots_world = 'command_world.wbt'
    number_robots = 3
    robot = 'e_puck'
    sensor_type = 'lidar'
    random_seed = None
    x_min, x_max = -1.0, -0.5
    y_min, y_max = -8.0, -7.0
    min_distance = 0.15
    z_height = 0.0
    static_map = ''
    rviz = 'false'

    for arg in sys.argv:
        if arg.startswith("webots_world:="):
            webots_world = arg.split(":=")[1]
        elif arg.startswith("number_robots:="):
            number_robots = int(arg.split(":=")[1])
        elif arg.startswith("robot:="):
            robot = arg.split(":=")[1]
        elif arg.startswith("sensor_type:="):
            sensor_type = arg.split(":=")[1]
        elif arg.startswith("random_seed:="):
            random_seed = int(arg.split(":=")[1])
        elif arg.startswith("x_min:="):
            x_min = float(arg.split(":=")[1])
        elif arg.startswith("x_max:="):
            x_max = float(arg.split(":=")[1])
        elif arg.startswith("y_min:="):
            y_min = float(arg.split(":=")[1])
        elif arg.startswith("y_max:="):
            y_max = float(arg.split(":=")[1])
        elif arg.startswith("min_distance:="):
            min_distance = float(arg.split(":=")[1])
        elif arg.startswith("z_height:="):
            z_height = float(arg.split(":=")[1])
        elif arg.startswith("static_map:="):
            static_map = arg.split(":=")[1]
        elif arg.startswith("rviz:="):
            rviz = arg.split(":=")[1]

    # resolve robot_type
    robot_type = robot
    if robot_type.startswith('burger'):
        robot_type = 'burger'
    elif robot_type.startswith('e_puck'):
        robot_type = 'e_puck'

    if random_seed is not None:
        random.seed(random_seed)

    # resolve static map path
    static_map_path = ''
    if static_map:
        if os.path.isabs(static_map):
            static_map_path = static_map
        else:
            static_map_path = os.path.join(launch_file_dir, 'maps', static_map)

    print("=" * 55)
    print("  RoboSAR Webots Simulation")
    print("=" * 55)
    print("  world template  |", webots_world)
    print("  num robots      |", number_robots)
    print("  robot type      |", robot_type)
    print("  sensor type     |", sensor_type)
    print("  static map      |", static_map_path or '(none)')
    print("  area            | X[%.1f, %.1f] Y[%.1f, %.1f]" % (
        x_min, x_max, y_min, y_max))
    print("=" * 55)

    # --- generate world file with robots ---
    worlds_dir = os.path.join(launch_file_dir, 'worlds')
    template_path = os.path.join(worlds_dir, webots_world)

    existing_objs = parse_world_file_objects(template_path)
    occupied = [(x, y, r) for x, y, r, _ in existing_objs]

    positions = generate_random_positions_with_obstacles(
        number_robots, x_min, x_max, y_min, y_max,
        min_distance, None, occupied)

    for i, (x, y) in enumerate(positions):
        print("  robot_%d -> (%.3f, %.3f)" % (i, x, y))

    generated_path = os.path.join(worlds_dir, 'generated_robosar.wbt')
    generate_world_file(
        template_path, generated_path,
        robot_type, number_robots, positions, z_height, {})

    # --- 1. start Webots ---
    webots_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'launch', 'start_webots.launch.py')),
        launch_arguments={'world_path': generated_path}.items(),
    )
    ld.add_action(webots_start)

    # --- 2. robot controller nodes (standard robot_N namespace) ---
    for i in range(number_robots):
        if robot_type == 'e_puck':
            nodes = create_epuck_nodes(i, use_sim_time=True)
        elif robot_type == 'burger':
            nodes = create_burger_nodes(i, use_sim_time=True)
        else:
            raise ValueError("Unsupported robot type: %s" % robot_type)
        for action in nodes:
            ld.add_action(action)

    # --- 3. bridge adapter (robot_N <-> agent_N topic mapping) ---
    # Webots e-puck odom starts at (0,0,0); we pass spawn positions so the
    # adapter can offset odom to world coordinates (matching Gazebo convention).
    spawn_x_list = [float(pos[0]) for pos in positions]
    spawn_y_list = [float(pos[1]) for pos in positions]
    if robot_type == 'e_puck':
        spawn_yaw_list = [math.pi] * number_robots
    else:
        spawn_yaw_list = [0.0] * number_robots
    adapter_node = Node(
        package='launch_webots',
        executable='robosar_bridge_adapter',
        name='robosar_bridge_adapter',
        output='screen',
        parameters=[{
            'num_robots': number_robots,
            'use_sim_time': True,
            'spawn_x': spawn_x_list,
            'spawn_y': spawn_y_list,
            'spawn_yaw': spawn_yaw_list,
        }],
    )
    ld.add_action(adapter_node)

    # --- 5. static map server (nav2_map_server, publishes /sim_map) ---
    if static_map_path and os.path.isfile(static_map_path):
        map_server_node = Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': static_map_path,
                'topic_name': '/sim_map',
                'frame_id': 'map',
            }],
        )
        ld.add_action(map_server_node)

        # nav2_map_server needs lifecycle activation
        lifecycle_manager = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server'],
            }],
        )
        ld.add_action(lifecycle_manager)
        print("  [map_server] publishing %s on /sim_map" % static_map_path)
    elif static_map:
        print("  [WARN] static_map file not found: %s" % static_map_path)

    # --- 6. RViz2 ---
    if rviz.lower() in ('true', '1', 'yes'):
        rviz_config = os.path.join(launch_file_dir, 'rviz', 'robosar_multi_robot.rviz')
        rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        )
        ld.add_action(rviz_node)

    return ld
