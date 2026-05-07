#!/usr/bin/env python3
"""
RoboSAR Webots simulation launch file.

Starts Webots with robots + controller nodes + bridge adapter + map server.
This launch file stays on the ROS 2 side only; ROS 1 nodes and ros1_bridge are
started separately by the wrapper script.
"""
import math
import os
import random

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_webots.environment_generator import (
    generate_random_positions_with_obstacles,
    generate_world_file,
    parse_world_file_objects,
)
from launch_webots.robot_nodes import create_burger_nodes, create_epuck_nodes


def _resolve_world_template_path(package_share: str, webots_world: str) -> str:
    world_path = os.path.join(package_share, 'worlds', webots_world)
    if not os.path.isfile(world_path):
        raise FileNotFoundError('Webots world template not found: %s' % world_path)
    return world_path


def _to_bool(value: str) -> bool:
    return str(value).strip().lower() in ('1', 'true', 'yes', 'on')


def _resolve_robot_type(raw_robot: str) -> str:
    if raw_robot.startswith('burger'):
        return 'burger'
    if raw_robot.startswith('e_puck'):
        return 'e_puck'
    raise ValueError('Unsupported robot type: %s' % raw_robot)


def _resolve_static_map_path(package_share: str, static_map: str) -> str:
    if not static_map:
        return ''
    if os.path.isabs(static_map):
        return static_map
    return os.path.join(package_share, 'maps', static_map)


def _build_robot_actions(number_robots: int, robot_type: str):
    node_factory = create_epuck_nodes if robot_type == 'e_puck' else create_burger_nodes
    actions = []
    for index in range(number_robots):
        actions.extend(node_factory(index, use_sim_time=True))
    return actions


def _default_spawn_yaws(number_robots: int, robot_type: str):
    default_yaw = math.pi if robot_type == 'e_puck' else 0.0
    return [default_yaw] * number_robots


def _build_launch(context, *args, **kwargs):
    package_share = get_package_share_directory('launch_webots')

    webots_world = LaunchConfiguration('webots_world').perform(context)
    number_robots = int(LaunchConfiguration('number_robots').perform(context))
    raw_robot = LaunchConfiguration('robot').perform(context)
    sensor_type = LaunchConfiguration('sensor_type').perform(context)
    random_seed_raw = LaunchConfiguration('random_seed').perform(context)
    x_min = float(LaunchConfiguration('x_min').perform(context))
    x_max = float(LaunchConfiguration('x_max').perform(context))
    y_min = float(LaunchConfiguration('y_min').perform(context))
    y_max = float(LaunchConfiguration('y_max').perform(context))
    min_distance = float(LaunchConfiguration('min_distance').perform(context))
    z_height = float(LaunchConfiguration('z_height').perform(context))
    static_map = LaunchConfiguration('static_map').perform(context)
    publish_static_map = _to_bool(
        LaunchConfiguration('publish_static_map').perform(context)
    )
    rviz = _to_bool(LaunchConfiguration('rviz').perform(context))
    apply_spawn_offset = _to_bool(
        LaunchConfiguration('apply_spawn_offset').perform(context)
    )
    normalize_scan = _to_bool(
        LaunchConfiguration('normalize_scan').perform(context)
    )
    
    if number_robots < 1:
        raise ValueError('number_robots must be >= 1')

    robot_type = _resolve_robot_type(raw_robot)
    static_map_path = _resolve_static_map_path(package_share, static_map)

    if random_seed_raw:
        random.seed(int(random_seed_raw))

    print('=' * 55)
    print('  RoboSAR Webots Simulation')
    print('=' * 55)
    print('  world template  |', webots_world)
    print('  num robots      |', number_robots)
    print('  robot type      |', robot_type)
    print('  sensor type     |', sensor_type)
    print('  static map      |', static_map_path or '(none)')
    print('  publish map     |', publish_static_map)
    print('  area            | X[%.1f, %.1f] Y[%.1f, %.1f]' % (
        x_min, x_max, y_min, y_max))
    print('  apply offset    |', apply_spawn_offset)
    print('  normalize scan  |', normalize_scan)
    print('=' * 55)

    worlds_dir = os.path.join(package_share, 'worlds')
    template_path = _resolve_world_template_path(package_share, webots_world)

    existing_objects = parse_world_file_objects(template_path)
    occupied = [(x, y, radius) for x, y, radius, _ in existing_objects]    
    positions = generate_random_positions_with_obstacles(
        number_robots,
        x_min,
        x_max,
        y_min,
        y_max,
        min_distance,
        None,
        occupied,
    )

    for index, (x_pos, y_pos) in enumerate(positions):
        print('  robot_%d -> (%.3f, %.3f)' % (index, x_pos, y_pos))

    generated_path = os.path.join(worlds_dir, 'generated_robosar.wbt')
    generate_world_file(
        template_path,
        generated_path,
        robot_type,
        number_robots,
        positions,
        z_height,
        {},
    )

    actions = []

    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(package_share, 'launch', 'start_webots.launch.py')
            ),
            launch_arguments={'world_path': generated_path}.items(),
        )
    )

    actions.extend(_build_robot_actions(number_robots, robot_type))

    spawn_x_list = [float(pos[0]) for pos in positions]
    spawn_y_list = [float(pos[1]) for pos in positions]
    spawn_yaw_list = _default_spawn_yaws(number_robots, robot_type)

    actions.append(
        Node(
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
                'apply_spawn_offset': apply_spawn_offset,
                'normalize_scan': normalize_scan,
            }],
        )
    )

    if publish_static_map and static_map_path and os.path.isfile(static_map_path):
        actions.append(
            Node(
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
        )
        actions.append(
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_map',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_server'],
                }],
            )
        )
        print('  [map_server] publishing %s on /sim_map' % static_map_path)
    elif static_map_path and not publish_static_map:
        print('  [map_server] disabled on ROS2 side; ROS1 side remains authoritative')
    elif static_map:
        print('  [WARN] static_map file not found: %s' % static_map_path)

    if rviz:
        rviz_config = os.path.join(package_share, 'rviz', 'robosar_multi_robot.rviz')
        actions.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('webots_world', default_value='command_world.wbt'),
        DeclareLaunchArgument('number_robots', default_value='3'),
        DeclareLaunchArgument('robot', default_value='e_puck'),
        DeclareLaunchArgument('sensor_type', default_value='lidar'),
        DeclareLaunchArgument('random_seed', default_value=''),
        DeclareLaunchArgument('x_min', default_value='-1.0'),
        DeclareLaunchArgument('x_max', default_value='-0.5'),
        DeclareLaunchArgument('y_min', default_value='-8.0'),
        DeclareLaunchArgument('y_max', default_value='-7.0'),
        DeclareLaunchArgument('min_distance', default_value='0.15'),
        DeclareLaunchArgument('z_height', default_value='0.0'),
        DeclareLaunchArgument('static_map', default_value=''),
        DeclareLaunchArgument('publish_static_map', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('apply_spawn_offset', default_value='true'),
        DeclareLaunchArgument('normalize_scan', default_value='true'),
        OpaqueFunction(function=_build_launch),
    ])
