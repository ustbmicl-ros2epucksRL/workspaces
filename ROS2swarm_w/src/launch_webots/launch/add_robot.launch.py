#!/usr/bin/env python3
"""
向已运行的 Webots 仿真中动态添加机器人并启动行为模式

使用场景：
  1. 先通过 create_enviroment.launch.py 启动仿真
  2. 再用本文件追加机器人（异构群体 / 热插拔）

该入口依赖 create_enviroment.launch.py 启动的 GlobalInfoSupervisor，
以获取 /global_info 并避开运行中的机器人和任务物体。

流程：
  对每个机器人：
  1. 通过 add_bot_node 调用 Ros2Supervisor/spawn_node_from_string 服务动态生成
  2. 创建 WebotsController + ros2_control 节点（robot_nodes.py）
  3. 启动行为模式（bringup_patterns.launch.py）
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_webots.robot_nodes import create_epuck_nodes, create_burger_nodes


SPAWN_SETTLE_TIME = 0.5


def generate_launch_description():
    """向已运行的仿真中添加机器人并启动行为模式"""

    launch_file_dir = get_package_share_directory('launch_webots')
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')
    launch_bringup_dir = get_package_share_directory('ros2swarm')
    map_file = os.path.join(launch_file_dir, 'maps', 'default.yaml')

    # 默认参数
    start_index = 0
    number_robots = 1
    pattern = 'drive_pattern'
    log_level = 'info'
    robot = 'e_puck'
    sensor_type = 'lidar'
    x_start = 0.0
    x_dist = 0.0
    y_start = -1.0
    y_dist = 0.4
    random_spawn = True
    driving_swarm = 'False'

    for arg in sys.argv:
        if arg.startswith("start_index:="):
            start_index = int(arg.split(":=")[1])
        elif arg.startswith("number_robots:="):
            number_robots = int(arg.split(":=")[1])
        elif arg.startswith("pattern:="):
            pattern = arg.split(":=")[1]
        elif arg.startswith("log_level:="):
            log_level = arg.split(":=")[1]
        elif arg.startswith("robot:="):
            robot = arg.split(":=")[1]
        elif arg.startswith("sensor_type:="):
            sensor_type = arg.split(":=")[1]
        elif arg.startswith("x_start:="):
            x_start = float(arg.split(":=")[1])
        elif arg.startswith("x_dist:="):
            x_dist = float(arg.split(":=")[1])
        elif arg.startswith("y_start:="):
            y_start = float(arg.split(":=")[1])
        elif arg.startswith("y_dist:="):
            y_dist = float(arg.split(":=")[1])
        elif arg.startswith("random_spawn:="):
            random_spawn = arg.split(":=")[1].strip().lower() in (
                '1', 'true', 'yes', 'on'
            )
        elif arg.startswith("driving_swarm:="):
            driving_swarm = arg.split(":=")[1]
            if driving_swarm == 'True':
                yaml_file = 'default.yaml'
                map_file = os.path.join(launch_file_dir, 'maps', yaml_file)

    # 确定机器人类型
    robot_type = robot
    if robot_type.startswith('burger'):
        robot_type = "burger"
    elif robot_type.startswith('waffle_pi'):
        robot_type = "waffle_pi"
    elif robot_type.startswith('e_puck'):
        robot_type = "e_puck"

    print("=" * 50)
    print("动态添加机器人到运行中的仿真")
    print("=" * 50)
    print(f"起始编号        | {start_index}")
    print(f"添加数量        | {number_robots}")
    print(f"机器人类型      | {robot_type}")
    print(f"行为模式        | {pattern}")
    print(f"传感器类型      | {sensor_type}")
    if random_spawn:
        print("位置/朝向       | 随机 (X[-1.0, 1.0], Y[-1.0, 1.0])")
    else:
        print(f"位置            | 起始({x_start}, {y_start}), 间距({x_dist}, {y_dist})")
    print("=" * 50)

    ld = LaunchDescription()

    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot_type)
    spawn_nodes = []

    for i in range(number_robots):
        num = i + start_index
        robot_name = f"robot_{num}"
        x = x_start + i * x_dist
        y = y_start + i * y_dist

        # 1. 通过 Ros2Supervisor 服务动态生成机器人
        spawn_arguments = [
            '--robot_name', robot_name,
            '--robot_namespace', robot_name,
            '-z', '0.0',
            '--type_of_robot', robot,
        ]
        if not random_spawn:
            spawn_arguments.extend(['-x', str(x), '-y', str(y)])

        spawn_node = Node(
            package='launch_webots',
            executable='add_bot_node',
            namespace=robot_name,
            name=f'spawner_{num}',
            output='screen',
            arguments=spawn_arguments,
        )
        spawn_nodes.append(spawn_node)

        # 2. 创建 WebotsController + ros2_control 节点
        if robot_type == "e_puck":
            robot_nodes = create_epuck_nodes(num, use_sim_time=True)
        elif robot_type == "burger":
            robot_nodes = create_burger_nodes(num, use_sim_time=True)
        else:
            raise ValueError(f"不支持的机器人类型: {robot_type}")

        for node in robot_nodes:
            ld.add_action(node)

        # 3. driving swarm 配置
        if driving_swarm == 'True':
            rviz_config_file = LaunchConfiguration(
                'rviz_config_file',
                default=os.path.join(
                    get_package_share_directory('driving_swarm_bringup'), 'rviz', 'custom.rviz'
                )
            )
            rviz = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')
                ),
                launch_arguments={
                    'namespace': robot_name,
                    'use_namespace': 'true',
                    'use_sim_time': 'true',
                    'rviz_config': rviz_config_file
                }.items()
            )
            ld.add_action(rviz)

            tf_exchange_dir = get_package_share_directory('tf_exchange')
            tf_exchange = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(tf_exchange_dir, 'launch', 'tf_exchange.launch.py')
                ),
                launch_arguments={
                    'namespace': robot_name,
                    'robot_name': robot_name,
                    'base_frame': 'base_link',
                }.items()
            )
            ld.add_action(tf_exchange)

        # 4. 启动行为模式
        pattern_launch_file_name = pattern + '.launch.py'
        pattern_path = None
        for root, dirs, files in os.walk(launch_pattern_dir):
            for name in files:
                if name == pattern_launch_file_name:
                    pattern_path = os.path.abspath(os.path.join(root, name))

        if pattern_path is None:
            raise FileNotFoundError(f"未找到行为模式: {pattern_launch_file_name}")

        launch_patterns = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_bringup_dir, 'bringup_patterns.launch.py')
            ),
            launch_arguments={
                'robot': robot,
                'robot_type': robot_type,
                'sensor_type': sensor_type,
                'robot_namespace': robot_name,
                'pattern': pattern_path,
                'config_dir': config_dir,
                'map': map_file,
                'driving_swarm': driving_swarm,
                'params_file': os.path.join(
                    get_package_share_directory('ros2swarm'),
                    'param',
                    f'nav2_params_{robot_type}_namespaced.yaml'
                )
            }.items()
        )
        ld.add_action(launch_patterns)

    if spawn_nodes:
        ld.add_action(spawn_nodes[0])
        for previous_spawn, next_spawn in zip(spawn_nodes, spawn_nodes[1:]):
            ld.add_action(
                RegisterEventHandler(
                    event_handler=OnProcessExit(
                        target_action=previous_spawn,
                        on_exit=[
                            TimerAction(
                                period=SPAWN_SETTLE_TIME,
                                actions=[next_spawn],
                            )
                        ],
                    )
                )
            )

    return ld
