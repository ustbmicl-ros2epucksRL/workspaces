#!/usr/bin/env python3
"""
启动环境的launch文件
通过预生成包含机器人的wbt世界文件，避免使用Ros2Supervisor服务动态添加机器人的延迟
支持E-puck和TurtleBot3 Burger两种机器人
根据pattern名称自动配置任务环境：
  - transport_pattern: 运输任务（箱子+目标点）
  - roundflocking_pattern: 环绕任务（圆柱目标）
  - navigation_pattern: 导航任务（每个机器人有不同颜色的可进入目标点）
  - 其他pattern: 基础环境
num_obstacles参数可在任意模式下添加障碍物
"""
import os
import sys
import random

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import (
    IncludeLaunchDescription, DeclareLaunchArgument,
    RegisterEventHandler, EmitEvent
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 导入机器人节点创建函数
from launch_webots.robot_nodes import create_epuck_nodes, create_burger_nodes
from launch_webots.environment_generator import (
    ROUND_FLOCKING_TARGET_CLEARANCE,
    inset_bounds_for_clearance,
    parse_world_file_objects,
    generate_random_positions_with_obstacles,
    generate_grid_positions,
    navigation_grid_columns,
    generate_world_file,
    get_task_type_from_pattern,
    normalize_transport_object_shape,
)


def generate_launch_description():
    """创建环境，预先生成包含机器人的世界文件，然后启动Webots和行为节点"""

    launch_file_dir = os.path.join(get_package_share_directory('launch_webots'))
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')
    launch_bringup_dir = os.path.join(get_package_share_directory('ros2swarm'))
    robot_file = os.path.join(get_package_share_directory('ros2swarm'), 'param', 'ROS2swarm_sim.yaml')
    map_file = os.path.join(launch_file_dir, 'maps', 'default.yaml')

    ld = LaunchDescription()

    # 解析命令行参数
    webots_world = 'command_world.wbt'
    number_robots = 4
    total_robots = 4
    pattern = 'drive_pattern'
    log_level = 'info'
    robot = 'e_puck'
    sensor_type = 'ir'
    driving_swarm = 'False'
    logging = 'False'
    transport_object_shape = 'cube'
    webots_mode = 'realtime'
    webots_gui = 'true'
    navigation_target_layout = 'random'     #random or columns

    # 随机位置生成参数
    random_seed = None
    x_min = -1.5
    x_max = 1.5
    y_min = -1.5
    y_max = 1.5
    min_distance = 0.15
    z_height = 0.0

    # 障碍物参数（任意模式下可用）
    num_obstacles = 0

    for arg in sys.argv:
        if arg.startswith("webots_world:="):
            webots_world = arg.split(":=")[1]
        elif arg.startswith("number_robots:="):
            number_robots = int(arg.split(":=")[1])
        elif arg.startswith("total_robots:="):
            total_robots = int(arg.split(":=")[1])
        elif arg.startswith("pattern:="):
            pattern = arg.split(":=")[1]
        elif arg.startswith("log_level:="):
            log_level = arg.split(":=")[1]
        elif arg.startswith("robot:="):
            robot = arg.split(":=")[1]
        elif arg.startswith("sensor_type:="):
            sensor_type = arg.split(":=")[1]
        elif arg.startswith("driving_swarm:="):
            driving_swarm = arg.split(":=")[1]
            if driving_swarm == 'True':
                yaml_file = webots_world[:-4] + '.yaml'
                map_file = os.path.join(launch_file_dir, 'maps', yaml_file)
        elif arg.startswith("logging:="):
            logging = arg.split(":=")[1]
        elif arg.startswith("transport_object_shape:="):
            transport_object_shape = arg.split(":=")[1]
        elif arg.startswith("webots_mode:="):
            webots_mode = arg.split(":=")[1]
        elif arg.startswith("webots_gui:="):
            webots_gui = arg.split(":=")[1]
        elif arg.startswith("navigation_target_layout:="):
            navigation_target_layout = arg.split(":=")[1]
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
        elif arg.startswith("num_obstacles:="):
            num_obstacles = int(arg.split(":=")[1])
        else:
            known_args = ['/opt/ros/humble/bin/ros2', 'launch', 'launch_webots',
                         'create_enviroment.launch.py']
            if arg not in known_args:
                pass  # 忽略未知参数的提示

    # 根据pattern判断任务类型
    task_type = get_task_type_from_pattern(pattern)
    transport_object_shape = normalize_transport_object_shape(transport_object_shape)

    # 确定世界文件路径
    package_dir = os.path.join(get_package_share_directory('launch_webots'))
    worlds_dir = os.path.join(package_dir, 'worlds')
    template_world_path = os.path.join(worlds_dir, webots_world)

    print("=" * 50)
    print(f"世界文件模板    | {webots_world}")
    print(f"机器人数量      | {number_robots}")
    print(f"机器人类型      | {robot}")
    print(f"行为模式        | {pattern}")
    print(f"传感器类型      | {sensor_type}")
    print(f"日志级别        | {log_level}")
    print(f"位置生成方式    | 随机 (区域: X[{x_min}, {x_max}], Y[{y_min}, {y_max}])")
    if random_seed is not None:
        print(f"随机种子        | {random_seed}")
    print(f"  障碍物数量    | {num_obstacles}")
    print("=" * 50)

    # 确定机器人类型
    robot_type = robot
    webots_flag = True

    if robot_type.startswith('burger'):
        robot_type = "burger"
        baseframe = 'base_link'
    elif robot_type.startswith('waffle_pi'):
        robot_type = "waffle_pi"
        baseframe = 'base_link'
    elif robot_type.startswith('e_puck'):
        robot_type = "e_puck"
        baseframe = 'base_link'

    # 设置随机种子
    if random_seed is not None:
        random.seed(random_seed)

    # 根据任务类型生成任务物体位置
    task_objects = {}
    occupied_positions = []  # [(x, y, radius), ...]

    # 解析世界文件中已存在的物体，避免生成位置与其重叠
    existing_world_objects = parse_world_file_objects(template_world_path)
    for x, y, radius, name in existing_world_objects:
        occupied_positions.append((x, y, radius))

    if task_type == 'transport':
        # 运输任务：生成箱子和目标点
        task_objects['transport_object_shape'] = transport_object_shape
        box_pos = generate_random_positions_with_obstacles(
            1, x_min, x_max, y_min, y_max, min_distance, None,
            occupied_positions
        )[0]
        task_objects['transport_box'] = box_pos
        occupied_positions.append((box_pos[0], box_pos[1], 0.1))
        print(f"运输物体形状: {transport_object_shape}")
        print(f"运输箱子位置: ({box_pos[0]:.3f}, {box_pos[1]:.3f})")

        target_pos = generate_random_positions_with_obstacles(
            1, x_min, x_max, y_min, y_max, min_distance, None,
            occupied_positions
        )[0]
        task_objects['target_zone'] = target_pos
        occupied_positions.append((target_pos[0], target_pos[1], 0.2))
        print(f"目标点位置: ({target_pos[0]:.3f}, {target_pos[1]:.3f})")

    elif task_type == 'roundflocking':
        # 环绕任务：生成圆柱目标
        target_x_min, target_x_max = inset_bounds_for_clearance(
            x_min,
            x_max,
            ROUND_FLOCKING_TARGET_CLEARANCE,
        )
        target_y_min, target_y_max = inset_bounds_for_clearance(
            y_min,
            y_max,
            ROUND_FLOCKING_TARGET_CLEARANCE,
        )
        print(
            f"环绕目标生成区域: "
            f"X[{target_x_min:.3f}, {target_x_max:.3f}], "
            f"Y[{target_y_min:.3f}, {target_y_max:.3f}] "
            f"(留出 {ROUND_FLOCKING_TARGET_CLEARANCE:.2f}m 环绕半径)"
        )
        flocking_pos = generate_random_positions_with_obstacles(
            1,
            target_x_min,
            target_x_max,
            target_y_min,
            target_y_max,
            min_distance,
            None,
            occupied_positions
        )[0]
        task_objects['flocking_target'] = flocking_pos
        occupied_positions.append((flocking_pos[0], flocking_pos[1], 0.15))
        print(f"环绕目标位置: ({flocking_pos[0]:.3f}, {flocking_pos[1]:.3f})")

    elif task_type == 'navigation':
        # 导航任务：navigation_target_layout 仅支持 random 或 columns。
        # columns 启用规则列布局；列数按机器人数量自动选择，便于展示和可复现实验。
        normalized_layout = navigation_target_layout.strip().lower()
        if normalized_layout == 'random':
            navigation_targets = generate_random_positions_with_obstacles(
                number_robots,
                x_min,
                x_max,
                y_min,
                y_max,
                min_distance,
                None,
                occupied_positions
            )
        elif normalized_layout == 'columns':
            navigation_targets = generate_grid_positions(
                number_robots,
                x_min,
                x_max,
                y_min,
                y_max,
            )
        else:
            raise ValueError(
                'navigation_target_layout must be random or columns, '
                f'got: {navigation_target_layout}'
            )
        if normalized_layout == 'columns':
            print(
                f"导航目标布局: {normalized_layout}, "
                f"自动列数: {navigation_grid_columns(number_robots)}"
            )
        else:
            print(f"导航目标布局: {normalized_layout}")
        task_objects['navigation_targets'] = navigation_targets
        for i, pos in enumerate(navigation_targets):
            # 目标点可进入，不需要添加碰撞半径，但要避免与机器人初始位置重叠
            occupied_positions.append((pos[0], pos[1], 0.05))
            color_name = ['红', '绿', '蓝', '黄', '紫', '青', '橙', '棕', '粉', '橄榄'][i % 10]
            print(f"导航目标 {i} ({color_name}色): ({pos[0]:.3f}, {pos[1]:.3f})")

    # 生成障碍物（任意模式下可用）
    if num_obstacles > 0:
        obstacle_positions = generate_random_positions_with_obstacles(
            num_obstacles, x_min, x_max, y_min, y_max, min_distance, None, occupied_positions
        )
        task_objects['obstacles'] = obstacle_positions
        for i, pos in enumerate(obstacle_positions):
            occupied_positions.append((pos[0], pos[1], 0.1))
            print(f"障碍物 {i} 位置: ({pos[0]:.3f}, {pos[1]:.3f})")

    # 生成机器人位置（避开已有物体）
    positions = generate_random_positions_with_obstacles(
        number_robots, x_min, x_max, y_min, y_max, min_distance, None, occupied_positions
    )
    # 打印位置信息
    print("机器人位置:")
    for i, (x, y) in enumerate(positions):
        print(f"  robot_{i}: ({x:.3f}, {y:.3f})")

    # 生成包含机器人和任务物体的世界文件
    generated_world_name = f"generated_{webots_world}"
    generated_world_path = os.path.join(worlds_dir, generated_world_name)

    # 调用世界文件生成函数
    generate_world_file(
        template_world_path,
        generated_world_path,
        robot_type,
        number_robots,
        positions,
        z_height,
        task_objects,
    )

    # 添加日志级别参数
    log = LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=log_level,
            description="Logging level",
        )
    ])
    ld.add_action(log)

    if webots_flag:
        # 通过 start_webots.launch.py 启动 Webots（含 supervisor 和退出处理）
        webots_start = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'launch', 'start_webots.launch.py')
            ),
            launch_arguments={
                'world_path': generated_world_path,
                'webots_mode': webots_mode,
                'webots_gui': webots_gui,
            }.items(),
        )
        ld.add_action(webots_start)

        # 启动 GlobalInfoSupervisor 节点（发布全局场景信息）
        from webots_ros2_driver.utils import controller_url_prefix
        from ament_index_python.packages import get_package_prefix

        global_info_supervisor = Node(
            package='launch_webots',
            executable='global_info_supervisor',
            namespace='GlobalInfoSupervisor',
            output='screen',
            additional_env={
                'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'GlobalInfoSupervisor',
                'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')
            },
            respawn=True,
            parameters=[{'use_sim_time': True}]
        )
        ld.add_action(global_info_supervisor)

        # driving swarm 配置
        if driving_swarm == 'True':
            run_timeout = LaunchConfiguration('run_timeout', default=5)
            init_timeout = LaunchConfiguration('init_timeout', default=0)

            command_node = Node(
                package="experiment_supervisor",
                executable="command_node",
                output="screen",
                parameters=[{
                    'use_sim_time': True,
                    'run_timeout': run_timeout,
                    'init_timeout': init_timeout,
                    'robots': ['robot_' + str(i) for i in range(number_robots)],
                }]
            )

            exit_event_handler = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=command_node,
                    on_exit=EmitEvent(event=Shutdown(reason="command node exited"))
                )
            )
            ld.add_action(command_node)
            ld.add_action(exit_event_handler)

        # 为每个机器人启动控制器节点
        for i in range(number_robots):
            if driving_swarm == 'True':
                rviz_config_file = LaunchConfiguration(
                    'rviz_config_file',
                    default=os.path.join(get_package_share_directory('driving_swarm_bringup'), 'rviz', 'custom.rviz')
                )

                rviz = IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')
                    ),
                    launch_arguments={
                        'namespace': f'robot_{i}',
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
                        'namespace': f'robot_{i}',
                        'robot_name': f'robot_{i}',
                        'base_frame': baseframe,
                    }.items()
                )
                ld.add_action(tf_exchange)

            # 根据机器人类型创建对应的节点
            if robot_type == "e_puck":
                robot_nodes = create_epuck_nodes(i, use_sim_time=True)
            elif robot_type == "burger":
                robot_nodes = create_burger_nodes(i, use_sim_time=True)
            else:
                raise ValueError(f"不支持的机器人类型: {robot_type}")

            for node in robot_nodes:
                ld.add_action(node)

    # 配置目录
    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot_type)

    # 启动行为模式
    for i in range(number_robots):
        pattern_launch_file_name = pattern + '.launch.py'
        for root, dirs, files in os.walk(launch_pattern_dir):
            for name in files:
                if name == pattern_launch_file_name:
                    pattern_path = os.path.abspath(os.path.join(root, name))

        launch_patterns = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_bringup_dir, '/bringup_patterns.launch.py']),
            launch_arguments={
                'robot': robot,
                'robot_type': robot_type,
                'sensor_type': sensor_type,
                'robot_namespace': f"robot_{i}",
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

    # 日志记录
    if logging == 'True':
        exp_measurement_dir = get_package_share_directory('experiment_measurement')

        rosbag_recording = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(exp_measurement_dir, 'launch', 'rosbag_recording.launch.py')
            ),
            launch_arguments={
                'n_robots': str(total_robots),
                'robots_file': robot_file,
                'use_rosbag': 'True',
                'rosbag_topics_file': os.path.join(
                    get_package_share_directory('ros2swarm'), 'param', 'rosbag_topics.yaml'
                ),
                'qos_override_file': os.path.join(
                    get_package_share_directory('experiment_measurement'), 'params', 'qos_override.yaml'
                ),
            }.items()
        )
        ld.add_action(rosbag_recording)

    return ld
