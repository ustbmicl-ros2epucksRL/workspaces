#!/usr/bin/env python3
"""
机器人节点创建模块
为E-puck和TurtleBot3 Burger机器人创建Webots控制器和相关节点
"""
import os
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch_webots.startup_policy import controller_start_delay_seconds


def _staggered_action(action, robot_index: int):
    delay = controller_start_delay_seconds(robot_index)
    if delay <= 0:
        return action
    return TimerAction(period=delay, actions=[action])


def create_epuck_nodes(
    robot_index: int,
    use_sim_time: bool = True,
    webots_port: str = '1234',
):
    """
    创建E-puck机器人的所有节点
    
    Args:
        robot_index: 机器人索引（用于命名空间和名称）
        use_sim_time: 是否使用仿真时间
    
    Returns:
        list: 需要添加到LaunchDescription的actions列表
    """
    package_dir = get_package_share_directory('launch_webots')
    
    robot_name = f"robot_{robot_index}"
    robot_namespace = f"robot_{robot_index}"
    
    # 控制器管理器配置
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    
    robot_description_path = os.path.join(package_dir, 'assets', 'epuck_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')
    
    # 判断是否使用带时间戳的速度命令
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (
        os.environ['ROS_DISTRO'] in ['kilted', 'rolling', 'jazzy']
    )
    
    if use_twist_stamped:
        mappings = [
            (f'/{robot_namespace}/diffdrive_controller/cmd_vel', f'/{robot_namespace}/cmd_vel'),
            (f'/{robot_namespace}/diffdrive_controller/odom', f'/{robot_namespace}/odom')
        ]
    else:
        mappings = [
            (f'/{robot_namespace}/diffdrive_controller/cmd_vel_unstamped', f'/{robot_namespace}/cmd_vel'),
            (f'/{robot_namespace}/diffdrive_controller/odom', f'/{robot_namespace}/odom'),
        ]
    
    actions = []
    
    # 同一 controller_manager 的加载调用必须串行。两个独立 spawner 在负载较高时
    # 会同时超时重试，形成“实际已加载、客户端仍重试”的重复加载竞态。
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller', 'joint_state_broadcaster']
        + controller_manager_timeout
        + ['--service-call-timeout', '50', '--switch-timeout', '50', '--activate-as-group'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    ros_control_spawners = [_staggered_action(controller_spawner, robot_index)]

    # Webots控制器节点
    epuck_controller = WebotsController(
        robot_name=robot_name,
        port=webots_port,
        namespace=robot_namespace,
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True
            },
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )
    actions.append(epuck_controller)

    # 等待控制器连接后启动控制器spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=epuck_controller,
        nodes_to_start=ros_control_spawners
    )
    actions.append(waiting_nodes)

    # E-puck传感器节点
    epuck_node = Node(
        package='launch_webots',
        executable='epuck_node',
        namespace=robot_namespace,
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )
    actions.append(epuck_node)
    
    return actions


def create_burger_nodes(
    robot_index: int,
    use_sim_time: bool = True,
    webots_port: str = '1234',
):
    """
    创建TurtleBot3 Burger机器人的所有节点
    
    Args:
        robot_index: 机器人索引（用于命名空间和名称）
        use_sim_time: 是否使用仿真时间
    
    Returns:
        list: 需要添加到LaunchDescription的actions列表
    """
    package_dir = get_package_share_directory('launch_webots')
    
    robot_name = f"robot_{robot_index}"
    robot_namespace = f"robot_{robot_index}"
    
    # 控制器管理器配置
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    
    robot_description_path = os.path.join(package_dir, 'assets', 'turtlebot_webots.urdf')
    ros2_control_params = os.path.join(package_dir, 'resource', 'burger_ros2control.yml')
    
    # 判断是否使用带时间戳的速度命令
    use_twist_stamped = 'ROS_DISTRO' in os.environ and (
        os.environ['ROS_DISTRO'] in ['kilted', 'rolling', 'jazzy']
    )
    
    if use_twist_stamped:
        mappings = [
            (f'/{robot_namespace}/diffdrive_controller/cmd_vel', f'/{robot_namespace}/cmd_vel'),
            (f'/{robot_namespace}/diffdrive_controller/odom', f'/{robot_namespace}/odom')
        ]
    else:
        mappings = [
            (f'/{robot_namespace}/diffdrive_controller/cmd_vel_unstamped', f'/{robot_namespace}/cmd_vel'),
            (f'/{robot_namespace}/diffdrive_controller/odom', f'/{robot_namespace}/odom'),
        ]
    
    actions = []
    
    # 静态TF发布器：base_link -> base_footprint
    footprint_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=robot_namespace,
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
    )
    actions.append(footprint_publisher)
    
    # 同一 controller_manager 的加载调用必须串行，原因见 create_epuck_nodes。
    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        namespace=robot_namespace,
        output='screen',
        prefix=controller_manager_prefix,
        arguments=['diffdrive_controller', 'joint_state_broadcaster']
        + controller_manager_timeout
        + ['--service-call-timeout', '50', '--switch-timeout', '50', '--activate-as-group'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    
    ros_control_spawners = [_staggered_action(controller_spawner, robot_index)]

    # Webots控制器节点
    burger_controller = WebotsController(
        robot_name=robot_name,
        port=webots_port,
        namespace=robot_namespace,
        parameters=[
            {
                'robot_description': robot_description_path,
                'use_sim_time': use_sim_time,
                'set_robot_state_publisher': True
            },
            ros2_control_params
        ],
        remappings=mappings,
        respawn=True
    )
    actions.append(burger_controller)

    # 等待控制器连接后启动控制器spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=burger_controller,
        nodes_to_start=ros_control_spawners
    )
    actions.append(waiting_nodes)
    
    return actions
