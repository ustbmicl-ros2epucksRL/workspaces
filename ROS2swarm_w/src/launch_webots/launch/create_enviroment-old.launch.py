import os
import sys
import launch_ros.actions

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.substitutions import PythonExpression

def launch_setup(context, *args, **kwargs):
    """Creates the environment with webots, add robots and starts their behaviour"""

    launch_file_dir = os.path.join(get_package_share_directory('launch_webots'))
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')
    launch_bringup_dir = os.path.join(get_package_share_directory('ros2swarm'))

    map_file = os.path.join(launch_file_dir, 'maps', 'default.yaml') 

    # 控制器与Webots连接的等待时长
    controller_manager_timeout = ['--controller-manager-timeout', '100']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''
    
    
    # 获取webots_ros2_epuck包的共享目录
    #package_dir = os.path.join(get_package_share_directory('webots_ros2_epuck'))
    package_dir = os.path.join(get_package_share_directory('launch_webots'))

    #创建一个空的launch描述对象
    ld = LaunchDescription()

    sim_idx = LaunchConfiguration('sim_idx').perform(context)
    port = LaunchConfiguration('port').perform(context)
    
    #遍历命令行参数
    for arg in sys.argv:
        if arg.startswith("webots_world:="):  # Name of the webots world
            webots_world = arg.split(":=")[1] 
        elif arg.startswith("number_robots:="):  # The number of robots to spawn in the world
            number_robots = int(arg.split(":=")[1])
        elif arg.startswith("number_sims:="):  # The number of simulations to spawn in the world
            number_simulations = int(arg.split(":=")[1])  
        elif arg.startswith("pattern:="):  # The pattern executed by the robots
            pattern = arg.split(":=")[1]
        elif arg.startswith("log_level:="):  # The log level used in this execution
            log_level = arg.split(":=")[1]
        elif arg.startswith("robot:="):  # The type of robot
            robot = arg.split(":=")[1]
        elif arg.startswith("sensor_type:="):  # The type of sensor
            sensor_type = arg.split(":=")[1]
        elif arg.startswith("x_start:="):  # position first robot on x-axis 
            x_start = float(arg.split(":=")[1])
        elif arg.startswith("x_dist:="):  # position first robot on x-axis 
            x_dist = float(arg.split(":=")[1])
        elif arg.startswith("y_start:="):  # position of first robot on y-axis 
            y_start = float(arg.split(":=")[1])
        elif arg.startswith("y_dist:="):  # increment of positions on y-axis 
            y_dist = float(arg.split(":=")[1])
        else:
            if arg not in ['/opt/ros/humble/bin/ros2',
                           'launch',
                           'launch_webots',
                           'create_enviroment.launch.py',
                           'create_multi_enviroment.launch.py']:
                print("Argument not known: '", arg, "'")

    worlds_dir = os.path.join(launch_file_dir, 'worlds')
    world_file_name = webots_world
    world_file_path = os.path.join(worlds_dir, world_file_name)
    
    # allows to use the same configuration files for each robot type but different mesh models
    robot_type = robot
    
    if robot_type.startswith('burger'):
        robot_type = "burger"
        robot_description_path = os.path.join(package_dir, 'assets', 'turtlebot_webots.urdf')
        ros2_control_params = os.path.join(package_dir, 'resource', 'burger_ros2control.yml')
    elif robot_type.startswith('e_puck'):
        robot_type = "e_puck"
        robot_description_path = os.path.join(package_dir, 'assets', 'epuck_webots.urdf')
        ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')

    # Add the log level argument to the launch description
    log = LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=log_level,
            description="Logging level",
        )
    ])
    ld.add_action(log)
    
    namespace = f"webots_instance_{sim_idx}"  # 设定唯一的 ROS 2 命名空间
    
    # 启动Webots仿真,world_name参数来指定世界文件
    webots_start = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_dir, '/start_webots.launch.py']),
        launch_arguments={'world_path': world_file_path,
                          'port': port      # 避免端口冲突
                          }.items(),
        )
    ld.add_action(webots_start)
            
    """    #启动一个节点，用于发布数据
    state_node = Node(
                    package="launch_webots",
                    executable="ground_truth_publisher",
                    namespace=namespace,  # 绑定到对应 Webots 实例
                    output="screen",
                    parameters=[{'use_sim_time': True,}]
                    )
    ld.add_action(state_node)"""
        
    for robot_idx in range(number_robots):            
                  
        # add webots node（在webots仿真中添加机器人节点）
        webots_node = launch_ros.actions.Node(
                package='launch_webots',
                executable='add_bot_node',
                namespace=f"{namespace}/robot_{robot_idx}" ,
                name=f'webots_{sim_idx}_RobotNode_{robot_idx}',
                output='screen',
                arguments=[
                    '--robot_name', f"robot_{sim_idx}_{robot_idx}",
                    '--robot_namespace', f"{namespace}/robot_{robot_idx}",
                    '--webots_instance', namespace,         #指定在第几个webots实例中生成机器人
                    '-x', str(x_start + robot_idx * x_dist),
                    '-y', str(y_start + robot_idx * y_dist),
                    '-z', '0.1',
                    '--type_of_robot', robot,
                    ]
                )
        
        # 使用 TimerAction 添加延迟，确保Webots启动后再生成机器人
        ld.add_action(TimerAction(
            period=15.0,  # 延迟15秒
            actions=[webots_node]
            ))
            
        # 机器人的差分驱动控制器节点
        diffdrive_controller_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=f"{namespace}/robot_{robot_idx}",
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['diffdrive_controller'] + controller_manager_timeout,
            parameters=[
                {'use_sim_time': True},
            ],
        )

        # 创建关节状态广播器节点生成器
        joint_state_broadcaster_spawner = Node(
            package='controller_manager',
            executable='spawner',
            namespace=f"{namespace}/robot_{robot_idx}",
            output='screen',
            prefix=controller_manager_prefix,
            arguments=['joint_state_broadcaster'] + controller_manager_timeout,
            parameters=[
                {'use_sim_time': True},
            ],
        )
        # 定义ROS 2控制节点的参数
        ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]

        # 定义话题重映射规则
        mappings = [
        (f'/{namespace}/robot_{robot_idx}/diffdrive_controller/cmd_vel_unstamped', f'/{namespace}/robot_{robot_idx}/cmd_vel'),
        (f'/{namespace}/robot_{robot_idx}/diffdrive_controller/odom', f'/{namespace}/robot_{robot_idx}/odom')
        ]

        # 为e-puck机器人创建Webots控制器节点
        mbot_controller = WebotsController(
            robot_name= f"robot_{sim_idx}_{robot_idx}",
            namespace=f"{namespace}/robot_{robot_idx}",
            port=port,
            parameters=[
                {'namespace':f"{namespace}/robot_{robot_idx}",
                'robot_description': robot_description_path,
                'use_sim_time': True,
                'set_robot_state_publisher': True},
                ros2_control_params
            ],
            remappings=mappings,
            respawn=True
        )
        ld.add_action(mbot_controller)

        # mbot_controller 需要首先启动并建立与 Webots 中机器人的连接，然后其他控制器（例如差分驱动控制器）才能依赖它继续工作。
        waiting_nodes = WaitForControllerConnection(
            target_driver=mbot_controller,
            nodes_to_start = ros_control_spawners
        )
        ld.add_action(waiting_nodes)

        epuck_process = Node(
            package='webots_ros2_epuck',
            executable='epuck_node',
            namespace=f"{namespace}/robot_{robot_idx}",
            output='screen',
            parameters=[
                {'use_sim_time': True},
            ],
            )
        ld.add_action(epuck_process)

    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot_type)

    if robot_type.startswith('burger'):
        urdf_file_name = 'turtlebot_webots.urdf'
        urdf_file = os.path.join(package_dir, 'resource', urdf_file_name)
    elif robot_type.startswith('e_puck'):
        urdf_file_name = 'epuck_webots.urdf'
        urdf_file = os.path.join(package_dir, 'resource', urdf_file_name)

    # find out exact path of the pattern launch file
    for robot_idx in range(number_robots):
        pattern_launch_file_name = pattern + '.launch.py'
        for root, dirs, files in os.walk(launch_pattern_dir):
            for name in files:
                if name == pattern_launch_file_name:
                       pattern_path = os.path.abspath(os.path.join(root, name))

        # add patterns
        launch_patterns = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [launch_bringup_dir, '/' + 'bringup_patterns.launch.py']),
                launch_arguments={'robot': robot,
                            'robot_type': robot_type,
                            'sensor_type': sensor_type,
                            'robot_namespace': f"{namespace}/robot_{robot_idx}",
                            'sensor_type': sensor_type,
                            'pattern': pattern_path,
                            'config_dir': config_dir,
                            'urdf_file': urdf_file,
                            'map': map_file,
                            'params_file': os.path.join(
                get_package_share_directory('ros2swarm'), 'param', 'nav2_params_' + robot_type + '_namespaced.yaml')
            }.items()
        )
        ld.add_action(launch_patterns)
    
    return list(ld.describe_sub_entities())


def generate_launch_description():

    # 声明所有输入参数
    declare_sim_idx = DeclareLaunchArgument('sim_idx', default_value='1',
                             description='Index of the simulation instance')
    declare_port = DeclareLaunchArgument('port', default_value='1234',
                             description='Webots server port')
    
    return LaunchDescription([
        declare_sim_idx,
        declare_port,
        OpaqueFunction(function=launch_setup),
    ])