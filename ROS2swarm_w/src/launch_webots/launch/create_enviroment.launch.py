import os
import sys
import launch_ros.actions

from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import TimerAction
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

def generate_launch_description():
    """Creates the environment with webots, add robots and starts their behaviour"""

    launch_file_dir = os.path.join(get_package_share_directory('launch_webots'))
    launch_pattern_dir = os.path.join(get_package_share_directory('ros2swarm'), 'launch', 'pattern')
    launch_bringup_dir = os.path.join(get_package_share_directory('ros2swarm'))
    # driving swarm 
    robot_file = os.path.join(get_package_share_directory('ros2swarm'), 'param', 'ROS2swarm_sim.yaml')
    map_file = os.path.join(launch_file_dir, 'maps', 'default.yaml') 
    #tf_exchange_dir = get_package_share_directory('tf_exchange')

    # 控制器与Webots连接的等待时长
    controller_manager_timeout = ['--controller-manager-timeout', '50']
    controller_manager_prefix = 'python.exe' if os.name == 'nt' else ''

    # 获取webots_ros2_epuck包的共享目录
    #package_dir = os.path.join(get_package_share_directory('webots_ros2_epuck'))
    package_dir = os.path.join(get_package_share_directory('launch_webots'))
    
    ld = LaunchDescription()
    
    for arg in sys.argv:
        if arg.startswith("webots_world:="):  # Name of the webots world
            webots_world = arg.split(":=")[1]        
        elif arg.startswith("number_robots:="):  # The number of robots to spawn in the world
            number_robots = int(arg.split(":=")[1])
        elif arg.startswith("total_robots:="):  # The number of robots to spawn in the world
            total_robots = int(arg.split(":=")[1])
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
        elif arg.startswith("driving_swarm:="):  # use driving swarm framework 
            driving_swarm = arg.split(":=")[1]
            
            if driving_swarm == 'True':
                # map file for driving swarm 
                yaml_file = webots_world[:-6] + '.yaml' 
                map_file = os.path.join(launch_file_dir, 'maps', yaml_file)
        elif arg.startswith("logging:="):  # log data in rosbag 
            logging = arg.split(":=")[1]
        else:
            if arg not in ['/opt/ros/humble/bin/ros2',
                           'launch',
                           'launch_webots',
                           'create_enviroment.launch.py']:
                print("Argument not known: '", arg, "'")

    worlds_dir = os.path.join(launch_file_dir, 'worlds')
    world_file_name = webots_world
    world_file_path = os.path.join(worlds_dir, world_file_name)


    print("---------------------------------------")
    print("world file name  |", world_file_name)
    print("---------------------------------------")
    print("number of robots |", number_robots)
    print("---------------------------------------")
    print("pattern          |", pattern)
    print("---------------------------------------")
    print("log level        |", log_level)
    print("---------------------------------------")
    print("robot            |", robot)
    print("---------------------------------------")
    print("sensor_type      |", sensor_type)
    print("---------------------------------------")
    
    # allows to use the same configuration files for each robot type but different mesh models
    robot_type = robot
    webots_flag = True
    
    if robot_type.startswith('burger'):
        robot_type = "burger"
        baseframe = 'base_link'
        robot_description_path = os.path.join(package_dir, 'assets', 'turtlebot_webots.urdf')
        ros2_control_params = os.path.join(package_dir, 'resource', 'burger_ros2control.yml')
    elif robot_type.startswith('waffle_pi'):
        robot_type = "waffle_pi"
        baseframe = 'base_link'
    elif robot_type.startswith('e_puck'):
        robot_type = "e_puck"
        baseframe = 'base_link' # ToDo 
        robot_description_path = os.path.join(package_dir, 'assets', 'epuck_webots.urdf')
        ros2_control_params = os.path.join(package_dir, 'resource', 'ros2_control.yml')


    print("robot type       |", robot_type)
    print("---------------------------------------")



    # Add the log level argument to the launch description
    log = LaunchDescription([
        DeclareLaunchArgument(
            "log_level",
            default_value=log_level,
            description="Logging level",
        )
    ])
    ld.add_action(log)
    
    
    if webots_flag:
        # Add webots start script
        webots_start = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([launch_file_dir, '/start_webots.launch.py']),
            launch_arguments={'world_path': world_file_path}.items(),
        )
        ld.add_action(webots_start)
        
        """
        启动一个节点，用于发布数据
        state_node = Node(package="launch_webots",
                          executable="ground_truth_publisher",
                          output="screen",
                          parameters=[{
                          'use_sim_time': True,
                          }])
        ld.add_action(state_node)
        """
        
        # driving swarm 
        if driving_swarm == 'True': 
            run_timeout = LaunchConfiguration('run_timeout', default=5)
            init_timeout = LaunchConfiguration('init_timeout', default=0)
        
            command_node = Node(package="experiment_supervisor",
                        executable="command_node",
                        output="screen",
                        parameters=[{
                           'use_sim_time': True,
                           'run_timeout': run_timeout,
                           'init_timeout': init_timeout,
                           'robots': ['robot_' + str(i) for i in range(number_robots)],
                           }])

            exit_event_handler = RegisterEventHandler(event_handler=OnProcessExit(
                                                      target_action=command_node,
                                                      on_exit=EmitEvent(event=Shutdown(reason="command node exited"))
                                                      )
                                                  )
            ld.add_action(command_node)
            ld.add_action(exit_event_handler)

        for i in range(number_robots):
        
            if driving_swarm == 'True': 
                rviz_config_file = LaunchConfiguration('rviz_config_file', default=os.path.join(get_package_share_directory('driving_swarm_bringup'), 'rviz', 'custom.rviz'))
                
                rviz = IncludeLaunchDescription(
	                   PythonLaunchDescriptionSource(
		            os.path.join(get_package_share_directory('nav2_bringup'), 'launch', 'rviz_launch.py')),
		        #condition=IfCondition(LaunchConfiguration('use_rviz')),
		        launch_arguments={
		            'namespace': ['robot_', str(i)],
		            'use_namespace': 'true',
		            'use_sim_time': 'true',
		            'rviz_config': rviz_config_file
		            }.items()
		        )

                ld.add_action(rviz)
                
                # DRIVING SWARM 
                tf_exchange = IncludeLaunchDescription(
		        PythonLaunchDescriptionSource(
		            os.path.join(tf_exchange_dir, 'launch', 'tf_exchange.launch.py')),
		            launch_arguments={
		            'namespace': ['robot_', str(i)],
		            'robot_name': ['robot_', str(i)],
		            'base_frame': baseframe,
		            }.items()
		        )
                ld.add_action(tf_exchange) 
            
            
            # add webots node
            webots_node = launch_ros.actions.Node(
                package='launch_webots',
                executable='add_bot_node',
                namespace=f"robot_{i}",
                name=['webotsRobotNode_', str(i)],
                output='screen',
                arguments=[
                    '--robot_name', ['robot_', str(i)],
                    '--robot_namespace', ['robot_', str(i)],
                    '-x', str(x_start + i * x_dist),
                    '-y', str(y_start + i * y_dist),
                    '-z', '0.1',
                    '--type_of_robot', robot,
                    '-ds', driving_swarm, 
                ]
            )
            ld.add_action(webots_node)
                
            # 机器人的差分驱动控制器节点
            diffdrive_controller_spawner = Node(
                package='controller_manager',
                executable='spawner',
                namespace=f"robot_{i}",
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
                namespace=f"robot_{i}",
                output='screen',
                prefix=controller_manager_prefix,
                arguments=['joint_state_broadcaster'] + controller_manager_timeout,
                parameters=[
                    {'use_sim_time': True},
                ],
            )
            # 定义ROS 2控制节点的参数
            ros_control_spawners = [diffdrive_controller_spawner, joint_state_broadcaster_spawner]
            use_twist_stamped = 'ROS_DISTRO' in os.environ and (os.environ['ROS_DISTRO'] in ['kilted', 'rolling', 'jazzy'])
            if use_twist_stamped:
                mappings = [
                    (f'/robot_{i}/diffdrive_controller/cmd_vel', f'/robot_{i}/cmd_vel'), 
                    (f'/robot_{i}/diffdrive_controller/odom', f'/robot_{i}/odom')
                    ]
            else:
                mappings = [
                    (f'/robot_{i}/diffdrive_controller/cmd_vel_unstamped', f'/robot_{i}/cmd_vel'),
                    (f'/robot_{i}/diffdrive_controller/odom', f'/robot_{i}/odom'),
                    ]

            # 为e-puck机器人创建Webots控制器节点
            mbot_controller = WebotsController(
                robot_name= f"robot_{i}",
                namespace=f"robot_{i}",
                parameters=[
                    {'robot_description': robot_description_path,
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
                namespace=f"robot_{i}",
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                ],
                )
            ld.add_action(epuck_process)

    config_dir = os.path.join(get_package_share_directory('ros2swarm'), 'config', robot_type)
        
    # find out exact path of the pattern launch file
    for i in range(number_robots):
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
                              'robot_namespace': f"robot_{i}",
                              'sensor_type': sensor_type, 
                              'pattern': pattern_path,
                              'config_dir': config_dir,
                              'map': map_file,
                              'driving_swarm': driving_swarm,
                              'params_file': os.path.join(
            get_package_share_directory('ros2swarm'), 'param', 'nav2_params_' + robot_type + '_namespaced.yaml')
        }.items()
        )
        ld.add_action(launch_patterns)

    
    if logging=='True':

    
        exp_measurement_dir = get_package_share_directory('experiment_measurement')
        
        rosbag_recording = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(exp_measurement_dir,
                             'launch', 'rosbag_recording.launch.py')),
                 launch_arguments={'n_robots': str(total_robots), 
                              'robots_file': robot_file,
                              'use_rosbag': 'True',
                              'rosbag_topics_file': os.path.join(get_package_share_directory('ros2swarm'), 'param', 'rosbag_topics.yaml'), 
                              'qos_override_file': os.path.join(get_package_share_directory('experiment_measurement'), 'params', 'qos_override.yaml'),
                             }.items()
        )
    
        ld.add_action(rosbag_recording)
    

    return ld
