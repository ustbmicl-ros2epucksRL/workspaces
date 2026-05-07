import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    """
    Starts a webots simulation in the mode, where elements could be added via command line
    """
    declare_world = DeclareLaunchArgument(
        'world_path',
        description='Full path to the Webots world file (.wbt)'
    )

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    def launch_setup(context):
        world = LaunchConfiguration('world_path').perform(context)
        webots = WebotsLauncher(
            world=world,
            ros2_supervisor=True
        )

        return [
            webots,
            webots._supervisor,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=webots,
                    on_exit=[EmitEvent(event=launch.events.Shutdown())]
                )
            )
        ]

    return LaunchDescription([
        declare_world,
        declare_sim_time,
        OpaqueFunction(function=launch_setup)
    ])