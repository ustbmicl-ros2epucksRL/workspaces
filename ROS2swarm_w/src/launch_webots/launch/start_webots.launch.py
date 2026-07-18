import launch
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    """
    Starts a Webots simulation where elements can be added via command line.
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
    declare_mode = DeclareLaunchArgument(
        'webots_mode',
        default_value='realtime',
        description='Webots run mode: pause, realtime, fast, or rendering'
    )
    declare_gui = DeclareLaunchArgument(
        'webots_gui',
        default_value='true',
        description='Start Webots with GUI/rendering enabled'
    )
    def launch_setup(context):
        world = LaunchConfiguration('world_path').perform(context)
        webots = WebotsLauncher(
            world=world,
            gui=LaunchConfiguration('webots_gui'),
            mode=LaunchConfiguration('webots_mode'),
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
        declare_mode,
        declare_gui,
        OpaqueFunction(function=launch_setup)
    ])
