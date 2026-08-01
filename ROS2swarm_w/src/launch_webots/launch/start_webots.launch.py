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


def _to_bool(value):
    normalized = value.strip().lower()
    if normalized == 'true':
        return True
    if normalized == 'false':
        return False
    raise ValueError('webots_stream must be true or false')


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
    declare_stream = DeclareLaunchArgument(
        'webots_stream',
        default_value='false',
        description='Enable the native Webots W3D stream'
    )
    declare_port = DeclareLaunchArgument(
        'webots_port',
        default_value='1234',
        description='Webots TCP/W3D stream port'
    )

    def launch_setup(context):
        world = LaunchConfiguration('world_path').perform(context)
        webots_stream = _to_bool(
            LaunchConfiguration('webots_stream').perform(context)
        )
        webots_port = LaunchConfiguration('webots_port').perform(context)
        if not webots_port.isdigit() or not 1 <= int(webots_port) <= 65535:
            raise ValueError('webots_port must be between 1 and 65535')
        webots = WebotsLauncher(
            world=world,
            gui=LaunchConfiguration('webots_gui'),
            mode=LaunchConfiguration('webots_mode'),
            stream=webots_stream,
            port=webots_port,
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
        declare_stream,
        declare_port,
        OpaqueFunction(function=launch_setup)
    ])
