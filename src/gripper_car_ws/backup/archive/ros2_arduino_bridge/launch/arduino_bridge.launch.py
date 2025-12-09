from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.substitutions import LaunchConfiguration
from launch.actions import EmitEvent
from launch.events import Shutdown
import os
import shutil
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('ros2_arduino_bridge')
    config_file = os.path.join(pkg_dir, 'config', 'arduino_params.yaml')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino'
    )

    use_xterm_arg = DeclareLaunchArgument(
        'use_xterm',
        default_value='true',
        description='Launch teleop in separate xterm window (true/false)'
    )

    arduino_bridge_node = Node(
        package='ros2_arduino_bridge',
        executable='arduino_bridge',
        name='arduino_bridge',
        parameters=[config_file, {'port': LaunchConfiguration('port')}],
        output='screen',
        emulate_tty=True,
        sigterm_timeout='5',
        sigkill_timeout='10'
    )

    # Determine if we should use xterm based on launch arg
    use_xterm = LaunchConfiguration('use_xterm')

    teleop_node_kwargs = dict(
        package='ros2_arduino_bridge',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        emulate_tty=True,
        sigterm_timeout='5',
        sigkill_timeout='10'
    )

    xterm_path = shutil.which('xterm')
    if xterm_path:
        # Add xterm with proper signal handling
        teleop_node_kwargs['prefix'] = f'{xterm_path} -hold -e'

    teleop_node = Node(**teleop_node_kwargs)

    # Shutdown entire launch when arduino_bridge exits
    shutdown_on_bridge_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=arduino_bridge_node,
            on_exit=[EmitEvent(event=Shutdown(reason='Arduino bridge exited'))]
        )
    )

    # Shutdown entire launch when teleop exits
    shutdown_on_teleop_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=teleop_node,
            on_exit=[EmitEvent(event=Shutdown(reason='Teleop keyboard exited'))]
        )
    )

    return LaunchDescription([
        port_arg,
        use_xterm_arg,
        arduino_bridge_node,
        teleop_node,
        shutdown_on_bridge_exit,
        shutdown_on_teleop_exit
    ])
