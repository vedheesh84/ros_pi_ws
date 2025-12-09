#!/usr/bin/env python3
"""Arduino Hardware Bridge Launch"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('arduino_bridge')

    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    baud_rate = LaunchConfiguration('baud_rate', default='115200')

    hardware_bridge = Node(
        package='arduino_bridge',
        executable='hardware_bridge.py',
        name='hardware_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'wheel_radius': 0.075,
            'wheel_base': 0.67,
            'encoder_ticks_per_rev': 360,
            'publish_rate': 50.0,
        }],
        emulate_tty=True
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        hardware_bridge,
    ])
