#!/usr/bin/env python3
"""
Teleop Launch for Mobile Base
==============================
Launch keyboard teleop for controlling the mobile base.

Usage:
    ros2 launch mobile_manipulator_bringup teleop.launch.py
    ros2 launch mobile_manipulator_bringup teleop.launch.py speed:=0.5 turn:=1.0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    speed = LaunchConfiguration('speed')
    turn = LaunchConfiguration('turn')

    return LaunchDescription([
        DeclareLaunchArgument('speed', default_value='0.3',
            description='Linear speed limit (m/s)'),
        DeclareLaunchArgument('turn', default_value='0.8',
            description='Angular speed limit (rad/s)'),

        # Teleop Keyboard Node
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            prefix='xterm -e',
            output='screen',
            parameters=[{
                'speed': speed,
                'turn': turn,
            }],
            remappings=[
                ('cmd_vel', '/cmd_vel')
            ]
        ),
    ])
