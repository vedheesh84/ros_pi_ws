#!/usr/bin/env python3
"""
Arm Controller Launch File
===========================
Launch servo bridge and trajectory bridge for robotic arm control.

Usage:
    ros2 launch arm_controller arm_control.launch.py
    ros2 launch arm_controller arm_control.launch.py serial_port:=/dev/ttyACM1
    ros2 launch arm_controller arm_control.launch.py use_trajectory_bridge:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0',
            description='Arduino serial port'),
        DeclareLaunchArgument('baud_rate', default_value='115200',
            description='Serial baud rate'),
        DeclareLaunchArgument('publish_rate', default_value='50.0',
            description='Joint state publish rate in Hz'),
        DeclareLaunchArgument('use_trajectory_bridge', default_value='true',
            description='Enable MoveIt trajectory bridge'),

        # Servo Bridge Node
        Node(
            package='arm_controller',
            executable='servo_bridge.py',
            name='servo_bridge',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'publish_rate': LaunchConfiguration('publish_rate'),
            }],
            output='screen'
        ),

        # Trajectory Bridge Node (for MoveIt integration)
        Node(
            package='arm_controller',
            executable='trajectory_bridge.py',
            name='trajectory_bridge',
            condition=IfCondition(LaunchConfiguration('use_trajectory_bridge')),
            output='screen'
        ),
    ])
