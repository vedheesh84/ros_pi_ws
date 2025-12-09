#!/usr/bin/env python3
"""
Base Controller Launch File
============================
Launch differential drive controller with Arduino bridge.

Usage:
    ros2 launch base_controller base_control.launch.py
    ros2 launch base_controller base_control.launch.py serial_port:=/dev/ttyUSB0
    ros2 launch base_controller base_control.launch.py wheel_radius:=0.1 wheel_base:=0.5
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0',
            description='Arduino serial port'),
        DeclareLaunchArgument('baud_rate', default_value='115200',
            description='Serial baud rate'),
        DeclareLaunchArgument('encoder_ticks_per_rev', default_value='360',
            description='Encoder ticks per revolution'),
        DeclareLaunchArgument('wheel_radius', default_value='0.075',
            description='Wheel radius in meters'),
        DeclareLaunchArgument('wheel_base', default_value='0.67',
            description='Distance between left/right wheels in meters'),
        DeclareLaunchArgument('publish_rate', default_value='50.0',
            description='State publish rate in Hz'),
        DeclareLaunchArgument('base_frame', default_value='body_link',
            description='Robot base frame'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
            description='Odometry frame'),

        Node(
            package='base_controller',
            executable='base_hardware_bridge.py',
            name='base_hardware_bridge',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': LaunchConfiguration('baud_rate'),
                'encoder_ticks_per_rev': LaunchConfiguration('encoder_ticks_per_rev'),
                'wheel_radius': LaunchConfiguration('wheel_radius'),
                'wheel_base': LaunchConfiguration('wheel_base'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'base_frame': LaunchConfiguration('base_frame'),
                'odom_frame': LaunchConfiguration('odom_frame'),
            }],
            output='screen'
        ),
    ])
