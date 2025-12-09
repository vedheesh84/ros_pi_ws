#!/usr/bin/env python3
"""
Hardware Bringup Launch
========================

Launch hardware nodes for real robot operation with Arduino.

Usage:
    ros2 launch base_controller hardware.launch.py
    ros2 launch base_controller hardware.launch.py serial_port:=/dev/ttyACM0
    ros2 launch base_controller hardware.launch.py use_rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_controller = get_package_share_directory('base_controller')
    pkg_description = get_package_share_directory('mobile_base_description')

    # Launch configuration
    serial_port = LaunchConfiguration('serial_port')
    use_rviz = LaunchConfiguration('use_rviz')

    # Paths
    params_file = os.path.join(pkg_controller, 'config', 'base_params.yaml')
    urdf_file = os.path.join(pkg_description, 'urdf', 'mobile_base.urdf.xacro')
    rviz_config = os.path.join(pkg_description, 'config', 'display.rviz')

    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    return LaunchDescription([
        # ==================== ARGUMENTS ====================

        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM0',
            description='Arduino serial port'
        ),

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2'
        ),

        # ==================== ROBOT STATE PUBLISHER ====================

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': False
            }]
        ),

        # ==================== SERIAL BRIDGE ====================

        Node(
            package='base_controller',
            executable='serial_bridge_node.py',
            name='serial_bridge',
            output='screen',
            parameters=[
                params_file,
                {'serial_port': serial_port}
            ]
        ),

        # ==================== ODOMETRY PUBLISHER ====================

        Node(
            package='base_controller',
            executable='odom_publisher_node.py',
            name='odom_publisher',
            output='screen',
            parameters=[params_file]
        ),

        # ==================== RVIZ ====================

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz)
        ),
    ])
