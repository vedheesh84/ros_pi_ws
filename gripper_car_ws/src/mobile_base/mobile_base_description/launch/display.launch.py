#!/usr/bin/env python3
"""
Mobile Base Description Display Launch
=======================================
Launch robot_state_publisher and RViz for visualization.

Usage:
    ros2 launch mobile_base_description display.launch.py
    ros2 launch mobile_base_description display.launch.py gui:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = FindPackageShare('mobile_base_description')

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'mobile_base.urdf.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'config', 'display.rviz'])

    robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true',
            description='Enable joint_state_publisher_gui'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation time'),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }]
        ),

        # Joint State Publisher GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(gui),
            parameters=[{'use_sim_time': use_sim_time}]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}]
        ),
    ])
