#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Get the package directory
    pkg_share = FindPackageShare('four_wheel_bot')
    
    # Get the URDF file path
    urdf_file_path = PathJoinSubstitution([pkg_share, 'urdf', 'differential_drive_robot.xacro'])
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'rviz_config.rviz']),
        description='Path to RViz configuration file'
    )
    
    # Robot State Publisher Node
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(Command(['xacro ', urdf_file_path]), value_type=str)
        }]
    )
    
    # Joint State Publisher Node
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])
