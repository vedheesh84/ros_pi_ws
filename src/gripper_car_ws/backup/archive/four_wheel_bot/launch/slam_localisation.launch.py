#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    pkg_share = get_package_share_directory('four_wheel_bot')

    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'rviz_config.rviz']),
        description='Path to RViz configuration file'
    )

    slam_params_file_arg = DeclareLaunchArgument(
        'slam_params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'localisation_params.yaml']),
        description='Full path to the parameters file for slam_toolbox'
    )

    # SLAM Toolbox localization node (direct node launch — not Include)
    slam_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            LaunchConfiguration('slam_params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Launch description
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(slam_localization_node)
    ld.add_action(rviz_node)

    return ld
