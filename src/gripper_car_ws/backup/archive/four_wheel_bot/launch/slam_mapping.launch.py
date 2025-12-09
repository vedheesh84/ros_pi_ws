#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directories
    pkg_share = FindPackageShare('four_wheel_bot')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

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
        default_value=PathJoinSubstitution([pkg_share, 'config', 'mapping_params.yaml']),
        description='Full path to the ROS2 parameters file for slam_toolbox'
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

    # SLAM Toolbox mapping launch include
    slam_launch_file = os.path.join(
        slam_toolbox_dir,
        'launch',
        'online_async_launch.py'
    )

    slam_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(slam_launch_file),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': LaunchConfiguration('slam_params_file')
        }.items()
    )

    # Launch description
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(slam_params_file_arg)
    ld.add_action(slam_include)
    ld.add_action(rviz_node)

    return ld
