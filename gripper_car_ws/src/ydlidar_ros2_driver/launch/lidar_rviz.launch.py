#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    package_name = 'ydlidar_ros2_driver'
    pkg_share_dir = get_package_share_directory(package_name)

    # URDF Path
    urdf_path = os.path.join(pkg_share_dir, 'urdf', 'lidar.xacro')

    # Robot Description (xacro ⇒ xml)
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    # RViz2 config
    rviz_config_file = os.path.join(pkg_share_dir, 'config', 'ydlidar.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
