#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():

    pkg_share = FindPackageShare('mobile_arm_manipulator_config')

    urdf_file_path = PathJoinSubstitution([
        pkg_share,
        'urdf',
        'mobile_arm_bot_2.urdf.xacro'
    ])

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )



    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_file_path]),
                value_type=str
            )
        }]
    )

    

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        robot_state_publisher_node,
        rviz_node
    ])
