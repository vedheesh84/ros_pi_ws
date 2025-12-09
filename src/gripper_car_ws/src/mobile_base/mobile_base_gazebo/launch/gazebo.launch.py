#!/usr/bin/env python3
"""
Gazebo Simulation Launch for Mobile Base
=========================================

Launch Gazebo simulation with the mobile base robot.

Usage:
    ros2 launch mobile_base_gazebo gazebo.launch.py
    ros2 launch mobile_base_gazebo gazebo.launch.py world:=test_arena
    ros2 launch mobile_base_gazebo gazebo.launch.py use_rviz:=true
    ros2 launch mobile_base_gazebo gazebo.launch.py x:=1.0 y:=2.0 yaw:=1.57
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_gazebo = get_package_share_directory('mobile_base_gazebo')
    pkg_description = get_package_share_directory('mobile_base_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    world_name = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'mobile_base.urdf.xacro')
    gazebo_xacro = os.path.join(pkg_gazebo, 'urdf', 'gazebo.xacro')
    rviz_config = os.path.join(pkg_gazebo, 'config', 'sim.rviz')

    # Robot description with Gazebo plugins included
    robot_description = ParameterValue(
        Command([
            'xacro ', urdf_file,
            ' gazebo_plugins:=true',
            ' gazebo_xacro_path:=', gazebo_xacro
        ]),
        value_type=str
    )

    return LaunchDescription([
        # ==================== ARGUMENTS ====================

        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2'
        ),

        DeclareLaunchArgument(
            'world',
            default_value='empty',
            description='World name (empty, test_arena)'
        ),

        DeclareLaunchArgument(
            'x',
            default_value='0.0',
            description='Initial X position'
        ),

        DeclareLaunchArgument(
            'y',
            default_value='0.0',
            description='Initial Y position'
        ),

        DeclareLaunchArgument(
            'z',
            default_value='0.1',
            description='Initial Z position'
        ),

        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial yaw angle'
        ),

        # ==================== GAZEBO ====================

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
            ]),
            launch_arguments={
                'world': [pkg_gazebo, '/worlds/', world_name, '.world'],
                'verbose': 'true'
            }.items(),
        ),

        # ==================== ROBOT STATE PUBLISHER ====================

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }]
        ),

        # ==================== SPAWN ROBOT ====================

        # Delay spawning to let Gazebo start
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_entity',
                    output='screen',
                    arguments=[
                        '-entity', 'mobile_base',
                        '-topic', 'robot_description',
                        '-x', x_pose,
                        '-y', y_pose,
                        '-z', z_pose,
                        '-Y', yaw
                    ]
                ),
            ]
        ),

        # ==================== RVIZ ====================

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz)
        ),
    ])
