#!/usr/bin/env python3
"""
Gazebo Simulation Launch
=========================
Launch Gazebo simulation with the mobile manipulator.

Usage:
    ros2 launch mobile_manipulator_bringup simulation.launch.py
    ros2 launch mobile_manipulator_bringup simulation.launch.py world:=/path/to/world.sdf
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_bringup = FindPackageShare('mobile_manipulator_bringup')
    pkg_description = FindPackageShare('mobile_manipulator_description')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    use_rviz = LaunchConfiguration('use_rviz')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # URDF
    urdf_file = PathJoinSubstitution([pkg_description, 'urdf', 'mobile_manipulator.urdf.xacro'])
    robot_description = ParameterValue(Command(['xacro ', urdf_file, ' use_sim:=true']), value_type=str)

    # RViz config
    rviz_config = PathJoinSubstitution([pkg_bringup, 'config', 'simulation.rviz'])

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('world', default_value=''),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),

        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
            ]),
            launch_arguments={'world': world}.items(),
        ),

        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': True
            }],
            output='screen'
        ),

        # Spawn Robot
        TimerAction(
            period=2.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    name='spawn_entity',
                    arguments=[
                        '-entity', 'mobile_manipulator',
                        '-topic', 'robot_description',
                        '-x', x_pose,
                        '-y', y_pose,
                        '-z', '0.1'
                    ],
                    output='screen'
                ),
            ]
        ),

        # RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])
