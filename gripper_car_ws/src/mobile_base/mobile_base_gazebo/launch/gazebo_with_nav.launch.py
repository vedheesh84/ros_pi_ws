#!/usr/bin/env python3
"""
Gazebo + Navigation Launch
===========================

Launch Gazebo simulation with SLAM/Nav2 stack for testing navigation.

Usage:
    # SLAM mode (mapping)
    ros2 launch mobile_base_gazebo gazebo_with_nav.launch.py slam:=true

    # Localization mode (with existing map)
    ros2 launch mobile_base_gazebo gazebo_with_nav.launch.py slam:=false map:=/path/to/map.yaml

    # Different world
    ros2 launch mobile_base_gazebo gazebo_with_nav.launch.py world:=test_arena

Parameters:
    slam: Enable SLAM mode for mapping (default: true)
    map: Path to map YAML file (required if slam:=false)
    world: Gazebo world name (default: test_arena)
    x, y, yaw: Initial robot pose
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Package directories
    pkg_gazebo = get_package_share_directory('mobile_base_gazebo')
    pkg_description = get_package_share_directory('mobile_base_description')
    pkg_nav = get_package_share_directory('navigation_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Launch configurations
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    world_name = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x')
    y_pose = LaunchConfiguration('y')
    z_pose = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    # Paths
    urdf_file = os.path.join(pkg_description, 'urdf', 'mobile_base.urdf.xacro')
    gazebo_xacro = os.path.join(pkg_gazebo, 'urdf', 'gazebo.xacro')
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_nav, 'config', 'slam_params.yaml')
    nav_rviz = os.path.join(pkg_gazebo, 'config', 'nav.rviz')

    # Robot description with Gazebo plugins
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
            'slam',
            default_value='true',
            description='Enable SLAM mode for mapping'
        ),

        DeclareLaunchArgument(
            'map',
            default_value='',
            description='Path to map YAML file'
        ),

        DeclareLaunchArgument(
            'world',
            default_value='test_arena',
            description='Gazebo world name'
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
                'world': PythonExpression([
                    "'", pkg_gazebo, "/worlds/', '", world_name, "', '.world'"
                ]),
                'verbose': 'false'
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

        # ==================== SLAM (MAPPING MODE) ====================

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='slam_toolbox',
                    executable='async_slam_toolbox_node',
                    name='slam_toolbox',
                    output='screen',
                    parameters=[
                        slam_params,
                        {'use_sim_time': True}
                    ],
                    condition=IfCondition(slam)
                ),
            ]
        ),

        # ==================== MAP SERVER (LOCALIZATION MODE) ====================

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_map_server',
                    executable='map_server',
                    name='map_server',
                    output='screen',
                    parameters=[
                        {'yaml_filename': map_yaml},
                        {'use_sim_time': True}
                    ],
                    condition=UnlessCondition(slam)
                ),
            ]
        ),

        # AMCL
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_amcl',
                    executable='amcl',
                    name='amcl',
                    output='screen',
                    parameters=[
                        nav2_params,
                        {'use_sim_time': True}
                    ],
                    condition=UnlessCondition(slam)
                ),
            ]
        ),

        # ==================== NAV2 NODES ====================

        # Controller Server
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='nav2_controller',
                    executable='controller_server',
                    name='controller_server',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}]
                ),
            ]
        ),

        # Planner Server
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='nav2_planner',
                    executable='planner_server',
                    name='planner_server',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}]
                ),
            ]
        ),

        # Behavior Server
        TimerAction(
            period=6.0,
            actions=[
                Node(
                    package='nav2_behaviors',
                    executable='behavior_server',
                    name='behavior_server',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}]
                ),
            ]
        ),

        # BT Navigator
        TimerAction(
            period=7.0,
            actions=[
                Node(
                    package='nav2_bt_navigator',
                    executable='bt_navigator',
                    name='bt_navigator',
                    output='screen',
                    parameters=[nav2_params, {'use_sim_time': True}]
                ),
            ]
        ),

        # Lifecycle Manager for Navigation
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_navigation',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': [
                            'controller_server',
                            'planner_server',
                            'behavior_server',
                            'bt_navigator'
                        ]
                    }]
                ),
            ]
        ),

        # Lifecycle Manager for Localization
        TimerAction(
            period=8.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager',
                    executable='lifecycle_manager',
                    name='lifecycle_manager_localization',
                    output='screen',
                    parameters=[{
                        'use_sim_time': True,
                        'autostart': True,
                        'node_names': ['map_server', 'amcl']
                    }],
                    condition=UnlessCondition(slam)
                ),
            ]
        ),

        # ==================== RVIZ ====================

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', nav_rviz],
                    parameters=[{'use_sim_time': True}]
                ),
            ]
        ),
    ])
