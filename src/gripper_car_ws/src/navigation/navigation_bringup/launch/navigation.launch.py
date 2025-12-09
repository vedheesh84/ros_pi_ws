#!/usr/bin/env python3
"""
Nav2 Navigation Launch File
============================
Modular, plug-and-play navigation stack with SLAM support.

Usage:
    # SLAM mode (mapping)
    ros2 launch nav2_bringup navigation.launch.py slam:=true

    # Localization mode (with existing map)
    ros2 launch nav2_bringup navigation.launch.py slam:=false map:=/path/to/map.yaml

    # Simulation mode
    ros2 launch nav2_bringup navigation.launch.py use_sim_time:=true

Parameters:
    slam: Enable SLAM mode (default: false)
    map: Path to map YAML file (required if slam:=false)
    use_sim_time: Use simulation time (default: false)
    base_frame: Robot base frame (default: body_link)
    odom_frame: Odometry frame (default: odom)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('nav2_bringup')

    # Launch configurations
    slam = LaunchConfiguration('slam')
    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    base_frame = LaunchConfiguration('base_frame')
    odom_frame = LaunchConfiguration('odom_frame')

    # Config files
    nav2_params_file = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    slam_params_file = PathJoinSubstitution([pkg_share, 'config', 'slam_params.yaml'])

    # SLAM Node (mapping mode)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[slam_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(slam),
        output='screen'
    )

    # Map Server (localization mode)
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{'yaml_filename': map_yaml}, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(slam),
        output='screen'
    )

    # AMCL (localization mode)
    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        condition=UnlessCondition(slam),
        output='screen'
    )

    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Smoother Server
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Lifecycle Manager for Navigation
    lifecycle_manager_nav = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'smoother_server',
                'velocity_smoother',
                'waypoint_follower'
            ]
        }],
        output='screen'
    )

    # Lifecycle Manager for Localization
    lifecycle_manager_loc = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': ['map_server', 'amcl']
        }],
        condition=UnlessCondition(slam),
        output='screen'
    )

    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument('slam', default_value='false',
            description='Enable SLAM mode for mapping'),
        DeclareLaunchArgument('map', default_value='',
            description='Path to map YAML file'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
            description='Use simulation time'),
        DeclareLaunchArgument('base_frame', default_value='body_link',
            description='Robot base frame'),
        DeclareLaunchArgument('odom_frame', default_value='odom',
            description='Odometry frame'),

        # Nodes
        slam_node,
        map_server,
        amcl,
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        smoother_server,
        velocity_smoother,
        waypoint_follower,
        lifecycle_manager_nav,
        lifecycle_manager_loc,
    ])
