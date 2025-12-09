#!/usr/bin/env python3
"""
Mobile Manipulator Bringup Launch
==================================
Master launch file for the complete mobile manipulator system.

Usage:
    # Hardware mode (default)
    ros2 launch mobile_manipulator_bringup bringup.launch.py

    # Simulation mode
    ros2 launch mobile_manipulator_bringup bringup.launch.py use_sim:=true

    # With MoveIt
    ros2 launch mobile_manipulator_bringup bringup.launch.py use_moveit:=true

    # With Navigation
    ros2 launch mobile_manipulator_bringup bringup.launch.py use_nav:=true slam:=true

    # Full system
    ros2 launch mobile_manipulator_bringup bringup.launch.py use_moveit:=true use_nav:=true use_sensors:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_bringup = FindPackageShare('mobile_manipulator_bringup')
    pkg_description = FindPackageShare('mobile_manipulator_description')
    pkg_base_controller = FindPackageShare('base_controller')
    pkg_arm_controller = FindPackageShare('arm_controller')
    pkg_arm_moveit = FindPackageShare('arm_moveit')
    pkg_lidar = FindPackageShare('lidar_driver')
    pkg_camera = FindPackageShare('camera_driver')
    pkg_nav2 = FindPackageShare('navigation_bringup')

    # Launch configurations
    use_sim = LaunchConfiguration('use_sim')
    use_moveit = LaunchConfiguration('use_moveit')
    use_nav = LaunchConfiguration('use_nav')
    use_sensors = LaunchConfiguration('use_sensors')
    use_teleop = LaunchConfiguration('use_teleop')
    use_rviz = LaunchConfiguration('use_rviz')
    slam = LaunchConfiguration('slam')
    map_file = LaunchConfiguration('map')

    # URDF
    urdf_file = PathJoinSubstitution([pkg_description, 'urdf', 'mobile_manipulator.urdf.xacro'])
    robot_description = Command(['xacro ', urdf_file, ' use_sim:=', use_sim])

    # RViz config
    rviz_config = PathJoinSubstitution([pkg_bringup, 'config', 'bringup.rviz'])

    return LaunchDescription([
        # ==================== ARGUMENTS ====================
        DeclareLaunchArgument('use_sim', default_value='false',
            description='Use simulation (Gazebo) instead of hardware'),
        DeclareLaunchArgument('use_moveit', default_value='false',
            description='Enable MoveIt for arm planning'),
        DeclareLaunchArgument('use_nav', default_value='false',
            description='Enable Nav2 navigation stack'),
        DeclareLaunchArgument('use_sensors', default_value='true',
            description='Enable sensor drivers (LiDAR, camera)'),
        DeclareLaunchArgument('use_teleop', default_value='true',
            description='Enable keyboard teleop'),
        DeclareLaunchArgument('use_rviz', default_value='true',
            description='Launch RViz'),
        DeclareLaunchArgument('slam', default_value='false',
            description='Enable SLAM mode (requires use_nav:=true)'),
        DeclareLaunchArgument('map', default_value='',
            description='Path to map file (for localization mode)'),

        # ==================== ROBOT STATE PUBLISHER ====================
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim
            }],
            output='screen'
        ),

        # ==================== BASE CONTROLLER (Hardware Only) ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_base_controller, 'launch', 'base_control.launch.py'])
            ]),
            condition=UnlessCondition(use_sim),
        ),

        # ==================== ARM CONTROLLER (Hardware Only) ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_arm_controller, 'launch', 'arm_control.launch.py'])
            ]),
            condition=UnlessCondition(use_sim),
        ),

        # ==================== SENSORS (Hardware Only) ====================
        # LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_lidar, 'launch', 'lidar.launch.py'])
            ]),
            condition=IfCondition(PythonExpression([
                "'", use_sensors, "' == 'true' and '", use_sim, "' == 'false'"
            ])),
        ),

        # Camera
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_camera, 'launch', 'camera.launch.py'])
            ]),
            condition=IfCondition(PythonExpression([
                "'", use_sensors, "' == 'true' and '", use_sim, "' == 'false'"
            ])),
        ),

        # ==================== MOVEIT ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_arm_moveit, 'launch', 'moveit.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': use_sim,
            }.items(),
            condition=IfCondition(use_moveit),
        ),

        # ==================== NAVIGATION ====================
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_nav2, 'launch', 'navigation.launch.py'])
            ]),
            launch_arguments={
                'use_sim_time': use_sim,
                'slam': slam,
                'map': map_file,
            }.items(),
            condition=IfCondition(use_nav),
        ),

        # ==================== TELEOP ====================
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_twist_keyboard',
            prefix='xterm -e',
            condition=IfCondition(use_teleop),
            output='screen'
        ),

        # ==================== RVIZ ====================
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim}],
            condition=IfCondition(use_rviz),
            output='screen'
        ),
    ])
