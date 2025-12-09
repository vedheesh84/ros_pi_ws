#!/usr/bin/env python3
"""
Robot Bringup Launch - Main entry point for complete mobile manipulator

Integrates all subsystems:
- Mobile base (URDF, robot_state_publisher)
- Arm (URDF integrated with base)
- MoveIt (arm motion planning)
- Nav2 (mobile base navigation)
- Sensors (camera, LiDAR)
- Teleop (keyboard control)

Usage:
  # Simulation mode
  ros2 launch robot_bringup bringup.launch.py use_sim:=true

  # Hardware mode
  ros2 launch robot_bringup bringup.launch.py use_sim:=false

  # With specific components
  ros2 launch robot_bringup bringup.launch.py enable_moveit:=true enable_nav2:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
    PythonExpression,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_robot_bringup = FindPackageShare('robot_bringup')
    pkg_arm_moveit = FindPackageShare('arm_moveit')
    pkg_mobile_base_nav2 = FindPackageShare('mobile_base_nav2')
    pkg_sensors = FindPackageShare('sensors')
    pkg_teleop = FindPackageShare('teleop')
    pkg_arduino_bridge = FindPackageShare('arduino_bridge')

    # Launch configurations
    use_sim = LaunchConfiguration('use_sim')
    enable_moveit = LaunchConfiguration('enable_moveit')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    enable_teleop = LaunchConfiguration('enable_teleop')
    enable_rviz = LaunchConfiguration('enable_rviz')

    # Robot description from combined URDF
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_robot_bringup, 'config', 'robot.urdf.xacro']),
        ' use_sim:=', use_sim
    ])

    robot_description = {'robot_description': robot_description_content}

    # Declare arguments
    declare_use_sim = DeclareLaunchArgument(
        'use_sim',
        default_value='false',
        description='Use simulation (Gazebo) or real hardware'
    )

    declare_enable_moveit = DeclareLaunchArgument(
        'enable_moveit',
        default_value='true',
        description='Enable MoveIt for arm motion planning'
    )

    declare_enable_nav2 = DeclareLaunchArgument(
        'enable_nav2',
        default_value='false',
        description='Enable Nav2 for mobile base navigation'
    )

    declare_enable_teleop = DeclareLaunchArgument(
        'enable_teleop',
        default_value='true',
        description='Enable keyboard teleop'
    )

    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz visualization'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Joint State Publisher (for testing without controllers)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=IfCondition(use_sim),
        parameters=[robot_description]
    )

    # RViz
    rviz_config = PathJoinSubstitution([
        pkg_robot_bringup, 'config', 'robot.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(enable_rviz),
        output='screen'
    )

    # MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_arm_moveit, 'launch', 'moveit.launch.py'])
        ]),
        condition=IfCondition(enable_moveit),
        launch_arguments={
            'use_sim_time': use_sim,
            'use_robot_bringup': 'true',
            'rviz': 'false',
        }.items()
    )

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_mobile_base_nav2, 'launch', 'navigation.launch.py'])
        ]),
        condition=IfCondition(enable_nav2),
        launch_arguments={
            'use_sim': use_sim,
        }.items()
    )

    # Teleop
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_teleop, 'launch', 'teleop.launch.py'])
        ]),
        condition=IfCondition(enable_teleop)
    )

    # Hardware bridge (only when not in simulation)
    hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_arduino_bridge, 'launch', 'hardware.launch.py'])
        ]),
        condition=UnlessCondition(use_sim)
    )

    # Sensors (camera and lidar)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_sensors, 'launch', 'camera.launch.py'])
        ]),
        condition=UnlessCondition(use_sim)
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_sensors, 'launch', 'lidar.launch.py'])
        ]),
        condition=UnlessCondition(use_sim)
    )

    return LaunchDescription([
        # Arguments
        declare_use_sim,
        declare_enable_moveit,
        declare_enable_nav2,
        declare_enable_teleop,
        declare_enable_rviz,

        # Core
        robot_state_publisher,
        joint_state_publisher,
        rviz_node,

        # Subsystems
        moveit_launch,
        nav2_launch,
        teleop_launch,

        # Hardware (only for real robot)
        hardware_launch,
        camera_launch,
        lidar_launch,
    ])
