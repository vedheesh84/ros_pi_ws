#!/usr/bin/env python3
"""
Hardware Launch - Real robot hardware bringup

Launches:
- Arduino hardware bridge
- Robot State Publisher
- Sensors (camera, LiDAR)
- Controllers
- Optionally: MoveIt, Nav2, Teleop

Usage:
  ros2 launch robot_bringup hardware.launch.py
  ros2 launch robot_bringup hardware.launch.py serial_port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
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
    pkg_arduino_bridge = FindPackageShare('arduino_bridge')
    pkg_teleop = FindPackageShare('teleop')

    # Launch configurations
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    lidar_type = LaunchConfiguration('lidar_type')
    lidar_port = LaunchConfiguration('lidar_port')
    camera_type = LaunchConfiguration('camera_type')
    enable_moveit = LaunchConfiguration('enable_moveit')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    enable_teleop = LaunchConfiguration('enable_teleop')
    enable_rviz = LaunchConfiguration('enable_rviz')

    # Robot description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_robot_bringup, 'config', 'robot.urdf.xacro']),
        ' use_sim:=false'
    ])

    robot_description = {'robot_description': robot_description_content}

    # Declare arguments
    declare_serial_port = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Arduino serial port'
    )

    declare_baud_rate = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )

    declare_lidar_type = DeclareLaunchArgument(
        'lidar_type',
        default_value='rplidar',
        description='LiDAR type: rplidar, ydlidar, ldlidar'
    )

    declare_lidar_port = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='LiDAR serial port'
    )

    declare_camera_type = DeclareLaunchArgument(
        'camera_type',
        default_value='usb',
        description='Camera type: usb, realsense, picamera'
    )

    declare_enable_moveit = DeclareLaunchArgument(
        'enable_moveit',
        default_value='true',
        description='Enable MoveIt'
    )

    declare_enable_nav2 = DeclareLaunchArgument(
        'enable_nav2',
        default_value='false',
        description='Enable Nav2'
    )

    declare_enable_teleop = DeclareLaunchArgument(
        'enable_teleop',
        default_value='true',
        description='Enable keyboard teleop'
    )

    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz'
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Arduino Hardware Bridge
    hardware_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_arduino_bridge, 'launch', 'hardware.launch.py'])
        ]),
        launch_arguments={
            'serial_port': serial_port,
            'baud_rate': baud_rate,
        }.items()
    )

    # LiDAR
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_sensors, 'launch', 'lidar.launch.py'])
        ]),
        launch_arguments={
            'lidar_type': lidar_type,
            'port': lidar_port,
        }.items()
    )

    # Camera
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_sensors, 'launch', 'camera.launch.py'])
        ]),
        launch_arguments={
            'camera_type': camera_type,
        }.items()
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
            'use_sim_time': 'false',
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
            'use_sim': 'false',
        }.items()
    )

    # Teleop
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_teleop, 'launch', 'teleop.launch.py'])
        ]),
        condition=IfCondition(enable_teleop)
    )

    return LaunchDescription([
        # Arguments
        declare_serial_port,
        declare_baud_rate,
        declare_lidar_type,
        declare_lidar_port,
        declare_camera_type,
        declare_enable_moveit,
        declare_enable_nav2,
        declare_enable_teleop,
        declare_enable_rviz,

        # Core
        robot_state_publisher,

        # Hardware
        hardware_bridge,
        lidar_launch,
        camera_launch,

        # Visualization
        rviz_node,

        # Subsystems
        moveit_launch,
        nav2_launch,
        teleop_launch,
    ])
