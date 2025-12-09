#!/usr/bin/env python3
"""
Mobile Manipulator Hardware Bringup for Raspberry Pi
=====================================================
This launch file brings up all hardware components for the mobile manipulator:
- Mobile base control (Arduino Mega via serial)
- Robotic arm servo control (PCA9685)
- Camera (v4l2_camera for Ubuntu Server - no libcamera)
- LiDAR (RPLIDAR or YDLIDAR)
- Robot state publisher
- Joint state publisher (for arm feedback)

Usage:
    ros2 launch components hardware_bringup.launch.py
    ros2 launch components hardware_bringup.launch.py lidar_type:=ydlidar
    ros2 launch components hardware_bringup.launch.py camera:=false lidar:=false
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package paths
    pkg_components = get_package_share_directory('components')
    pkg_mobile_arm = get_package_share_directory('mobile_arm_manipulator_config')

    # URDF file
    urdf_file = os.path.join(pkg_components, 'urdf', 'robot.urdf.xacro')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Camera arguments
    enable_camera = LaunchConfiguration('camera', default='true')
    camera_device = LaunchConfiguration('camera_device', default='/dev/video0')
    camera_width = LaunchConfiguration('camera_width', default='640')
    camera_height = LaunchConfiguration('camera_height', default='480')
    camera_fps = LaunchConfiguration('camera_fps', default='30')

    # LiDAR arguments
    enable_lidar = LaunchConfiguration('lidar', default='true')
    lidar_type = LaunchConfiguration('lidar_type', default='rplidar')  # rplidar or ydlidar
    lidar_port = LaunchConfiguration('lidar_port', default='/dev/ttyUSB0')
    lidar_frame = LaunchConfiguration('lidar_frame', default='laser_frame')

    # Arduino arguments
    arduino_port = LaunchConfiguration('arduino_port', default='/dev/ttyACM0')
    arduino_baud = LaunchConfiguration('arduino_baud', default='115200')

    # Process URDF
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )

    # ==================== DECLARE LAUNCH ARGUMENTS ====================
    declared_arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='Use simulation clock'),
        DeclareLaunchArgument('camera', default_value='true',
                              description='Enable camera'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0',
                              description='Camera device path'),
        DeclareLaunchArgument('camera_width', default_value='640',
                              description='Camera image width'),
        DeclareLaunchArgument('camera_height', default_value='480',
                              description='Camera image height'),
        DeclareLaunchArgument('camera_fps', default_value='30',
                              description='Camera framerate'),
        DeclareLaunchArgument('lidar', default_value='true',
                              description='Enable LiDAR'),
        DeclareLaunchArgument('lidar_type', default_value='rplidar',
                              description='LiDAR type: rplidar or ydlidar'),
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0',
                              description='LiDAR serial port'),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame',
                              description='LiDAR TF frame'),
        DeclareLaunchArgument('arduino_port', default_value='/dev/ttyACM0',
                              description='Arduino serial port'),
        DeclareLaunchArgument('arduino_baud', default_value='115200',
                              description='Arduino baud rate'),
    ]

    # ==================== ROBOT STATE PUBLISHER ====================
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
            'publish_frequency': 50.0,
        }],
        output='screen'
    )

    # ==================== JOINT STATE PUBLISHER (GUI disabled for headless) ====================
    # This provides fake joint states for testing without hardware
    # In production, joint states come from the servo bridge
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'source_list': ['/arm/joint_states', '/base/joint_states'],
        }],
        output='screen'
    )

    # ==================== ARDUINO SERVO BRIDGE (ARM) ====================
    # This node provides joint feedback for the robotic arm (no hardware encoders)
    servo_bridge_node = Node(
        package='mobile_arm_manipulator_config',
        executable='arduino_ros2_bridge.py',
        name='servo_bridge',
        parameters=[{
            'serial_port': arduino_port,
            'baud_rate': arduino_baud,
        }],
        remappings=[
            ('/joint_states', '/arm/joint_states'),
        ],
        output='screen'
    )

    # ==================== ARDUINO BRIDGE (MOBILE BASE) ====================
    # This node handles wheel control and encoder feedback
    base_bridge_node = Node(
        package='ros2_arduino_bridge',
        executable='arduino_bridge',
        name='arduino_bridge',
        parameters=[{
            'port': arduino_port,
            'baud_rate': 57600,
            'wheel_radius': 0.05,
            'encoder_resolution': 360,
            'cmd_timeout': 0.30,
        }],
        remappings=[
            ('/joint_states', '/base/joint_states'),
        ],
        output='screen'
    )

    # ==================== V4L2 CAMERA (for Ubuntu Server on Raspberry Pi) ====================
    # Using v4l2_camera since libcamera doesn't work on Ubuntu Server
    # Alternative: camera_ros with picamera2
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        condition=IfCondition(enable_camera),
        parameters=[{
            'video_device': camera_device,
            'image_size': [640, 480],  # Use integers, not LaunchConfiguration
            'camera_frame_id': 'camera_link_optical',
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
            'framerate': 30.0,
            'io_method': 'mmap',
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    # ==================== RPLIDAR NODE ====================
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        condition=IfCondition(enable_lidar),
        parameters=[{
            'serial_port': lidar_port,
            'serial_baudrate': 115200,
            'frame_id': lidar_frame,
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Standard',
        }],
        output='screen'
    )

    # ==================== STATIC TRANSFORMS ====================
    # odom -> body_link is published by the arduino bridge (odometry)
    # These are static transforms for sensor mounts

    # If your base_link is named differently, adjust here
    static_tf_base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_laser',
        arguments=['0', '0', '0.16', '0', '0', '0', 'body_link', 'laser_frame'],
    )

    static_tf_base_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_camera',
        arguments=['0.275', '0', '0.058', '0', '0', '0', 'body_link', 'camera_link'],
    )

    # ==================== LAUNCH DESCRIPTION ====================
    return LaunchDescription(
        declared_arguments + [
            robot_state_publisher_node,
            joint_state_publisher_node,
            servo_bridge_node,
            # base_bridge_node,  # Uncomment if using separate Arduino for base
            v4l2_camera_node,
            rplidar_node,
            static_tf_base_to_laser,
            static_tf_base_to_camera,
        ]
    )
