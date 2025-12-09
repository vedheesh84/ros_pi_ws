#!/usr/bin/env python3
"""
LiDAR Driver Launch File
=========================
Modular, plug-and-play LiDAR driver supporting multiple sensor types.

Usage:
    ros2 launch lidar_driver lidar.launch.py
    ros2 launch lidar_driver lidar.launch.py lidar_type:=rplidar
    ros2 launch lidar_driver lidar.launch.py lidar_type:=ydlidar lidar_port:=/dev/ttyUSB0
    ros2 launch lidar_driver lidar.launch.py lidar_type:=ldlidar

Parameters:
    lidar_type: rplidar (default), ydlidar, ldlidar/ld06
    lidar_port: Serial port (default: /dev/ttyUSB0)
    lidar_frame: TF frame ID (default: laser_frame)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    lidar_type = LaunchConfiguration('lidar_type').perform(context)
    lidar_port = LaunchConfiguration('lidar_port').perform(context)
    lidar_frame = LaunchConfiguration('lidar_frame').perform(context)

    nodes = []

    if lidar_type.lower() == 'rplidar':
        # RPLIDAR A1/A2/A3 configuration
        rplidar_node = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': lidar_frame,
                'inverted': False,
                'angle_compensate': True,
                'scan_mode': 'Standard',  # 'Express' for A2/A3
            }],
            output='screen'
        )
        nodes.append(rplidar_node)

    elif lidar_type.lower() == 'ydlidar':
        # YDLIDAR X2/X4/G2/G4 configuration
        ydlidar_node = Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_node',
            parameters=[{
                'port': lidar_port,
                'baudrate': 128000,  # 115200 for X2, 128000 for X4
                'frame_id': lidar_frame,
                'fixed_resolution': True,
                'reversion': False,
                'inverted': True,
                'auto_reconnect': True,
                'angle_min': -180.0,
                'angle_max': 180.0,
                'range_min': 0.1,
                'range_max': 12.0,
                'frequency': 10.0,
                'samp_rate': 5,
                'isSingleChannel': False,  # True for X2
                'lidar_type': 1,  # 0=G2, 1=G4, 2=X2, 3=X4
            }],
            remappings=[('/scan', '/scan')],
            output='screen'
        )
        nodes.append(ydlidar_node)

    elif lidar_type.lower() in ['ldlidar', 'ld06']:
        # LDLIDAR LD06/LD19 configuration
        ldlidar_node = Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ldlidar_node',
            parameters=[{
                'product_name': 'LDLiDAR_LD06',  # or LDLiDAR_LD19
                'serial_port_name': lidar_port,
                'serial_baudrate': 230400,
                'frame_id': lidar_frame,
                'laser_scan_dir': True,
                'enable_angle_crop_func': False,
                'angle_crop_min': 135.0,
                'angle_crop_max': 225.0,
            }],
            output='screen'
        )
        nodes.append(ldlidar_node)

    else:
        # Fallback: RPLIDAR
        rplidar_node = Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'serial_port': lidar_port,
                'serial_baudrate': 115200,
                'frame_id': lidar_frame,
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen'
        )
        nodes.append(rplidar_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_type',
            default_value='rplidar',
            description='LiDAR type: rplidar, ydlidar, ldlidar, ld06'
        ),
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='LiDAR serial port'
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='laser_frame',
            description='LiDAR TF frame ID'
        ),
        OpaqueFunction(function=launch_setup),
    ])
