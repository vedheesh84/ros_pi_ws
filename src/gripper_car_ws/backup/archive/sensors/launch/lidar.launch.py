#!/usr/bin/env python3
"""LiDAR Launch - supports RPLIDAR, YDLIDAR, LDLIDAR"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    lidar_type = LaunchConfiguration('lidar_type').perform(context)
    port = LaunchConfiguration('port').perform(context)
    frame_id = LaunchConfiguration('frame_id').perform(context)

    nodes = []

    if lidar_type.lower() == 'rplidar':
        nodes.append(Node(
            package='rplidar_ros',
            executable='rplidar_node',
            parameters=[{
                'serial_port': port,
                'serial_baudrate': 115200,
                'frame_id': frame_id,
                'angle_compensate': True,
            }]
        ))
    elif lidar_type.lower() == 'ydlidar':
        nodes.append(Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            parameters=[{
                'port': port,
                'baudrate': 128000,
                'frame_id': frame_id,
            }]
        ))
    elif lidar_type.lower() in ['ldlidar', 'ld06']:
        nodes.append(Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            parameters=[{
                'serial_port_name': port,
                'frame_id': frame_id,
            }]
        ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('lidar_type', default_value='rplidar'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('frame_id', default_value='laser_frame'),
        OpaqueFunction(function=launch_setup),
    ])
