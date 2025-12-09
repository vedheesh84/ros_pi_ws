#!/usr/bin/env python3
"""
YDLIDAR X2 2D LiDAR Launch File
Optimized for YDLIDAR X2 360° 8m ROS Compatible sensor.

Usage:
    ros2 launch lidar_driver lidar.launch.py
    ros2 launch lidar_driver lidar.launch.py lidar_port:=/dev/ttyUSB0 lidar_frame:=lidar_link
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    lidar_port = LaunchConfiguration('lidar_port').perform(context)
    lidar_frame = LaunchConfiguration('lidar_frame').perform(context)

    # YDLIDAR X2 optimized configuration
    # Specifications:
    #   - 360° scan, 2D LiDAR
    #   - Max range: 8 meters
    #   - Single channel (no multi-layer)
    #   - Recommended frequency: 5–8 Hz
    #   - Baud rate: 115200

    ydlidar_node = Node(
        package='ydlidar_ros2',
        executable='ydlidar_node',
        name='ydlidar_x2_node',
        parameters=[{
            'port': lidar_port,
            'baudrate': 115200,              # YDLIDAR X2 standard baud rate
            'frame_id': lidar_frame,
            'singleChannel': True,           # X2 is single-channel 2D LiDAR
            'resolution_fixed': True,        # Fixed resolution mode
            'auto_reconnect': True,          # Auto-reconnect on disconnection
            'reversion': False,              # No inverted/reversed scanning
            'isToFLidar': False,             # Not a ToF (Time-of-Flight) LiDAR
            'angle_min': -3.14159,           # Full 360° coverage (−π to +π)
            'angle_max': 3.14159,
            'range_min': 0.1,                # Minimum range: 0.1 m
            'range_max': 8.0,                # Maximum range: 8 m (X2 spec)
            'frequency': 7.0,                # Scan frequency: 7 Hz (middle of 5–8 Hz recommended)
            'ignore_array': "",              # No ignored angles
            'intensities': False,            # Disable intensity values if not needed (reduces bandwidth)
        }],
        output='screen'
    )

    return [ydlidar_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_port',
            default_value='/dev/ttyUSB0',
            description='Serial port for YDLIDAR X2 connection'
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='laser_frame',
            description='TF frame ID for LiDAR scans'
        ),
        OpaqueFunction(function=launch_setup),
    ])
