#!/usr/bin/env python3
"""
Camera Driver Launch File
=========================
Usage:
    ros2 launch camera_driver camera.launch.py
    ros2 launch camera_driver camera.launch.py video_device:=/dev/video0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link_optical'),
        DeclareLaunchArgument('width', default_value='1280'),
        DeclareLaunchArgument('height', default_value='720'),
        DeclareLaunchArgument('fps', default_value='30'),

        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[{
                'video_device': LaunchConfiguration('video_device'),
                'image_size': [1280, 720],
                'camera_frame_id': LaunchConfiguration('camera_frame'),
                'pixel_format': 'YUYV',
                'output_encoding': 'rgb8',
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
            ],
            output='screen'
        ),
    ])
