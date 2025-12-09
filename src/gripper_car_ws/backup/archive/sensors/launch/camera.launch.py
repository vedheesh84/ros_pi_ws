#!/usr/bin/env python3
"""Camera Launch - v4l2 camera driver"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    device = LaunchConfiguration('device', default='/dev/video0')

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        parameters=[{
            'video_device': device,
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link_optical',
            'pixel_format': 'YUYV',
            'framerate': 30.0,
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('device', default_value='/dev/video0'),
        camera_node,
    ])
