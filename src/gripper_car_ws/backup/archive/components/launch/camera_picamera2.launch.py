#!/usr/bin/env python3
"""
Pi Camera 2 Launch File using camera_ros (picamera2 backend)
=============================================================
For Raspberry Pi with Ubuntu Server where libcamera doesn't work directly.
Uses camera_ros package with picamera2 backend.

Prerequisites (install via pip on Raspberry Pi):
    pip3 install picamera2
    sudo apt install python3-libcamera  # May need manual installation

Alternative: Use v4l2_camera if picamera2 doesn't work:
    ros2 launch components camera.launch.py

Usage:
    ros2 launch components camera_picamera2.launch.py
    ros2 launch components camera_picamera2.launch.py width:=1280 height:=720
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    width = LaunchConfiguration('width', default='640')
    height = LaunchConfiguration('height', default='480')
    fps = LaunchConfiguration('fps', default='30')
    camera_frame = LaunchConfiguration('camera_frame', default='camera_link_optical')

    declared_arguments = [
        DeclareLaunchArgument('width', default_value='640',
                              description='Image width'),
        DeclareLaunchArgument('height', default_value='480',
                              description='Image height'),
        DeclareLaunchArgument('fps', default_value='30',
                              description='Frames per second'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link_optical',
                              description='Camera optical frame'),
    ]

    # camera_ros node with picamera2 backend
    # Note: camera_ros automatically detects picamera2 on Raspberry Pi
    camera_ros_node = Node(
        package='camera_ros',
        executable='camera_node',
        name='picamera',
        namespace='camera',
        parameters=[{
            'width': 640,
            'height': 480,
            'format': 'RGB888',  # or 'YUYV', 'MJPEG'
            'frame_id': 'camera_link_optical',
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    # Alternative: v4l2_camera for generic USB cameras or Pi Camera as /dev/video0
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [640, 480],
            'camera_frame_id': 'camera_link_optical',
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
            'framerate': 30.0,
            'io_method': 'mmap',
            # Camera calibration (optional)
            # 'camera_info_url': 'file:///path/to/camera_info.yaml',
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info'),
        ],
        output='screen'
    )

    # Image transport for compressed images (reduce bandwidth)
    image_republisher = Node(
        package='image_transport',
        executable='republish',
        name='image_republisher',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/image_raw'),
            ('out/compressed', '/camera/image_raw/compressed'),
        ],
        output='screen'
    )

    return LaunchDescription(
        declared_arguments + [
            # Use v4l2_camera by default (more reliable on Ubuntu Server)
            v4l2_camera_node,
            # Uncomment below and comment above if using camera_ros with picamera2:
            # camera_ros_node,
            image_republisher,
        ]
    )
