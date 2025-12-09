#!/usr/bin/env python3
"""
Camera Driver Launch File
=========================
Modular, plug-and-play camera driver supporting multiple sensor types.

Usage:
    ros2 launch camera_driver camera.launch.py
    ros2 launch camera_driver camera.launch.py camera_type:=v4l2
    ros2 launch camera_driver camera.launch.py camera_type:=v4l2 video_device:=/dev/video0
    ros2 launch camera_driver camera.launch.py width:=1280 height:=720 fps:=30

Parameters:
    camera_type: v4l2 (default), picamera2
    video_device: Camera device (default: /dev/video0)
    camera_frame: TF frame ID (default: camera_link_optical)
    width: Image width (default: 1280)
    height: Image height (default: 720)
    fps: Framerate (default: 30)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    camera_type = LaunchConfiguration('camera_type').perform(context)
    video_device = LaunchConfiguration('video_device').perform(context)
    camera_frame = LaunchConfiguration('camera_frame').perform(context)
    width = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    fps = float(LaunchConfiguration('fps').perform(context))

    nodes = []

    if camera_type.lower() == 'v4l2':
        # V4L2 camera (USB cameras, CSI via V4L2)
        v4l2_camera_node = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[{
                'video_device': video_device,
                'image_size': [width, height],
                'camera_frame_id': camera_frame,
                'pixel_format': 'YUYV',
                'output_encoding': 'rgb8',
                'framerate': fps,
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info'),
            ],
            output='screen'
        )
        nodes.append(v4l2_camera_node)

    elif camera_type.lower() == 'picamera2':
        # PiCamera2 (Raspberry Pi camera module)
        # Note: Requires picamera2 ROS2 package
        picamera_node = Node(
            package='v4l2_camera',  # Using V4L2 for libcamera compatibility
            executable='v4l2_camera_node',
            name='picamera2',
            parameters=[{
                'video_device': video_device,
                'image_size': [width, height],
                'camera_frame_id': camera_frame,
                'pixel_format': 'YUYV',
                'output_encoding': 'rgb8',
                'framerate': fps,
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
                ('/camera_info', '/camera/camera_info'),
            ],
            output='screen'
        )
        nodes.append(picamera_node)

    else:
        # Default: V4L2
        v4l2_camera_node = Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera',
            parameters=[{
                'video_device': video_device,
                'image_size': [width, height],
                'camera_frame_id': camera_frame,
                'pixel_format': 'YUYV',
                'output_encoding': 'rgb8',
                'framerate': fps,
            }],
            remappings=[
                ('/image_raw', '/camera/image_raw'),
            ],
            output='screen'
        )
        nodes.append(v4l2_camera_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_type',
            default_value='v4l2',
            description='Camera type: v4l2, picamera2'
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video0',
            description='Camera device path'
        ),
        DeclareLaunchArgument(
            'camera_frame',
            default_value='camera_link_optical',
            description='Camera TF frame ID'
        ),
        DeclareLaunchArgument(
            'width',
            default_value='1280',
            description='Image width in pixels'
        ),
        DeclareLaunchArgument(
            'height',
            default_value='720',
            description='Image height in pixels'
        ),
        DeclareLaunchArgument(
            'fps',
            default_value='30',
            description='Framerate'
        ),
        OpaqueFunction(function=launch_setup),
    ])
