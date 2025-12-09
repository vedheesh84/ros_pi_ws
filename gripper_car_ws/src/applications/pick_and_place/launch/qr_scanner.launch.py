#!/usr/bin/env python3
"""
QR Scanner Standalone Launch
=============================
Launch the QR scanner node independently for testing or continuous mode.

This launch file starts only the QR scanner node, which can be used:
- For standalone QR code detection testing
- In continuous mode for real-time detection publishing
- To verify camera setup and QR detection before full system test

Usage:
    # Basic launch (service mode)
    ros2 launch pick_and_place qr_scanner.launch.py

    # With continuous mode (publishes to /qr_scanner/detections)
    ros2 launch pick_and_place qr_scanner.launch.py continuous_mode:=true

    # With custom camera topic
    ros2 launch pick_and_place qr_scanner.launch.py camera_topic:=/my_camera/image_raw

    # Test the service
    ros2 service call /qr_scanner/scan std_srvs/srv/Trigger

    # View debug image
    ros2 run rqt_image_view rqt_image_view /qr_scanner/debug_image

Dependencies:
    - Camera driver must be running and publishing to camera_topic
    - pyzbar library must be installed (sudo apt install libzbar0 && pip3 install pyzbar)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic to subscribe to'
    )

    continuous_mode_arg = DeclareLaunchArgument(
        'continuous_mode',
        default_value='false',
        description='Enable continuous QR detection mode'
    )

    publish_debug_arg = DeclareLaunchArgument(
        'publish_debug',
        default_value='true',
        description='Publish debug image with QR overlay'
    )

    min_qr_size_arg = DeclareLaunchArgument(
        'min_qr_size',
        default_value='50',
        description='Minimum QR code size in pixels'
    )

    # QR Scanner node
    qr_scanner_node = Node(
        package='pick_and_place',
        executable='qr_scanner_node.py',
        name='qr_scanner_node',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'continuous_mode': LaunchConfiguration('continuous_mode'),
            'publish_debug': LaunchConfiguration('publish_debug'),
            'min_qr_size': LaunchConfiguration('min_qr_size'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        # Arguments
        camera_topic_arg,
        continuous_mode_arg,
        publish_debug_arg,
        min_qr_size_arg,

        # Nodes
        qr_scanner_node,
    ])
