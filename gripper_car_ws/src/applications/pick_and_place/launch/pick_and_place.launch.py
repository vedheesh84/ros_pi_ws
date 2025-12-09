#!/usr/bin/env python3
"""
Pick and Place System Launch
=============================
Launch the complete pick-and-place system with QR code scanning.

This launch file starts:
- QR scanner node (for scanning object QR codes)
- Pick and place orchestrator (main state machine)

Prerequisites:
    Before launching this, ensure the robot system is running:

    ros2 launch mobile_manipulator_bringup bringup.launch.py \\
        use_moveit:=true use_nav:=true use_sensors:=true

Usage:
    # Launch pick-and-place system
    ros2 launch pick_and_place pick_and_place.launch.py

    # With custom configuration
    ros2 launch pick_and_place pick_and_place.launch.py \\
        continuous_mode:=false

    # Start the cycle
    ros2 topic pub /pick_and_place/command std_msgs/String "data: 'start'" --once

    # Monitor state
    ros2 topic echo /pick_and_place/state

    # Stop/pause/resume
    ros2 topic pub /pick_and_place/command std_msgs/String "data: 'stop'" --once
    ros2 topic pub /pick_and_place/command std_msgs/String "data: 'pause'" --once
    ros2 topic pub /pick_and_place/command std_msgs/String "data: 'resume'" --once

Configuration Files:
    config/locations.yaml      - Named locations (QR content -> coordinates)
    config/arm_poses.yaml      - Custom arm poses for pick/scan/place
    config/pick_and_place.yaml - Timeouts, retries, behavior settings
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package path
    pkg_share = FindPackageShare('pick_and_place')

    # ==========================================================================
    # Launch Arguments
    # ==========================================================================

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/camera/image_raw',
        description='Camera image topic for QR scanning'
    )

    continuous_mode_arg = DeclareLaunchArgument(
        'continuous_mode',
        default_value='true',
        description='Run in continuous loop mode'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # ==========================================================================
    # Nodes
    # ==========================================================================

    # QR Scanner Node
    qr_scanner_node = Node(
        package='pick_and_place',
        executable='qr_scanner_node.py',
        name='qr_scanner_node',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'continuous_mode': False,  # Service mode for pick-and-place
            'publish_debug': True,
            'min_qr_size': 50,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
        emulate_tty=True,
    )

    # Pick and Place Orchestrator Node
    pick_and_place_node = Node(
        package='pick_and_place',
        executable='pick_and_place_node.py',
        name='pick_and_place_node',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        # Arguments
        camera_topic_arg,
        continuous_mode_arg,
        use_sim_time_arg,

        # Nodes
        qr_scanner_node,
        pick_and_place_node,
    ])
