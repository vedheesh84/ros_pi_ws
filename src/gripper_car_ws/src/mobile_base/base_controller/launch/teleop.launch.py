#!/usr/bin/env python3
"""
Teleop Launch
==============

Launch teleop keyboard node for manual robot control.

Usage:
    ros2 launch base_controller teleop.launch.py
    ros2 launch base_controller teleop.launch.py linear_speed:=0.5
"""

import os
import shutil

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Package directory
    pkg_controller = get_package_share_directory('base_controller')

    # Paths
    params_file = os.path.join(pkg_controller, 'config', 'base_params.yaml')

    # Try to detect an available terminal emulator. If found, use it as a
    # prefix so the teleop node can receive keyboard input in a dedicated window.
    prefix_value = None
    for cmd in ('xterm', 'x-terminal-emulator', 'konsole', 'xfce4-terminal', 'gnome-terminal'):
        path = shutil.which(cmd)
        if path:
            # Most terminals accept '-e' to execute a command; gnome-terminal prefers '--'
            if os.path.basename(path) == 'gnome-terminal':
                prefix_value = f"{path} --"
            else:
                prefix_value = f"{path} -e"
            break

    node_kwargs = {
        'package': 'base_controller',
        'executable': 'teleop_keyboard.py',
        'name': 'teleop_keyboard',
        'output': 'screen',
        'parameters': [
            params_file,
            {
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
            }
        ],
    }

    if prefix_value:
        node_kwargs['prefix'] = prefix_value  # run in separate terminal for keyboard input

    return LaunchDescription([
        # ==================== ARGUMENTS ====================

        DeclareLaunchArgument(
            'linear_speed',
            default_value='0.3',
            description='Initial linear speed in m/s'
        ),

        DeclareLaunchArgument(
            'angular_speed',
            default_value='0.5',
            description='Initial angular speed in rad/s'
        ),

        # ==================== TELEOP NODE ====================

        Node(**node_kwargs),
    ])
