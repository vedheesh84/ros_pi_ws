#!/usr/bin/env python3

#arduino_bridge.launch.py
"""
Launch file for Arduino ROS2 Bridge
This launch file starts the Arduino communication bridge
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino communication (e.g., /dev/ttyUSB0, /dev/ttyACM0)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate for Arduino communication'
    )
    
    timeout_arg = DeclareLaunchArgument(
        'timeout',
        default_value='1.0',
        description='Serial communication timeout in seconds'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='50.0',
        description='Rate for publishing sensor data (Hz)'
    )
    
    emergency_stop_enabled_arg = DeclareLaunchArgument(
        'emergency_stop_enabled',
        default_value='true',
        description='Enable emergency stop functionality'
    )
    
    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    timeout = LaunchConfiguration('timeout')
    publish_rate = LaunchConfiguration('publish_rate')
    emergency_stop_enabled = LaunchConfiguration('emergency_stop_enabled')
    
    # Arduino ROS2 Bridge Node
    arduino_bridge_node = Node(
        package='mobile_arm_manipulator_config',
        executable='arduino_ros2_bridge.py',
        name='arduino_ros2_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'timeout': timeout,
            'publish_rate': publish_rate,
            'emergency_stop_enabled': emergency_stop_enabled,
        }],
        remappings=[
            ('/arduino/joint_states', '/joint_states'),
            ('/arduino/servo_commands', '/joint_trajectory_controller/joint_trajectory'),
        ]
    )
    
    # Information message
    info_message = LogInfo(
        msg=[
            'Arduino ROS2 Bridge starting with:',
            '  Serial Port: ', serial_port,
            '  Baud Rate: ', baud_rate,
            '  Publish Rate: ', publish_rate, ' Hz'
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        timeout_arg,
        publish_rate_arg,
        emergency_stop_enabled_arg,
        info_message,
        arduino_bridge_node,
    ])
