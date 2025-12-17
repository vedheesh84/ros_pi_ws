#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Package directories
    arm_moveit_dir = get_package_share_directory('arm_moveit')
    arm_controller_dir = get_package_share_directory('arm_controller')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz with MoveIt')
    
    # MoveIt Demo Launch (includes move_group + RViz)
    moveit_demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_moveit_dir, 'launch', 'demo.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'rviz': use_rviz
        }.items()
    )
    
    # Arm Hardware Controller with Trajectory Bridge
    arm_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_controller_dir, 'launch', 'arm_control.launch.py')
        ),
        launch_arguments={
            'use_trajectory_bridge': 'true'
        }.items()
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    
    # Add launches
    ld.add_action(moveit_demo_launch)
    ld.add_action(arm_control_launch)
    
    return ld
