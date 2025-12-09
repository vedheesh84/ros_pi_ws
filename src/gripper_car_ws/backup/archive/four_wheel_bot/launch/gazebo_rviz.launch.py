#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
import xacro


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([FindPackageShare('four_wheel_bot'), 'config', 'rviz_config.rviz']),
        description='Path to RViz configuration file'
    )

    # Robot name and package info
    robotXacroName = 'differential_drive_robot'
    namePackage = 'four_wheel_bot'
    modelFileRelativePath = 'urdf/differential_drive_robot.xacro'
    worldFileRelativePath = 'worlds/empty_world.world'

    # Get package directory and file paths
    pkg_share = FindPackageShare(namePackage)
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)
    
    # Process URDF file
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # Gazebo launch file
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={'world': pathWorldFile}.items()
    )

    # Spawn robot in Gazebo at z=0.2 to ensure wheels are above ground
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName,
                   '-x', '0.0', '-y', '0.0', '-z', '0.2'],
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}]
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robotDescription,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # RViz Node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Joint State Publisher GUI Node (for manual joint control)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        rviz_config_arg,
        gazeboLaunch,
        spawnModelNode,
        robot_state_publisher_node,
        rviz_node,
        joint_state_publisher_node
    ])
