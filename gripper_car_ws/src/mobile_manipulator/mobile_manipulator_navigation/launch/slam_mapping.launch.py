#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    
    # Package directories
    mobile_base_description_dir = get_package_share_directory('mobile_base_description')
    base_controller_dir = get_package_share_directory('base_controller')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    pkg_dir = get_package_share_directory('mobile_manipulator_navigation')
    
    # Paths
    xacro_file = os.path.join(mobile_base_description_dir, 'urdf', 'mobile_base.urdf.xacro')
    ydlidar_params_file = os.path.join(ydlidar_dir, 'params', 'X2L.yaml')
    slam_params_file = os.path.join(pkg_dir, 'config', 'mapping_params.yaml')
    rviz_config = os.path.join(mobile_base_description_dir, 'rviz', 'slam_mapping.rviz')
    
    # Process xacro to get robot description
    robot_description_content = xacro.process_file(xacro_file).toxml()
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')
    
    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for mobile base controller')
    
    # Robot State Publisher - publishes TF from URDF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher (if needed for arm joints)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Mobile Base Hardware Controller
    hardware_controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_controller_dir, 'launch', 'hardware.launch.py')
        ),
        launch_arguments={
            'serial_port': serial_port,
            'use_rviz': 'false'  # We'll launch our own RViz
        }.items()
    )
    
    # Teleop Launch
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(base_controller_dir, 'launch', 'teleop.launch.py')
        )
    )
    
    # YDLidar Launch
    ydlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ydlidar_dir, 'launch', 'ydlidar_launch.py')
        ),
        launch_arguments={
            'params_file': ydlidar_params_file
        }.items()
    )
    
    # SLAM Toolbox - Online Async Mapping
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # RViz2 for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_serial_port_cmd)
    
    # Add nodes and launches
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(hardware_controller_launch)
    ld.add_action(teleop_launch)
    ld.add_action(ydlidar_launch)
    ld.add_action(slam_toolbox_node)
    ld.add_action(rviz_node)
    
    return ld
