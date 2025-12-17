#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    
    # Package directories
    mobile_base_description_dir = get_package_share_directory('mobile_base_description')
    base_controller_dir = get_package_share_directory('base_controller')
    ydlidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # File paths
    xacro_file = os.path.join(mobile_base_description_dir, 'urdf', 'mobile_base.urdf.xacro')
    ydlidar_params_file = os.path.join(ydlidar_dir, 'params', 'X2L.yaml')
    
    # SLAM Localization params (you'll need to create this - see below)
    slam_localization_params = os.path.join(mobile_base_description_dir, 'config', 
                                           'localisation_params.yaml')
    
    # Nav2 params (you'll need to create this - see below)
    nav2_params_file = os.path.join(mobile_base_description_dir, 'config', 
                                    'nav2_params.yaml')
    
    # Map file (generated from SLAM mapping)
    default_map_file = os.path.join(mobile_base_description_dir, 'maps', 'my_map.yaml')
    
    # RViz config for navigation
    rviz_config = os.path.join(mobile_base_description_dir, 'rviz', 'navigation.rviz')
    
    # Process xacro to get robot description
    robot_description_content = xacro.process_file(xacro_file).toxml()
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM0')
    map_yaml_file = LaunchConfiguration('map', default=default_map_file)
    autostart = LaunchConfiguration('autostart', default='true')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')
    
    declare_serial_port_cmd = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for mobile base controller')
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack')
    
    # Robot State Publisher
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
    
    # Joint State Publisher
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
            'use_rviz': 'false'
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
    
    # SLAM Toolbox - Localization Mode
    slam_localization_node = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_localization_params,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Nav2 Bringup - includes all navigation nodes
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_file,
            'autostart': autostart
        }.items()
    )
    
    # RViz2 for navigation visualization
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
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_autostart_cmd)
    
    # Add nodes and launches
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(hardware_controller_launch)
    ld.add_action(teleop_launch)
    ld.add_action(ydlidar_launch)
    ld.add_action(slam_localization_node)
    ld.add_action(nav2_bringup_launch)
    ld.add_action(rviz_node)
    
    return ld
