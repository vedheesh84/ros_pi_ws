#!/usr/bin/env python3
"""
MoveIt2 Launch for Robotic Arm

Usage:
  ros2 launch arm_moveit moveit.launch.py
  ros2 launch arm_moveit moveit.launch.py rviz:=false
"""

import os
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def load_yaml(package_name, file_path):
    """Load a YAML file from a package share directory."""
    package_share = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_share, file_path)

    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except Exception as e:
        print(f"Error loading {absolute_file_path}: {e}")
        return {}


def launch_setup(context, *args, **kwargs):
    # Get package paths
    pkg_moveit = get_package_share_directory('arm_moveit')
    pkg_arm = get_package_share_directory('arm_description')

    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz = LaunchConfiguration('rviz')

    # Load URDF
    urdf_file = os.path.join(pkg_arm, 'urdf', 'arm.urdf.xacro')
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}

    # Load SRDF
    srdf_file = os.path.join(pkg_moveit, 'config', 'arm.srdf')
    with open(srdf_file, 'r') as f:
        robot_description_semantic_content = f.read()
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_content}

    # Load kinematics config
    kinematics_yaml = load_yaml('arm_moveit', 'config/kinematics.yaml')

    # Load joint limits config
    joint_limits_yaml = load_yaml('arm_moveit', 'config/joint_limits.yaml')

    # Load controllers config
    controllers_yaml = load_yaml('arm_moveit', 'config/moveit_controllers.yaml')

    # RViz config
    rviz_config = os.path.join(pkg_moveit, 'config', 'moveit.rviz')

    nodes = []

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            robot_description,
            {'use_sim_time': use_sim_time}
        ],
        output='screen'
    )
    nodes.append(robot_state_publisher)

    # Move Group Node
    move_group_params = {
        'use_sim_time': LaunchConfiguration('use_sim_time').perform(context) == 'true',
        'publish_robot_description': True,
        'publish_robot_description_semantic': True,
    }
    move_group_params.update(robot_description)
    move_group_params.update(robot_description_semantic)
    move_group_params.update(kinematics_yaml)
    move_group_params.update(joint_limits_yaml)
    move_group_params.update(controllers_yaml)

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        parameters=[move_group_params],
        output='screen'
    )
    nodes.append(move_group)

    # RViz
    rviz_params = {}
    rviz_params.update(robot_description)
    rviz_params.update(robot_description_semantic)
    rviz_params['use_sim_time'] = LaunchConfiguration('use_sim_time').perform(context) == 'true'

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[rviz_params],
        condition=IfCondition(rviz),
        output='screen'
    )
    nodes.append(rviz_node)

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false',
                             description='Use simulation time'),
        DeclareLaunchArgument('rviz', default_value='true',
                             description='Launch RViz'),
        OpaqueFunction(function=launch_setup),
    ])
