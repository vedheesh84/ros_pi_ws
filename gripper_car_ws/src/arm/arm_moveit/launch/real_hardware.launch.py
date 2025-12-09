"""
Real Hardware Launch for Arm MoveIt
===================================
Combined launch file for running MoveIt with real Arduino hardware.

Launches:
1. robot_state_publisher - Publishes TF from URDF
2. servo_bridge - Arduino communication + /joint_states publisher
3. trajectory_bridge - MoveIt action servers
4. move_group - MoveIt planning
5. rviz2 - Visualization (optional)
6. static_transform_publisher - world → base_link TF

Usage:
    ros2 launch arm_moveit real_hardware.launch.py
    ros2 launch arm_moveit real_hardware.launch.py serial_port:=/dev/ttyUSB0
    ros2 launch arm_moveit real_hardware.launch.py use_rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino connection'
    )

    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz2 with MoveIt plugin'
    )

    # Get launch configurations
    serial_port = LaunchConfiguration('serial_port')
    baud_rate = LaunchConfiguration('baud_rate')
    use_rviz = LaunchConfiguration('use_rviz')

    # Build MoveIt configuration
    moveit_config = MoveItConfigsBuilder(
        "mobile_arm_bot_2", package_name="arm_moveit"
    ).to_moveit_configs()

    # Package paths
    arm_moveit_share = FindPackageShare('arm_moveit')
    arm_controller_share = FindPackageShare('arm_controller')

    # 1. Robot State Publisher (publishes TF from URDF)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[moveit_config.robot_description],
    )

    # 2. Static TF: world → base_link (virtual joint)
    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world_to_base',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_link'],
    )

    # 3. Servo Bridge (Arduino communication + joint state publisher)
    servo_bridge = Node(
        package='arm_controller',
        executable='servo_bridge.py',
        name='servo_bridge',
        output='screen',
        parameters=[{
            'serial_port': serial_port,
            'baud_rate': baud_rate,
            'publish_rate': 50.0,
            'min_command_interval': 1.0,
            'wait_for_ack': True,
            'ack_timeout': 2.0,
        }],
    )

    # 4. Trajectory Bridge (MoveIt action servers)
    trajectory_bridge = Node(
        package='arm_controller',
        executable='trajectory_bridge.py',
        name='trajectory_bridge',
        output='screen',
        parameters=[{
            'min_command_interval': 1.0,
            'skip_intermediate_points': True,
        }],
    )

    # 5. Move Group (MoveIt planning)
    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        name='move_group',
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            {'use_sim_time': False},
        ],
    )

    # 6. RViz with MoveIt plugin
    rviz_config_file = PathJoinSubstitution([
        arm_moveit_share, 'config', 'moveit.rviz'
    ])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        # Arguments
        serial_port_arg,
        baud_rate_arg,
        use_rviz_arg,
        # Nodes
        robot_state_publisher,
        static_tf_world,
        servo_bridge,
        trajectory_bridge,
        move_group,
        rviz,
    ])
