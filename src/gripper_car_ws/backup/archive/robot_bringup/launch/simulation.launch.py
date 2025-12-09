#!/usr/bin/env python3
"""
Gazebo Simulation Launch - Complete robot in simulation

Launches:
- Gazebo with world
- Robot spawn
- Robot State Publisher
- Controllers
- Optionally: MoveIt, Nav2, RViz

Usage:
  ros2 launch robot_bringup simulation.launch.py
  ros2 launch robot_bringup simulation.launch.py world:=empty_world.world
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    Command,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package paths
    pkg_robot_bringup = FindPackageShare('robot_bringup')
    pkg_arm_moveit = FindPackageShare('arm_moveit')
    pkg_mobile_base_nav2 = FindPackageShare('mobile_base_nav2')
    pkg_gazebo_ros = FindPackageShare('gazebo_ros')

    # Launch configurations
    world = LaunchConfiguration('world')
    enable_moveit = LaunchConfiguration('enable_moveit')
    enable_nav2 = LaunchConfiguration('enable_nav2')
    enable_rviz = LaunchConfiguration('enable_rviz')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')

    # Robot description
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([pkg_robot_bringup, 'config', 'robot.urdf.xacro']),
        ' use_sim:=true'
    ])

    robot_description = {'robot_description': robot_description_content}

    # Declare arguments
    declare_world = DeclareLaunchArgument(
        'world',
        default_value='mobile_manipulator_world.world',
        description='World file name'
    )

    declare_enable_moveit = DeclareLaunchArgument(
        'enable_moveit',
        default_value='true',
        description='Enable MoveIt'
    )

    declare_enable_nav2 = DeclareLaunchArgument(
        'enable_nav2',
        default_value='false',
        description='Enable Nav2'
    )

    declare_enable_rviz = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable RViz'
    )

    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='X position of robot'
    )

    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Y position of robot'
    )

    declare_z_pose = DeclareLaunchArgument(
        'z_pose', default_value='0.1',
        description='Z position of robot'
    )

    # Gazebo
    world_path = PathJoinSubstitution([pkg_robot_bringup, 'worlds', world])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
        ]),
        launch_arguments={
            'world': world_path,
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Spawn robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'mobile_manipulator',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen'
    )

    # Joint State Broadcaster
    joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    # Arm Controller
    arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'arm_controller'],
        output='screen'
    )

    # Gripper Controller
    gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'gripper_controller'],
        output='screen'
    )

    # Register event handlers to load controllers after spawn
    load_jsb_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster],
        )
    )

    load_arm_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller],
        )
    )

    load_gripper_after_arm = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=arm_controller,
            on_exit=[gripper_controller],
        )
    )

    # RViz
    rviz_config = PathJoinSubstitution([
        pkg_robot_bringup, 'config', 'robot.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(enable_rviz),
        output='screen'
    )

    # MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_arm_moveit, 'launch', 'moveit.launch.py'])
        ]),
        condition=IfCondition(enable_moveit),
        launch_arguments={
            'use_sim_time': 'true',
            'use_robot_bringup': 'true',
            'rviz': 'false',
        }.items()
    )

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_mobile_base_nav2, 'launch', 'navigation.launch.py'])
        ]),
        condition=IfCondition(enable_nav2),
        launch_arguments={
            'use_sim': 'true',
        }.items()
    )

    return LaunchDescription([
        # Arguments
        declare_world,
        declare_enable_moveit,
        declare_enable_nav2,
        declare_enable_rviz,
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,

        # Gazebo
        gazebo,
        robot_state_publisher,
        spawn_robot,

        # Controller loading
        load_jsb_after_spawn,
        load_arm_after_jsb,
        load_gripper_after_arm,

        # Visualization
        rviz_node,

        # Subsystems
        moveit_launch,
        nav2_launch,
    ])
