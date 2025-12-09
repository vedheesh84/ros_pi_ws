import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("mobile_arm_manipulator_config")
    rviz_config_file = os.path.join(package_share, "config", "moveit.rviz")
    controllers_file = os.path.join(package_share, "config", "ros2_controllers.yaml")

    default_world = os.path.join(package_share, "worlds", "empty_world.world")
    gazebo_launch_file = os.path.join(
        get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py"
    )

    moveit_config = (
        MoveItConfigsBuilder("mobile_arm_bot_2", package_name="mobile_arm_manipulator_config")
        .robot_description(file_path="config/mobile_arm_bot_2.urdf.xacro")
        .robot_description_semantic(file_path="config/mobile_arm_bot_2.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulated clock from Gazebo"
    )
    x_arg = DeclareLaunchArgument("x", default_value="0.0", description="Robot spawn position X")
    y_arg = DeclareLaunchArgument("y", default_value="0.0", description="Robot spawn position Y")
    z_arg = DeclareLaunchArgument("z", default_value="0.2", description="Robot spawn position Z")
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=default_world,
        description="Full path to the Gazebo world file",
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gazebo_launch_file),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "debug": "false",
            "gui": "true",
            "paused": "false",
            "world": LaunchConfiguration("world"),
        }.items(),
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.moveit_cpp,
            moveit_config.planning_scene_monitor,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-entity",
            "mobile_arm_bot_2",
            "-database",
            "mobile_arm_bot_2",
            "-x",
            LaunchConfiguration("x"),
            "-y",
            LaunchConfiguration("y"),
            "-z",
            LaunchConfiguration("z"),
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    mobile_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["mobile_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    moveit_parameters = moveit_config.to_dict()
    moveit_parameters["use_sim_time"] = LaunchConfiguration("use_sim_time")

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_parameters],
        arguments=["--ros-args", "--log-level", "info"],
    )

    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_mobile_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[mobile_arm_controller_spawner],
        )
    )

    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[gripper_controller_spawner],
        )
    )

    delay_rviz = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[rviz_node],
        )
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_arg)
    ld.add_action(x_arg)
    ld.add_action(y_arg)
    ld.add_action(z_arg)
    ld.add_action(gazebo)
    ld.add_action(spawn_robot)
    ld.add_action(robot_state_publisher)
    ld.add_action(move_group_node)
    ld.add_action(delay_joint_state_broadcaster)
    ld.add_action(delay_mobile_arm_controller)
    ld.add_action(delay_gripper_controller)
    ld.add_action(delay_rviz)

    return ld
