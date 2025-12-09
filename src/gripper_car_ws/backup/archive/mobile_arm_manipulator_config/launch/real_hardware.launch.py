import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_arm_manipulator_config')

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name='xacro')]),
        ' ',
        PathJoinSubstitution([pkg_share, 'urdf', 'mobile_arm_bot.urdf.xacro']),
        ' use_sim:=false',
    ])
    robot_description = {'robot_description': robot_description_content}

    real_controllers_file = os.path.join(pkg_share, 'config', 'real_controllers.yaml')

    microros_agent = ExecuteProcess(
        cmd=['micro-ros-agent', 'serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        output='screen',
    )

    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, real_controllers_file],
        output='screen',
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['mobile_arm_controller', '-c', '/controller_manager'],
        output='screen',
    )

    return LaunchDescription([
        microros_agent,
        ros2_control_node,
        controller_spawner,
        # ... add MoveIt/RViz nodes as needed ...
    ])
