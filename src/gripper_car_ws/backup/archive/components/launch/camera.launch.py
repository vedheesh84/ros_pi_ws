import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.conditions import IfCondition  # Add this line
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the URDF file
    pkg_path = get_package_share_directory('components')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    
    # Declare use_sim_time argument
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    # Process the URDF with xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher Node (for fixed joints)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # V4L2 Camera Node
    v4l2_camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        parameters=[{
            'video_device': '/dev/video0',
            'image_size': [1280, 720],
            'camera_frame_id': 'camera_link_optical',
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
            'framerate': 30.0,
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
        ]
    )
    
    # RViz Node
    rviz_config = os.path.join(pkg_path, 'rviz', 'robot_camera.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(LaunchConfiguration('rviz', default='true'))  # Fixed line
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        robot_state_publisher_node,
        joint_state_publisher_node,
        v4l2_camera_node,
        rviz_node,
    ])