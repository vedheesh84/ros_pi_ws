#!/usr/bin/env python3
import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
def generate_launch_description():

	use_sim_time = LaunchConfiguration('use_sim_time', default='false')

	 # Paths
	package_name = 'components'
	rviz_pkg_share_dir = get_package_share_directory('rviz_tutorial')
	pkg_share_dir = get_package_share_directory(package_name)



	urdf_file_name = 'robot.urdf.xacro'
	urdf_path = os.path.join(pkg_share_dir, 'urdf', urdf_file_name)
    
	robot_description_content = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)
	robot_description_param = {'robot_description': robot_description_content}


	rviz_config_file = os.path.join(rviz_pkg_share_dir, 'config', 'default.rviz')

	

	# Robot State Publisher
	robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}]
    )


	rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
	
	
	ld = LaunchDescription()

    
	ld.add_action(robot_state_publisher_node)
	ld.add_action(rviz_node)

	return ld


    #rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'),
     #                              'rviz', 'tb3_cartographer.rviz')