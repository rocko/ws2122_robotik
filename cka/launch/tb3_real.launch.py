"""
This launch file should:
- run a zigzag move pattern on the real turtlebot   
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

def generate_launch_description():
	ld = LaunchDescription()
	
	config = os.path.join(get_package_share_directory("turtle_life"),"config","config.yaml")

    # Declare gazebo start and model spawn
    # Example from turtlebot3_world.launch.py

	cka_tb3_zigzag = Node(
		package="cka_tb3_zigzag",
		executable="cka_tb3_zigzag",
		output="screen",
	)

	ld.add_action(cka_tb3_zigzag)

	return ld