"""
This launch file should:
- start gazebo
- spawn a turtlebot in a world
- start the project node    
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

    # Declare project node
	# TBD
    #turtle_life_node = Node(
	#	package="turtle_life",
	#	executable="turtle_life_node",
	#	parameters = [config],
	#	output="screen",
	#	emulate_tty=True,
	#)

    # Declare gazebo start and model spawn
    # Example from turtlebot3_world.launch.py

	turtlesim_node = Node(
		package="turtlesim",
		executable="turtlesim_node",
		parameters=[
			{"background_b": 200},
			{"background_g": 200},
			{"background_r": 200}
		],
		output="screen",
	)

	#ld.add_action(turtle_life_node)
	ld.add_action(turtlesim_node)

	return ld