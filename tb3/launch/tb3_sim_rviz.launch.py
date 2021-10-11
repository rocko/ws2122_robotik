"""
This launch file should:
- start "turtlebot3_fake_node" node
- start "robot_state_publisher" node
- start "rviz"

# TODO Launch parameters: teleop_key for controlling


FOR SIMULATION PURPOSES ONLY!

"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo

from launch.substitutions import LaunchConfiguration


TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']  # Should be BURGER

def generate_launch_description():
    ld = LaunchDescription()
    
    rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_fake_node'),'rviz', 'model.rviz')

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_dir],
        output='screen'
    )

    param_dir = LaunchConfiguration('param_dir', default=os.path.join(get_package_share_directory('turtlebot3_fake_node'), 'param', TURTLEBOT3_MODEL + '.yaml'))

    turtlebot3_fake_node = Node(
		package="turtlebot3_fake_node",
		executable="turtlebot3_fake_node",
        parameters=[param_dir],
		output="screen",
	)

    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf]   
    )

    ld.add_action(rviz2)
    ld.add_action(turtlebot3_fake_node)
    ld.add_action(robot_state_publisher)

    return ld