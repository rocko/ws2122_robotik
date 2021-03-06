"""
This launch file should:
- launch gazebo and rviz and start turtlebot3 in the given world


FOR SIMULATION PURPOSES ONLY!

"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']  # Should be BURGER

def generate_launch_description():

    # Gazebo Params
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    #world_file_name = 'turtlebot3_houses/' + TURTLEBOT3_MODEL + '.model'
    world_file_name = 'simple_room_2/' + 'simple_room_2' + '.model'
    world = os.path.join(get_package_share_directory('cka_gazebo'),'worlds', world_file_name)
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # Turtlebot3 Fake Node Params
    param_dir = LaunchConfiguration('param_dir', default=os.path.join(get_package_share_directory('turtlebot3_fake_node'), 'param', TURTLEBOT3_MODEL + '.yaml'))
    # RVIZ Params
    urdf_file_name = 'turtlebot3_' + TURTLEBOT3_MODEL + '.urdf'
    urdf = os.path.join(get_package_share_directory('turtlebot3_description'), 'urdf', urdf_file_name)
    rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_fake_node'),'rviz', 'model.rviz')


    return LaunchDescription([
		# Gazebo Server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world}.items(),
        ),

		# Gazebo Client
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
        ),

		# Robot State Publisher Node
        #IncludeLaunchDescription(
        #    PythonLaunchDescriptionSource([launch_file_dir, '/robot_state_publisher.launch.py']),
        #    launch_arguments={'use_sim_time': use_sim_time}.items(),
        #),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]),        
        # RVIZ Node
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            output='screen',
        ),
        # Turtlebot3 Fake Node
        Node(
            package="turtlebot3_fake_node",
            executable="turtlebot3_fake_node",
            parameters=[param_dir],
            output="screen",
        ),

        # ZigZagNode
        Node(
            package='cka',
            executable='zigzag',
            output='screen',
        )
    ])
