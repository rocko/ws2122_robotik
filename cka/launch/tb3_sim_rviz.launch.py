"""
This launch file should:
- launch gazebo and rviz and start turtlebot3 in the given world
- launch cartographer

FOR SIMULATION PURPOSES ONLY!


LAUNCH ORDER

- gazebo 
- robot_state_publisher 
- cartographer
- rviz
- manipulation


"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import TimerAction
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
    #rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_fake_node'),'rviz', 'model.rviz')
    rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'),'rviz', 'tb3_cartographer.rviz')

    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')
    resolution = LaunchConfiguration('resolution', default='0.05')
    configuration_basename = LaunchConfiguration('configuration_basename', default='turtlebot3_lds_2d.lua')
    turtlebot3_cartographer_prefix = get_package_share_directory('turtlebot3_cartographer')
    cartographer_config_dir = LaunchConfiguration('cartographer_config_dir', default=os.path.join(turtlebot3_cartographer_prefix, 'config'))
    #cartographer_launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'), 'launch')

    return LaunchDescription([
		# Gazebo Server
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')),launch_arguments={'world': world}.items(),),
		# Gazebo Client
        IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')),),

        # LAUNCH ARGS
        # Robot State Publisher
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        # Cartographer
        DeclareLaunchArgument('cartographer_config_dir', default_value=cartographer_config_dir, description='Full path to config file to load'),
        DeclareLaunchArgument('configuration_basename', default_value=configuration_basename, description='Name of lua file for cartographer'),
        #DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        # RVIZ
        DeclareLaunchArgument('resolution', default_value=resolution, description='Resolution of a grid cell in the published occupancy grid'),
        DeclareLaunchArgument('publish_period_sec', default_value=publish_period_sec, description='OccupancyGrid publishing period'),

        # Timed Nodes
        TimerAction(
            period=10.0,
            actions=[
                # Robot State Publisher Node
                Node(
                    package='robot_state_publisher',
                    executable='robot_state_publisher',
                    name='robot_state_publisher',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=[urdf]
                ),
                # Turtlebot3 Fake Node
                Node(
                    package="turtlebot3_fake_node",
                    executable="turtlebot3_fake_node",
                    parameters=[param_dir],
                    output="screen",
                ),
                # Cartographer
                Node(
                    package='cartographer_ros',
                    executable='cartographer_node',
                    name='cartographer_node',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=['-configuration_directory', cartographer_config_dir,'-configuration_basename', configuration_basename]
                ),
                # Cartographer Occupancy
                Node(
                    package='cartographer_ros',
                    executable='occupancy_grid_node',
                    name='occupancy_grid_node',
                    output='screen',
                    parameters=[{'use_sim_time': use_sim_time}],
                    arguments=['-resolution', resolution, '-publish_period_sec', publish_period_sec]
                ),
                # RVIZ
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    arguments=['-d', rviz_config_dir],
                    parameters=[{'use_sim_time': use_sim_time}],
                    output='screen',
                ),
                # ZigZagNode
                Node(
                    package='cka',
                    executable='zigzag',
                    output='screen',
                )
            ]
        ),
    ])
