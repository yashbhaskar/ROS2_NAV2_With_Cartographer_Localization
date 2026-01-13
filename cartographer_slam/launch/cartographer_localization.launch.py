from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import os
 
def generate_launch_description():
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    configuration_directory = LaunchConfiguration('configuration_directory')
    configuration_basename = LaunchConfiguration('configuration_basename')
    load_state_filename = LaunchConfiguration('load_state_filename')
 
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument(
            'configuration_directory',
            default_value='/home/yash/robot_ws/src/cartographer_slam/config',           # chnage path with your directory
            description='Directory that contains Cartographer config files'),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='localization.lua',
            description='Base name of the Cartographer configuration file'),
        DeclareLaunchArgument(
            'load_state_filename',
            default_value='/home/yash/robot_ws/src/robot_navigation/map/big_map.pbstream',           # chnage path with your directory
            description='Full path to the pbstream file to load'),
 
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[
                '-configuration_directory', configuration_directory,
                '-configuration_basename', configuration_basename,
                '-load_state_filename', load_state_filename,
                '-start_trajectory_with_default_topics'  # start using /scan, /odom, /imu if available
            ],
            remappings=[
                ('/scan', '/scan'),  # remap if your lidar topic differs
                ('/odom', '/odom'),
            ],
        ),
 
        # Optional occupancy grid publisher (for visualization)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
