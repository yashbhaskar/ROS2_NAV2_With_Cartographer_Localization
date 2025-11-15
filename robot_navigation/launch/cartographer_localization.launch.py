import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
  nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
  carto_launch_dir = os.path.join(get_package_share_directory('cartographer_slam'), 'launch')
  prefix_address = get_package_share_directory('robot_navigation') 
  params_file= os.path.join(prefix_address, 'config', 'nav2_params.yaml')

  use_sim_time = LaunchConfiguration('use_sim_time', default='true')
  map_dir = LaunchConfiguration(
      'map',
      default=os.path.join(
          get_package_share_directory('robot_navigation'),
          'map',
          'yash_map.yaml'))
  rviz_config_path = os.path.join(
    get_package_share_directory('robot_navigation'),
    'rviz',
    'yash.rviz'
    )

  param_file_name = 'nav2_params.yaml'
  param_dir = LaunchConfiguration(
      params_file,
      default=os.path.join(
          get_package_share_directory('robot_navigation'),
          'config',
          param_file_name))
  
  navigation_launch_cmd=IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_dir, '/navigation_launch.py']),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': param_dir}.items(),
        )
  
  carto_localization = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([carto_launch_dir, '/cartographer_localization.launch.py']),
        )

  rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    name='Rviz',
    arguments=['-d', rviz_config_path],
    output='screen',
    parameters=[{'use_sim_time': True}]
    )
  
  return LaunchDescription([
    navigation_launch_cmd,
    rviz_node,
    carto_localization
  ]
)
