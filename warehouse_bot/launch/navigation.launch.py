"""Nav2 (map_server + AMCL + planner/controller/behaviours) and RViz.

    ros2 launch warehouse_bot navigation.launch.py
    ros2 launch warehouse_bot navigation.launch.py map:=/path/to/your_slam_map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_bot')
    nav2 = get_package_share_directory('nav2_bringup')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(pkg, 'maps', 'warehouse_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'nav2_params.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(nav2, 'launch', 'bringup_launch.py')),
            launch_arguments={'map': LaunchConfiguration('map'),
                              'use_sim_time': use_sim_time,
                              'params_file': LaunchConfiguration('params_file'),
                              'autostart': 'true'}.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg, 'rviz', 'warehouse.rviz')],
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
