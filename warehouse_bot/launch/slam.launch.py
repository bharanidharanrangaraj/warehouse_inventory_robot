"""Optional: build your own map with slam_toolbox (for the report's map-design section).

    ros2 launch warehouse_bot world.launch.py
    ros2 launch warehouse_bot slam.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard  # drive around every aisle
    ros2 run nav2_map_server map_saver_cli -f ~/warehouse_slam_map
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_bot')
    slam = get_package_share_directory('slam_toolbox')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(slam, 'launch', 'online_async_launch.py')),
            launch_arguments={'use_sim_time': 'true'}.items()),
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(pkg, 'rviz', 'warehouse.rviz')],
             parameters=[{'use_sim_time': True}]),
    ])
