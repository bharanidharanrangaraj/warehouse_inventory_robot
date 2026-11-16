"""Runs the autonomous inventory audit (needs world, navigation and inventory running).

    ros2 launch warehouse_bot mission.launch.py
    ros2 launch warehouse_bot mission.launch.py zone_order:=A,B,D,C set_initial_pose:=false
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('layout_file', default_value=''),
        DeclareLaunchArgument('zone_order', default_value='',
                              description='Comma-separated rack ids ("" = order in layout file)'),
        DeclareLaunchArgument('set_initial_pose', default_value='true',
                              description='Re-localise at the dock; use false if the robot is elsewhere'),

        Node(package='warehouse_bot', executable='mission_manager', name='mission_manager',
             output='screen', emulate_tty=True,
             parameters=[{
                 'use_sim_time': LaunchConfiguration('use_sim_time'),
                 'layout_file': ParameterValue(LaunchConfiguration('layout_file'), value_type=str),
                 'zone_order': ParameterValue(LaunchConfiguration('zone_order'), value_type=str),
                 'set_initial_pose': LaunchConfiguration('set_initial_pose'),
             }]),
    ])
