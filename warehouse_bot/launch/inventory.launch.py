"""Inventory nodes: zone_detector, marker_scanner and inventory_manager.

    ros2 launch warehouse_bot inventory.launch.py
    ros2 launch warehouse_bot inventory.launch.py scan_mode:=rfid_sim
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    layout = ParameterValue(LaunchConfiguration('layout_file'), value_type=str)
    common = {'use_sim_time': use_sim_time, 'layout_file': layout}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('layout_file', default_value='',
                              description='Layout YAML ("" = installed default)'),
        DeclareLaunchArgument('scan_mode', default_value='aruco',
                              description='aruco (camera) or rfid_sim'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),

        Node(package='warehouse_bot', executable='zone_detector', name='zone_detector',
             output='screen', parameters=[common]),
        Node(package='warehouse_bot', executable='marker_scanner', name='marker_scanner',
             output='screen',
             parameters=[common, {
                 'scan_mode': ParameterValue(LaunchConfiguration('scan_mode'), value_type=str),
                 'image_topic': ParameterValue(LaunchConfiguration('image_topic'), value_type=str)}]),
        Node(package='warehouse_bot', executable='inventory_manager', name='inventory_manager',
             output='screen', parameters=[common]),
    ])
