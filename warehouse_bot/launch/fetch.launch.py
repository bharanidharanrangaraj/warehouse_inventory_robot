"""Retrieval mode: fetch_manager + item_handler + the order web page.

Needs world, navigation and inventory running (bringup.launch.py does all of it):

    ros2 launch warehouse_bot fetch.launch.py
    ros2 launch warehouse_bot fetch.launch.py web_port:=9000 confirm_with_camera:=false

Then open http://localhost:8088 and order a component, or:

    ros2 service call /warehouse/request_item warehouse_interfaces/srv/RequestItem "{product: diode}"
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
        DeclareLaunchArgument('web_host', default_value='127.0.0.1',
                              description='Address the order page listens on '
                                          '(0.0.0.0 = whole network; the page has no login)'),
        DeclareLaunchArgument('web_port', default_value='8088',
                              description='Port of the order page (http://localhost:<port>)'),
        DeclareLaunchArgument('confirm_with_camera', default_value='true',
                              description='Read the rack labels with the camera before picking'),
        DeclareLaunchArgument('set_initial_pose', default_value='true',
                              description='Localise at the dock; false if the robot is elsewhere'),

        Node(package='warehouse_bot', executable='item_handler', name='item_handler',
             output='screen', parameters=[common]),
        # No name= here: the node runs a BasicNavigator in the same process, and a
        # launch name remap would rename both nodes.
        Node(package='warehouse_bot', executable='fetch_manager', output='screen',
             emulate_tty=True,
             parameters=[common, {
                 'confirm_with_camera': ParameterValue(LaunchConfiguration('confirm_with_camera'),
                                                       value_type=bool),
                 'set_initial_pose': ParameterValue(LaunchConfiguration('set_initial_pose'),
                                                    value_type=bool)}]),
        Node(package='warehouse_bot', executable='web_server', name='web_server',
             output='screen',
             parameters=[common, {
                 'host': ParameterValue(LaunchConfiguration('web_host'), value_type=str),
                 'port': ParameterValue(LaunchConfiguration('web_port'), value_type=int)}]),
    ])
