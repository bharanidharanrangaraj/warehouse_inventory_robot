"""Everything in one command: Gazebo world + Nav2 + RViz + inventory nodes + a mode.

    ros2 launch warehouse_bot bringup.launch.py                       # retrieval (fetch) mode
    ros2 launch warehouse_bot bringup.launch.py mode:=audit start_mission:=true

mode:=fetch (default)  the robot brings components that users order on the web page
                       (http://localhost:<web_port>, default 8088)
mode:=audit            the inventory audit mission; it starts after mission_delay
                       seconds when start_mission:=true, or run mission.launch.py yourself
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('warehouse_bot'), 'launch')

    def include(name, condition=None, **args):
        # Scoped group: launch arguments declared inside an included file must not leak
        # into the next one. gazebo_ros' gzserver.launch.py declares params_file:='' which
        # would otherwise override navigation.launch.py's Nav2 params_file default.
        return GroupAction([IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)),
            launch_arguments={k: v for k, v in args.items()}.items())], condition=condition)

    mode = LaunchConfiguration('mode')
    is_fetch = IfCondition(PythonExpression(["'", mode, "' == 'fetch'"]))
    is_audit_now = IfCondition(PythonExpression(
        ["'", mode, "' == 'audit' and '", LaunchConfiguration('start_mission'), "' == 'true'"]))

    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='fetch', choices=['fetch', 'audit'],
                              description='fetch = deliver ordered components, audit = inventory audit'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('scan_mode', default_value='aruco'),
        DeclareLaunchArgument('web_host', default_value='0.0.0.0'),
        DeclareLaunchArgument('web_port', default_value='8088'),
        DeclareLaunchArgument('start_mission', default_value='false',
                              description='audit mode: start the audit automatically'),
        DeclareLaunchArgument('mission_delay', default_value='25.0',
                              description='audit mode: seconds to wait before starting the audit'),

        include('world.launch.py', gui=LaunchConfiguration('gui')),
        include('navigation.launch.py', rviz=LaunchConfiguration('rviz')),
        include('inventory.launch.py', scan_mode=LaunchConfiguration('scan_mode')),
        include('fetch.launch.py', condition=is_fetch,
                web_host=LaunchConfiguration('web_host'), web_port=LaunchConfiguration('web_port')),
        TimerAction(period=LaunchConfiguration('mission_delay'),
                    actions=[include('mission.launch.py')], condition=is_audit_now),
    ])
