"""Gazebo Classic: warehouse world + the warehouse rover spawned on the charging dock.

    ros2 launch warehouse_bot world.launch.py            # with Gazebo GUI
    ros2 launch warehouse_bot world.launch.py gui:=false # headless

The rover (4-wheel skid-steer base, 4-joint arm, two-finger gripper, LiDAR, camera)
is described once in warehouse_bot/rover_description.py: its URDF goes to
robot_state_publisher, its SDF (with materials and Gazebo plugins) is spawned, and
ros2_control starts joint_state_broadcaster, arm_controller and gripper_controller.
"""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler,
                            SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from warehouse_bot import rover_description as rover
from warehouse_bot.layout import WarehouseLayout


def generate_launch_description():
    pkg = get_package_share_directory('warehouse_bot')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    # Dock pose from the layout file so the spawn point always matches
    layout = WarehouseLayout.from_file(os.path.join(pkg, 'config', 'warehouse_layout.yaml'))
    dock = layout.dock

    controllers = os.path.join(pkg, 'config', 'rover_controllers.yaml')
    sdf_path = os.path.join(tempfile.gettempdir(), 'warehouse_rover.sdf')
    with open(sdf_path, 'w') as f:
        f.write(rover.build_sdf(controllers))

    model_path = os.pathsep.join(p for p in (
        os.path.join(pkg, 'models'), os.environ.get('GAZEBO_MODEL_PATH', '')) if p)

    gui = LaunchConfiguration('gui')
    world = LaunchConfiguration('world')

    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', output='screen',
                 arguments=['-entity', 'rover', '-file', sdf_path,
                            '-x', LaunchConfiguration('x_pose'), '-y', LaunchConfiguration('y_pose'),
                            '-z', '0.01', '-Y', LaunchConfiguration('yaw')])
    controllers_up = Node(package='controller_manager', executable='spawner', output='screen',
                          arguments=['joint_state_broadcaster', 'arm_controller', 'gripper_controller',
                                     '--controller-manager', '/controller_manager',
                                     '--controller-manager-timeout', '120'])

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Start the Gazebo GUI'),
        DeclareLaunchArgument('world', default_value=os.path.join(pkg, 'worlds', 'warehouse.world')),
        DeclareLaunchArgument('x_pose', default_value=str(dock.x)),
        DeclareLaunchArgument('y_pose', default_value=str(dock.y)),
        DeclareLaunchArgument('yaw', default_value=str(dock.yaw)),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', model_path),
        # No online model database: on a firewalled PC the lookup only stalls the GUI
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
            launch_arguments={'world': world}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
            condition=IfCondition(gui)),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': rover.build_urdf(), 'use_sim_time': True}]),
        spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[controllers_up])),
    ])
