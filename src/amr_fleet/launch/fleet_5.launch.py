"""
Launch script for 5 AMRs in Gazebo Classic warehouse world.
Enables high-density decentralized coordination, battery depletion testing, and multi-robot auctioning.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_amr_fleet = get_package_share_directory('amr_fleet')
    pkg_amr_desc = get_package_share_directory('amr_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_amr_fleet, 'worlds', 'warehouse.world')
    spawn_launch_file = os.path.join(pkg_amr_desc, 'launch', 'spawn_robot.launch.py')
    single_robot_launch_file = os.path.join(pkg_amr_fleet, 'launch', 'single_robot.launch.py')

    gui_arg = DeclareLaunchArgument('gui', default_value='true')
    gui = LaunchConfiguration('gui')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'gui': gui}.items()
    )

    robot_configs = [
        {'id': 'robot_1', 'x': '-10.0', 'y': '0.0', 'yaw': '0.0', 'bat': '100.0'},
        {'id': 'robot_2', 'x': '10.0', 'y': '0.0', 'yaw': '3.14159', 'bat': '90.0'},
        {'id': 'robot_3', 'x': '0.0', 'y': '8.0', 'yaw': '-1.57079', 'bat': '80.0'},
        {'id': 'robot_4', 'x': '0.0', 'y': '-8.0', 'yaw': '1.57079', 'bat': '45.0'},
        {'id': 'robot_5', 'x': '-5.0', 'y': '4.0', 'yaw': '-0.7853', 'bat': '18.0'},  # Below 20% test case
    ]

    launch_actions = [gui_arg, gazebo_cmd]

    for cfg in robot_configs:
        spawn_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch_file),
            launch_arguments={
                'robot_name': cfg['id'],
                'robot_namespace': cfg['id'],
                'x_pose': cfg['x'],
                'y_pose': cfg['y'],
                'z_pose': '0.05',
                'yaw': cfg['yaw']
            }.items()
        )
        stack_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(single_robot_launch_file),
            launch_arguments={'robot_id': cfg['id'], 'initial_battery': cfg['bat']}.items()
        )
        launch_actions.extend([spawn_cmd, stack_cmd])

    return LaunchDescription(launch_actions)
