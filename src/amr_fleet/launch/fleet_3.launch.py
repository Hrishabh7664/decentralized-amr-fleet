"""
Launch script for 3 AMRs in Gazebo Classic warehouse world.
Demonstrates decentralized peer-to-peer coordination, ORCA collision avoidance,
and dynamic priority negotiation on overlapping trajectories.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
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

    # Start Gazebo Server & Client
    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'gui': gui}.items()
    )

    # Robot 1: Spawn at West end of central corridor (facing East)
    spawn_r1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file),
        launch_arguments={
            'robot_name': 'robot_1',
            'robot_namespace': 'robot_1',
            'x_pose': '-10.0',
            'y_pose': '0.0',
            'z_pose': '0.05',
            'yaw': '0.0'
        }.items()
    )
    stack_r1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_robot_launch_file),
        launch_arguments={'robot_id': 'robot_1', 'initial_battery': '100.0'}.items()
    )

    # Robot 2: Spawn at East end of central corridor (facing West) - creates head-on overlap
    spawn_r2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file),
        launch_arguments={
            'robot_name': 'robot_2',
            'robot_namespace': 'robot_2',
            'x_pose': '10.0',
            'y_pose': '0.0',
            'z_pose': '0.05',
            'yaw': '3.14159'
        }.items()
    )
    stack_r2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_robot_launch_file),
        launch_arguments={'robot_id': 'robot_2', 'initial_battery': '90.0'}.items()
    )

    # Robot 3: Spawn at North aisle (facing South) - creates cross-intersection conflict
    spawn_r3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file),
        launch_arguments={
            'robot_name': 'robot_3',
            'robot_namespace': 'robot_3',
            'x_pose': '0.0',
            'y_pose': '8.0',
            'z_pose': '0.05',
            'yaw': '-1.57079'
        }.items()
    )
    stack_r3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_robot_launch_file),
        launch_arguments={'robot_id': 'robot_3', 'initial_battery': '75.0'}.items()
    )

    return LaunchDescription([
        gui_arg,
        gazebo_cmd,
        spawn_r1,
        stack_r1,
        spawn_r2,
        stack_r2,
        spawn_r3,
        stack_r3
    ])
