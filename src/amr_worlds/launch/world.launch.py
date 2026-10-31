"""
Launch script to start Gazebo Classic with the warehouse world.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_amr_worlds = get_package_share_directory('amr_worlds')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_path = os.path.join(pkg_amr_worlds, 'worlds', 'warehouse.world')

    gui_arg = DeclareLaunchArgument('gui', default_value='true', description='Set to false for headless execution')
    gui = LaunchConfiguration('gui')

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_path, 'gui': gui}.items()
    )

    return LaunchDescription([
        gui_arg,
        gazebo_cmd
    ])
