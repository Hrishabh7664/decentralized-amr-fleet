"""
Launch script to spawn a single AMR model in Gazebo with namespaced robot_state_publisher.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def spawn_robot_eval(context, *args, **kwargs):
    robot_name = context.perform_substitution(LaunchConfiguration('robot_name'))
    robot_namespace = context.perform_substitution(LaunchConfiguration('robot_namespace'))
    x_pose = context.perform_substitution(LaunchConfiguration('x_pose'))
    y_pose = context.perform_substitution(LaunchConfiguration('y_pose'))
    z_pose = context.perform_substitution(LaunchConfiguration('z_pose'))
    yaw = context.perform_substitution(LaunchConfiguration('yaw'))

    spawn_entity_node = Node(
        package='amr_description',
        executable='amr_spawner.py',
        name=f'spawner_{robot_name}',
        arguments=[
            robot_name,
            robot_namespace,
            x_pose,
            y_pose,
            z_pose,
            yaw
        ],
        output='screen'
    )

    return [spawn_entity_node]


def generate_launch_description():
    pkg_share = get_package_share_directory('amr_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'amr.urdf.xacro')

    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='robot_1')
    robot_ns_arg = DeclareLaunchArgument('robot_namespace', default_value='robot_1')
    x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0')
    y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0')
    z_pose_arg = DeclareLaunchArgument('z_pose', default_value='0.05')
    yaw_arg = DeclareLaunchArgument('yaw', default_value='0.0')

    robot_namespace = LaunchConfiguration('robot_namespace')

    robot_description_content = Command([
        'xacro ', xacro_file,
        ' robot_namespace:=', robot_namespace
    ])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_namespace,
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    return LaunchDescription([
        robot_name_arg,
        robot_ns_arg,
        x_pose_arg,
        y_pose_arg,
        z_pose_arg,
        yaw_arg,
        robot_state_publisher_node,
        OpaqueFunction(function=spawn_robot_eval)
    ])
