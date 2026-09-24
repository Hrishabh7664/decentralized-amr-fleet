"""
Launch script for a single AMR's onboard autonomous decision stack.
Runs fully decentralized without any central server dependencies.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_id_arg = DeclareLaunchArgument('robot_id', default_value='robot_1')
    initial_battery_arg = DeclareLaunchArgument('initial_battery', default_value='100.0')

    robot_id = LaunchConfiguration('robot_id')
    initial_battery = LaunchConfiguration('initial_battery')

    state_pub_node = Node(
        package='amr_fleet',
        executable='state_publisher.py',
        name='state_publisher',
        namespace=robot_id,
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    intent_pub_node = Node(
        package='amr_fleet',
        executable='intent_publisher.py',
        name='intent_publisher',
        namespace=robot_id,
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    global_planner_node = Node(
        package='amr_fleet',
        executable='global_planner.py',
        name='global_planner',
        namespace=robot_id,
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    local_planner_orca_node = Node(
        package='amr_fleet',
        executable='local_planner_orca.py',
        name='local_planner_orca',
        namespace=robot_id,
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    conflict_resolver_node = Node(
        package='amr_fleet',
        executable='conflict_resolver.py',
        name='conflict_resolver',
        namespace=robot_id,
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    task_allocator_node = Node(
        package='amr_fleet',
        executable='task_allocator.py',
        name='task_allocator',
        namespace=robot_id,
        output='screen',
        parameters=[{'robot_id': robot_id}]
    )

    battery_monitor_node = Node(
        package='amr_fleet',
        executable='battery_monitor.py',
        name='battery_monitor',
        namespace=robot_id,
        output='screen',
        parameters=[{
            'robot_id': robot_id,
            'initial_battery': initial_battery
        }]
    )

    return LaunchDescription([
        robot_id_arg,
        initial_battery_arg,
        state_pub_node,
        intent_pub_node,
        global_planner_node,
        local_planner_orca_node,
        conflict_resolver_node,
        task_allocator_node,
        battery_monitor_node
    ])
