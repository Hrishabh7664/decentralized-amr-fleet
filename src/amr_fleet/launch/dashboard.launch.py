"""
Launch script for the Fleet Monitoring Web Dashboard.
Starts rosbridge_server (WebSocket on port 9090) and static web dashboard on port 3000.
Dashboard is STRICTLY READ-ONLY for monitoring; does not issue control commands.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    rosbridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': 9090,
            'address': '0.0.0.0',
            'retry_startup_delay': 2.0
        }]
    )

    # Launch lightweight HTTP server for the built React web dashboard
    web_server_cmd = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '3000', '--directory', 'src/amr_dashboard/public'],
        output='screen'
    )

    return LaunchDescription([
        rosbridge_node,
        web_server_cmd
    ])
