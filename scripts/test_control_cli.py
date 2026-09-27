#!/usr/bin/env python3
"""
Test CLI tool to trigger control topics and services from inside or outside the container.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import Empty


def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_control_cli.py [speed <float> | add <name> | remove <name> | pause | unpause]")
        return

    cmd = sys.argv[1].lower()
    rclpy.init()
    node = Node('test_control_cli')

    if cmd == 'speed':
        val = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
        pub = node.create_publisher(Float32, '/fleet/speed_scale', 10)
        time.sleep(0.5)
        msg = Float32()
        msg.data = val
        pub.publish(msg)
        print(f"Published /fleet/speed_scale: {val}")
        time.sleep(0.5)

    elif cmd == 'add':
        name = sys.argv[2] if len(sys.argv) > 2 else 'robot_4'
        pub = node.create_publisher(String, '/fleet/add_robot', 10)
        time.sleep(0.5)
        msg = String()
        msg.data = name
        pub.publish(msg)
        print(f"Published /fleet/add_robot: {name}")
        time.sleep(1.0)

    elif cmd == 'remove':
        name = sys.argv[2] if len(sys.argv) > 2 else 'robot_3'
        pub = node.create_publisher(String, '/fleet/remove_robot', 10)
        time.sleep(0.5)
        msg = String()
        msg.data = name
        pub.publish(msg)
        print(f"Published /fleet/remove_robot: {name}")
        time.sleep(1.0)

    elif cmd == 'pause':
        cli = node.create_client(Empty, '/gazebo/pause_physics')
        if not cli.wait_for_service(timeout_sec=2.0):
            cli = node.create_client(Empty, '/pause_physics')
            cli.wait_for_service(timeout_sec=2.0)
        future = cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        print("Called pause_physics service")

    elif cmd == 'unpause':
        cli = node.create_client(Empty, '/gazebo/unpause_physics')
        if not cli.wait_for_service(timeout_sec=2.0):
            cli = node.create_client(Empty, '/unpause_physics')
            cli.wait_for_service(timeout_sec=2.0)
        future = cli.call_async(Empty.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        print("Called unpause_physics service")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
