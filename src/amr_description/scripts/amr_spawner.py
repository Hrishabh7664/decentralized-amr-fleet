#!/usr/bin/env python3
"""
Reliable Gazebo AMR entity spawner for ROS 2.
Actively polls ROS 2 graph for /spawn_entity before creating client,
preventing CycloneDDS pre-service client deadlock.
"""

from __future__ import annotations
import math
import sys
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from std_msgs.msg import String


def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert Euler angles in radians to quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class AMRSpawner:
    """Spawns an AMR entity into Gazebo via /spawn_entity service."""

    def __init__(self, robot_name: str, namespace: str, x: float, y: float, z: float, yaw: float) -> None:
        self.robot_name = robot_name
        self.namespace = namespace
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.desc_xml: Optional[str] = None

    def _desc_cb(self, msg: String) -> None:
        self.desc_xml = msg.data

    def run(self) -> int:
        rclpy.init()
        node = Node(f'spawner_{self.robot_name}')
        node.get_logger().info(f"Waiting for /spawn_entity in ROS graph for {self.robot_name}...")

        start_time = time.time()
        while rclpy.ok():
            services = [name for name, _ in node.get_service_names_and_types()]
            if '/spawn_entity' in services:
                break
            time.sleep(0.5)
            if time.time() - start_time > 120.0:
                node.get_logger().error("Timeout waiting for /spawn_entity in graph!")
                node.destroy_node()
                rclpy.shutdown()
                return 1

        node.get_logger().info(f"/spawn_entity found in graph! Subscribing to XML for {self.robot_name}...")
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        topic_name = f'/{self.namespace}/robot_description' if self.namespace else '/robot_description'
        node.create_subscription(String, topic_name, self._desc_cb, qos)

        node.get_logger().info(f"Waiting for entity XML on {topic_name}...")
        while self.desc_xml is None and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            time.sleep(0.1)

        client = node.create_client(SpawnEntity, '/spawn_entity')

        req = SpawnEntity.Request()
        req.name = self.robot_name
        req.xml = self.desc_xml
        req.robot_namespace = self.namespace
        req.initial_pose = Pose()
        req.initial_pose.position.x = float(self.x)
        req.initial_pose.position.y = float(self.y)
        req.initial_pose.position.z = float(self.z)
        qx, qy, qz, qw = euler_to_quaternion(0.0, 0.0, float(self.yaw))
        req.initial_pose.orientation.x = qx
        req.initial_pose.orientation.y = qy
        req.initial_pose.orientation.z = qz
        req.initial_pose.orientation.w = qw

        node.get_logger().info(f"Calling /spawn_entity for {self.robot_name}...")
        success = False
        status_msg = "Unknown"
        for attempt in range(1, 6):
            node.get_logger().info(f"Calling /spawn_entity for {self.robot_name} (attempt {attempt}/5)...")
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=30.0)

            if future.done() and future.result():
                res = future.result()
                if res.success:
                    node.get_logger().info(f"Successfully spawned [{self.robot_name}]: {res.status_message}")
                    success = True
                    break
                else:
                    node.get_logger().warn(f"Attempt {attempt} failed: {res.status_message}")
                    status_msg = res.status_message
            else:
                node.get_logger().warn(f"Attempt {attempt} timed out after 30s. Retrying...")
            time.sleep(2.0)

        if success:
            exit_code = 0
        else:
            node.get_logger().error(f"Failed to spawn [{self.robot_name}] after multiple attempts: {status_msg}")
            exit_code = 1

        node.destroy_node()
        rclpy.shutdown()
        return exit_code


def main() -> None:
    if len(sys.argv) < 7:
        print("Usage: amr_spawner.py <robot_name> <namespace> <x> <y> <z> <yaw>")
        sys.exit(1)

    spawner = AMRSpawner(
        robot_name=sys.argv[1],
        namespace=sys.argv[2],
        x=float(sys.argv[3]),
        y=float(sys.argv[4]),
        z=float(sys.argv[5]),
        yaw=float(sys.argv[6])
    )
    sys.exit(spawner.run())


if __name__ == '__main__':
    main()
