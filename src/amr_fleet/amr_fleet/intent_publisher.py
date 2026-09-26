#!/usr/bin/env python3
"""
Intent Publisher Node.
Publishes planned path, target goal, ETA, priority score, and token reservations to /fleet/{id}/intent
at 2-5 Hz using RELIABLE and TRANSIENT_LOCAL QoS for peer coordination.
"""

from __future__ import annotations
import math
import time
from typing import List

from amr_fleet.utils import Vector2D, get_intent_qos


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Path
        from geometry_msgs.msg import Point, PoseStamped
        from std_msgs.msg import String, Float32
        try:
            from amr_fleet.msg import RobotIntent
            has_custom_msg = True
        except ImportError:
            has_custom_msg = False
    except ImportError:
        print("[IntentPublisher] rclpy not found; run in ROS 2 container.")
        return

    rclpy.init(args=args)

    class IntentPublisherNode(Node):
        def __init__(self):
            super().__init__('intent_publisher')
            self.declare_parameter('robot_id', 'robot_1')
            self.declare_parameter('publish_rate_hz', 5.0)

            self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
            self.rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

            self.current_goal = Vector2D(0.0, 0.0)
            self.planned_path: List[PoseStamped] = []
            self.eta_seconds = 0.0
            self.priority_score = 0.5
            self.has_token = False
            self.reserved_corridor_id = ""
            self.task_id = "task_init"

            qos = get_intent_qos()
            if has_custom_msg:
                self.intent_pub = self.create_publisher(RobotIntent, f'/fleet/{self.robot_id}/intent', qos)
            else:
                self.intent_pub = self.create_publisher(String, f'/fleet/{self.robot_id}/intent', 10)

            self.create_subscription(Path, f'/fleet/{self.robot_id}/plan', self.plan_callback, 10)
            self.create_subscription(Point, f'/fleet/{self.robot_id}/goal', self.goal_callback, 10)
            self.create_subscription(Float32, f'/fleet/{self.robot_id}/priority', self.priority_callback, 10)
            self.create_subscription(String, f'/fleet/{self.robot_id}/token_status', self.token_callback, 10)

            self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_intent)
            self.get_logger().info(f"IntentPublisher active on /fleet/{self.robot_id}/intent at {self.rate_hz} Hz")

        def plan_callback(self, msg: Path):
            self.planned_path = msg.poses
            # Estimate ETA assuming 0.5 m/s
            total_dist = 0.0
            for i in range(1, len(self.planned_path)):
                p1 = self.planned_path[i - 1].pose.position
                p2 = self.planned_path[i].pose.position
                total_dist += math.hypot(p2.x - p1.x, p2.y - p1.y)
            self.eta_seconds = total_dist / 0.5

        def goal_callback(self, msg: Point):
            self.current_goal = Vector2D(msg.x, msg.y)

        def priority_callback(self, msg: Float32):
            self.priority_score = msg.data

        def token_callback(self, msg: String):
            # Format "corridor_id:ACQUIRED" or "corridor_id:RELEASED"
            parts = msg.data.split(':')
            if len(parts) >= 2:
                self.reserved_corridor_id = parts[0]
                self.has_token = (parts[1] == "ACQUIRED")

        def publish_intent(self):
            if has_custom_msg:
                msg = RobotIntent()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "map"
                msg.robot_id = self.robot_id
                msg.task_id = self.task_id
                msg.current_goal.x = self.current_goal.x
                msg.current_goal.y = self.current_goal.y
                msg.planned_path = self.planned_path
                msg.estimated_arrival_time = float(self.eta_seconds)
                msg.priority_score = float(self.priority_score)
                msg.has_token = self.has_token
                msg.reserved_corridor_id = self.reserved_corridor_id
                self.intent_pub.publish(msg)
            else:
                import json
                payload = {
                    "robot_id": self.robot_id,
                    "goal": {"x": self.current_goal.x, "y": self.current_goal.y},
                    "path_len": len(self.planned_path),
                    "eta": round(self.eta_seconds, 1),
                    "priority": round(self.priority_score, 3),
                    "has_token": self.has_token,
                    "corridor": self.reserved_corridor_id,
                    "time": time.time()
                }
                msg = String()
                msg.data = json.dumps(payload)
                self.intent_pub.publish(msg)

    node = IntentPublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
