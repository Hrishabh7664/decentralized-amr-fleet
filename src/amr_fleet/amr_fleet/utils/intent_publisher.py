#!/usr/bin/env python3
"""
Intent Publisher Node.
Publishes planned trajectories and priority metadata to /fleet/{id}/intent
using RELIABLE + TRANSIENT_LOCAL QoS for peer spatiotemporal coordination.
"""

from __future__ import annotations
import json
import time
from typing import List, Optional, Any

from amr_fleet.utils import Vector2D, get_intent_qos


class IntentPublisherNode:
    """ROS 2 Node broadcasting AMR route intentions and corridor tokens."""
    def __init__(self, node: Any, has_custom_msg: bool = False, intent_msg_cls: Any = None) -> None:
        """Initialize intent publisher subscribers, publishers, and parameters."""
        self.node = node
        self.has_custom_msg = has_custom_msg
        self.RobotIntent = intent_msg_cls

        self.node.declare_parameter('robot_id', 'robot_1')
        self.node.declare_parameter('publish_rate_hz', 2.0)

        self.robot_id = self.node.get_parameter('robot_id').get_parameter_value().string_value
        self.rate_hz = self.node.get_parameter('publish_rate_hz').get_parameter_value().double_value

        self.current_goal = Vector2D(0.0, 0.0)
        self.planned_path: List[Vector2D] = []
        self.task_id: str = "task_0"
        self.priority_score: float = 0.5
        self.has_token: bool = False
        self.reserved_corridor_id: str = ""

        from nav_msgs.msg import Path
        from geometry_msgs.msg import Point
        from std_msgs.msg import String, Float32

        qos = get_intent_qos()
        if self.has_custom_msg and self.RobotIntent is not None:
            self.intent_pub = self.node.create_publisher(self.RobotIntent, f'/fleet/{self.robot_id}/intent', qos)
        else:
            self.intent_pub = self.node.create_publisher(String, f'/fleet/{self.robot_id}/intent', 10)

        self.node.create_subscription(Path, f'/fleet/{self.robot_id}/plan', self.plan_callback, 10)
        self.node.create_subscription(Path, f'/{self.robot_id}/plan', self.plan_callback, 10)
        self.node.create_subscription(Point, f'/fleet/{self.robot_id}/goal', self.goal_callback, 10)
        self.node.create_subscription(Point, f'/{self.robot_id}/goal', self.goal_callback, 10)
        self.node.create_subscription(Float32, f'/fleet/{self.robot_id}/priority', self.priority_callback, 10)
        self.node.create_subscription(String, f'/fleet/{self.robot_id}/token_status', self.token_callback, 10)

        self.timer = self.node.create_timer(1.0 / self.rate_hz, self.publish_intent)
        self.node.get_logger().info(f"IntentPublisher active on /fleet/{self.robot_id}/intent at {self.rate_hz} Hz")

    def plan_callback(self, msg: Any) -> None:
        """Update cached planned path waypoints."""
        path = []
        for ps in getattr(msg, 'poses', []):
            if hasattr(ps, 'pose'):
                path.append(Vector2D(ps.pose.position.x, ps.pose.position.y))
            elif hasattr(ps, 'position'):
                path.append(Vector2D(ps.position.x, ps.position.y))
            elif hasattr(ps, 'x') and hasattr(ps, 'y'):
                path.append(Vector2D(ps.x, ps.y))
        self.planned_path = path

    def goal_callback(self, msg: Any) -> None:
        """Update active navigation destination."""
        self.current_goal = Vector2D(msg.x, msg.y)

    def priority_callback(self, msg: Any) -> None:
        """Update assigned dynamic priority score."""
        self.priority_score = float(msg.data)

    def token_callback(self, msg: Any) -> None:
        """Update corridor reservation status."""
        try:
            parts = msg.data.split(':')
            if len(parts) >= 2:
                self.reserved_corridor_id = parts[0]
                self.has_token = (parts[1].upper() == "HELD")
        except Exception as e:
            self.node.get_logger().debug(f"Token callback parse: {e}")

    def publish_intent(self) -> None:
        """Broadcast serialized intent payload to peer AMRs."""
        if self.has_custom_msg and self.RobotIntent is not None:
            from geometry_msgs.msg import Point, PoseStamped
            from std_msgs.msg import Header
            msg = self.RobotIntent()
            msg.header = Header()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.robot_id = str(self.robot_id)
            msg.task_id = str(self.task_id)
            msg.current_goal = Point(x=float(self.current_goal.x), y=float(self.current_goal.y), z=0.0)
            poses = []
            for p in self.planned_path:
                ps = PoseStamped()
                ps.header.frame_id = "map"
                ps.pose.position.x = float(p.x)
                ps.pose.position.y = float(p.y)
                ps.pose.orientation.w = 1.0
                poses.append(ps)
            msg.planned_path = poses
            msg.estimated_arrival_time = 0.0
            msg.priority_score = float(self.priority_score)
            msg.has_token = bool(self.has_token)
            msg.reserved_corridor_id = str(self.reserved_corridor_id)
            self.intent_pub.publish(msg)
        else:
            from std_msgs.msg import String
            payload = {
                "robot_id": self.robot_id,
                "task_id": self.task_id,
                "goal": {"x": round(self.current_goal.x, 3), "y": round(self.current_goal.y, 3)},
                "path": [{"x": round(p.x, 3), "y": round(p.y, 3)} for p in self.planned_path],
                "priority": round(self.priority_score, 3),
                "has_token": self.has_token,
                "corridor": self.reserved_corridor_id,
                "time": time.time()
            }
            msg = String()
            msg.data = json.dumps(payload)
            self.intent_pub.publish(msg)


def main(args=None) -> None:
    """Entrypoint for intent publisher node."""
    try:
        import rclpy
        from rclpy.node import Node
        try:
            from amr_fleet.msg import RobotIntent
            has_custom_msg = True
            intent_msg_cls = RobotIntent
        except ImportError:
            has_custom_msg = False
            intent_msg_cls = None
    except ImportError:
        print("[IntentPublisher] rclpy not detected; run in ROS 2 container.")
        return

    rclpy.init(args=args)
    node = Node('intent_publisher')
    _ = IntentPublisherNode(node, has_custom_msg, intent_msg_cls)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
