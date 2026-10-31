#!/usr/bin/env python3
"""
State Publisher Node.
Publishes telemetry (pose, twist, battery, operating state) to /fleet/{id}/state
at 10-20 Hz using BEST_EFFORT QoS with a 100ms deadline.
Also publishes PoseStamped to /{id}/pose for internal node consumption.
"""

from __future__ import annotations
import json
import time
from typing import Any

from amr_fleet.utils import Pose2D, Twist2D, get_state_qos, quaternion_to_yaw, MessageRelayCache


class StatePublisherNode:
    """ROS 2 Node wrapper for broadcasting AMR odometry, twist, and battery telemetry."""
    def __init__(self, node: Any, has_custom_msg: bool = False, robot_state_cls: Any = None) -> None:
        """Initialize the state publisher node."""
        self.node = node
        self.has_custom_msg = has_custom_msg
        self.RobotState = robot_state_cls

        self.node.declare_parameter('robot_id', 'robot_1')
        self.node.declare_parameter('publish_rate_hz', 20.0)
        self.node.declare_parameter('relay_in_topic', '/fleet/relay/in')
        self.node.declare_parameter('relay_out_topic', '/fleet/relay/out')
        self.node.declare_parameter('enable_relay', True)

        self.robot_id = self.node.get_parameter('robot_id').get_parameter_value().string_value
        self.rate_hz = self.node.get_parameter('publish_rate_hz').get_parameter_value().double_value
        relay_in = self.node.get_parameter('relay_in_topic').get_parameter_value().string_value
        relay_out = self.node.get_parameter('relay_out_topic').get_parameter_value().string_value
        enable_relay = self.node.get_parameter('enable_relay').get_parameter_value().bool_value

        self.current_pose = Pose2D()
        self.current_twist = Twist2D()
        self.battery_pct = 100.0
        self.is_charging = False
        self.current_state = "IDLE"

        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float32, String
        from geometry_msgs.msg import PoseStamped

        qos = get_state_qos()
        qos_state = qos if qos is not None else 10
        if self.has_custom_msg and self.RobotState is not None:
            self.state_pub = self.node.create_publisher(self.RobotState, f'/fleet/{self.robot_id}/state', qos_state)
        else:
            self.state_pub = self.node.create_publisher(String, f'/fleet/{self.robot_id}/state', qos_state)

        self.pose_pub = self.node.create_publisher(PoseStamped, f'/{self.robot_id}/pose', 10)

        # Multi-Hop Mesh Relay Cache
        self.relay_cache = MessageRelayCache(ttl_seconds=10.0, max_cache_size=1000)
        self.relay_pub = self.node.create_publisher(String, relay_out, 10)
        if enable_relay:
            self.node.create_subscription(String, relay_in, self.relay_callback, 10)

        self.node.create_subscription(Odometry, f'/{self.robot_id}/odom', self.odom_callback, 10)
        self.node.create_subscription(Float32, f'/fleet/{self.robot_id}/battery', self.battery_callback, 10)
        self.node.create_subscription(Float32, f'/{self.robot_id}/battery', self.battery_callback, 10)
        self.node.create_subscription(String, f'/fleet/{self.robot_id}/status', self.status_callback, 10)
        self.node.create_subscription(String, f'/fleet/{self.robot_id}/charging_intent', self.charging_intent_callback, 10)
        self.node.create_subscription(String, f'/{self.robot_id}/charging_intent', self.charging_intent_callback, 10)

        self.timer = self.node.create_timer(1.0 / self.rate_hz, self.publish_state)
        self.node.get_logger().info(f"StatePublisher active on /fleet/{self.robot_id}/state at {self.rate_hz} Hz")

    def relay_callback(self, msg: Any) -> None:
        """Relay incoming mesh broadcast packets across network dead zones using deduplication cache."""
        from std_msgs.msg import String
        try:
            raw_data = msg.data if hasattr(msg, 'data') else str(msg)
            try:
                payload = json.loads(raw_data)
                if isinstance(payload, dict):
                    msg_id = payload.get("message_id") or payload.get("id") or str(raw_data)
                    ttl = int(payload.get("ttl", 3))
                    if ttl <= 0:
                        return
                    if not self.relay_cache.should_forward(str(msg_id)):
                        return
                    payload["ttl"] = ttl - 1
                    payload["relayed_by"] = self.robot_id
                    out_msg = String()
                    out_msg.data = json.dumps(payload)
                    self.relay_pub.publish(out_msg)
                    return
            except (json.JSONDecodeError, TypeError):
                pass

            msg_id = str(raw_data)
            if self.relay_cache.should_forward(msg_id):
                out_msg = String()
                out_msg.data = str(raw_data)
                self.relay_pub.publish(out_msg)
        except Exception as e:
            self.node.get_logger().debug(f"Relay forwarding exception: {e}")

    def odom_callback(self, msg: Any) -> None:
        """Process incoming wheel odometry messages."""
        from geometry_msgs.msg import PoseStamped
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.current_pose.theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        self.current_twist.linear.x = msg.twist.twist.linear.x
        self.current_twist.linear.y = msg.twist.twist.linear.y
        self.current_twist.angular = msg.twist.twist.angular.z

        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        self.pose_pub.publish(ps)

        if abs(self.current_twist.linear.x) > 0.05 or abs(self.current_twist.angular) > 0.05:
            if self.current_state not in ("RESOLVING_CONFLICT", "CHARGING"):
                self.current_state = "NAVIGATING"
        elif self.current_state == "NAVIGATING":
            self.current_state = "IDLE"

    def battery_callback(self, msg: Any) -> None:
        """Update local battery percentage reading supporting Float32 and BatteryState."""
        if hasattr(msg, 'percentage'):
            pct = float(msg.percentage)
            self.battery_pct = pct * 100.0 if pct <= 1.0 else pct
        elif hasattr(msg, 'data'):
            self.battery_pct = float(msg.data)

    def status_callback(self, msg: Any) -> None:
        """Update robot state machine status string."""
        self.current_state = msg.data

    def charging_intent_callback(self, msg: Any) -> None:
        """Process charging intent notifications and update robot state."""
        try:
            data = str(msg.data)
            parts = data.split(':')
            action = parts[0].upper()
            if action in ("CHARGING_INTENT", "CHARGING", "CLAIM"):
                self.is_charging = True
                self.current_state = "CHARGING"
            elif action in ("RELEASE", "COMPLETED", "DISCHARGED", "IDLE"):
                self.is_charging = False
                if self.current_state == "CHARGING":
                    self.current_state = "IDLE"
        except Exception as e:
            self.node.get_logger().debug(f"Failed to parse charging intent: {e}")

    def publish_state(self) -> None:
        """Publish updated telemetry state to the fleet mesh network."""
        if self.has_custom_msg and self.RobotState is not None:
            msg = self.RobotState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.robot_id = self.robot_id
            msg.pose.position.x = self.current_pose.x
            msg.pose.position.y = self.current_pose.y
            msg.twist.linear.x = self.current_twist.linear.x
            msg.twist.linear.y = self.current_twist.linear.y
            msg.twist.angular.z = self.current_twist.angular
            msg.battery_percentage = float(self.battery_pct)
            msg.is_charging = self.is_charging
            msg.current_state = self.current_state
            self.state_pub.publish(msg)
        else:
            import json
            from std_msgs.msg import String
            payload = {
                "robot_id": self.robot_id,
                "x": round(self.current_pose.x, 3),
                "y": round(self.current_pose.y, 3),
                "vx": round(self.current_twist.linear.x, 3),
                "battery": round(self.battery_pct, 1),
                "state": self.current_state,
                "time": time.time()
            }
            msg = String()
            msg.data = json.dumps(payload)
            self.state_pub.publish(msg)


def main(args=None) -> None:
    """Entrypoint for the state publisher node."""
    try:
        import rclpy
        from rclpy.node import Node
        try:
            from amr_fleet.msg import RobotState
            has_custom_msg = True
            robot_state_cls = RobotState
        except ImportError:
            has_custom_msg = False
            robot_state_cls = None
    except ImportError:
        print("[StatePublisher] rclpy not found; run in ROS 2 container.")
        return

    rclpy.init(args=args)
    node = Node('state_publisher')
    _ = StatePublisherNode(node, has_custom_msg, robot_state_cls)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
