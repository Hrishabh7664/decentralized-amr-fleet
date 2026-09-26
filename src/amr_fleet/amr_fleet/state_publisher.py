#!/usr/bin/env python3
"""
State Publisher Node.
Publishes telemetry (pose, twist, battery, operating state) to /fleet/{id}/state
at 10-20 Hz using BEST_EFFORT QoS with a 100ms deadline.
Also publishes PoseStamped to /{id}/pose for internal node consumption.
"""

from __future__ import annotations
import math
import time

from amr_fleet.utils import Pose2D, Twist2D, Vector2D, get_state_qos, quaternion_to_yaw


def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
        from nav_msgs.msg import Odometry
        from std_msgs.msg import Float32, String
        from geometry_msgs.msg import PoseStamped, Twist
        try:
            from amr_fleet.msg import RobotState
            has_custom_msg = True
        except ImportError:
            has_custom_msg = False
    except ImportError:
        print("[StatePublisher] rclpy not found; run in ROS 2 container.")
        return

    rclpy.init(args=args)

    class StatePublisherNode(Node):
        def __init__(self):
            super().__init__('state_publisher')
            self.declare_parameter('robot_id', 'robot_1')
            self.declare_parameter('publish_rate_hz', 20.0)

            self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
            self.rate_hz = self.get_parameter('publish_rate_hz').get_parameter_value().double_value

            self.current_pose = Pose2D()
            self.current_twist = Twist2D()
            self.battery_pct = 100.0
            self.is_charging = False
            self.current_state = "IDLE"

            qos = get_state_qos()
            if has_custom_msg:
                self.state_pub = self.create_publisher(RobotState, f'/fleet/{self.robot_id}/state', qos)
            else:
                self.state_pub = self.create_publisher(String, f'/fleet/{self.robot_id}/state', 10)

            # Internal pose publisher so other onboard nodes receive PoseStamped
            self.pose_pub = self.create_publisher(PoseStamped, f'/{self.robot_id}/pose', 10)

            # Subscriptions
            self.create_subscription(Odometry, f'/{self.robot_id}/odom', self.odom_callback, 10)
            self.create_subscription(Float32, f'/fleet/{self.robot_id}/battery', self.battery_callback, 10)
            self.create_subscription(String, f'/fleet/{self.robot_id}/status', self.status_callback, 10)

            self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_state)
            self.get_logger().info(f"StatePublisher active on /fleet/{self.robot_id}/state at {self.rate_hz} Hz")

        def odom_callback(self, msg: Odometry):
            self.current_pose.x = msg.pose.pose.position.x
            self.current_pose.y = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            self.current_pose.theta = quaternion_to_yaw(q.x, q.y, q.z, q.w)

            self.current_twist.linear.x = msg.twist.twist.linear.x
            self.current_twist.linear.y = msg.twist.twist.linear.y
            self.current_twist.angular = msg.twist.twist.angular.z

            # Publish PoseStamped so other onboard nodes receive current pose
            ps = PoseStamped()
            ps.header = msg.header
            ps.pose = msg.pose.pose
            self.pose_pub.publish(ps)

            if abs(self.current_twist.linear.x) > 0.05 or abs(self.current_twist.angular) > 0.05:
                if self.current_state != "RESOLVING_CONFLICT":
                    self.current_state = "NAVIGATING"
            elif self.current_state == "NAVIGATING":
                self.current_state = "IDLE"

        def battery_callback(self, msg: Float32):
            self.battery_pct = msg.data

        def status_callback(self, msg: String):
            self.current_state = msg.data

        def publish_state(self):
            if has_custom_msg:
                msg = RobotState()
                msg.header.stamp = self.get_clock().now().to_msg()
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

    node = StatePublisherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
