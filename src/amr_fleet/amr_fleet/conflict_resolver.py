"""
Conflict Resolution & Deadlock Handling Node.
Detects edge conflicts, vertex conflicts, and stationary deadlocks (>3s).
Executes dynamic composite priority negotiation and token-based aisle reservations.
"""

from __future__ import annotations
import math
import time
from typing import Dict, List, Optional, Tuple, Set
from dataclasses import dataclass, field

from amr_fleet.utils import Vector2D, ConflictData, RobotIntentData


@dataclass
class CorridorZone:
    id: str
    name: str
    min_x: float
    min_y: float
    max_x: float
    max_y: float
    holding_point_a: Vector2D
    holding_point_b: Vector2D

    def contains(self, pt: Vector2D) -> bool:
        return self.min_x <= pt.x <= self.max_x and self.min_y <= pt.y <= self.max_y


class ConflictResolver:
    """
    Decentralized conflict detection and priority negotiation engine.
    """
    def __init__(
        self,
        robot_id: str,
        safety_distance: float = 0.8,
        deadlock_timeout_s: float = 3.0,
        speed_threshold: float = 0.05
    ):
        self.robot_id = robot_id
        self.safety_distance = safety_distance
        self.deadlock_timeout_s = deadlock_timeout_s
        self.speed_threshold = speed_threshold

        self.last_moving_time = time.time()
        self.is_in_deadlock = False
        self.active_conflicts: Dict[str, ConflictData] = {}

        # Token-based aisle reservation state
        self.corridor_tokens: Dict[str, str] = {}  # corridor_id -> holding_robot_id
        self.token_request_timestamps: Dict[str, float] = {}

        # Predefined narrow single-lane corridors in warehouse
        self.corridors: List[CorridorZone] = [
            CorridorZone(
                id="corridor_main",
                name="Main Narrow Choke",
                min_x=-1.5, min_y=-1.0, max_x=1.5, max_y=1.0,
                holding_point_a=Vector2D(-2.5, 0.0),
                holding_point_b=Vector2D(2.5, 0.0)
            ),
            CorridorZone(
                id="aisle_central",
                name="Central Single Lane",
                min_x=4.0, min_y=-3.0, max_x=6.0, max_y=3.0,
                holding_point_a=Vector2D(5.0, -4.0),
                holding_point_b=Vector2D(5.0, 4.0)
            )
        ]

    def compute_composite_priority(
        self,
        distance_to_goal: float,
        task_urgency: float,
        battery_percentage: float
    ) -> float:
        """
        Composite priority score:
        - Closer to goal = higher priority (minimizes overall fleet waiting)
        - Higher task urgency = higher priority
        - Lower battery = higher priority (prevents stranding low-battery robots)
        """
        w_dist = 0.45
        w_urgency = 0.35
        w_battery = 0.20

        dist_score = 1.0 / (1.0 + max(0.0, distance_to_goal))
        urgency_score = min(1.0, max(0.0, task_urgency))
        battery_score = 1.0 - (battery_percentage / 100.0)

        composite = (w_dist * dist_score) + (w_urgency * urgency_score) + (w_battery * battery_score)
        return float(composite)

    def detect_trajectory_conflicts(
        self,
        my_path: List[Vector2D],
        peer_intents: Dict[str, RobotIntentData]
    ) -> List[Tuple[str, str, Vector2D]]:
        """
        Detects vertex conflicts, edge conflicts, and corridor contention.
        Returns list of (peer_id, conflict_type, conflict_location).
        """
        conflicts = []
        if not my_path:
            return conflicts

        for peer_id, peer_intent in peer_intents.items():
            if peer_id == self.robot_id:
                continue

            peer_path = peer_intent.planned_path
            if not peer_path:
                continue

            # 1. Edge Conflict check (opposite traversal on overlapping segment)
            for i in range(len(my_path) - 1):
                seg1_start = my_path[i]
                seg1_end = my_path[i + 1]
                for j in range(len(peer_path) - 1):
                    seg2_start = peer_path[j]
                    seg2_end = peer_path[j + 1]

                    # Opposite direction vectors
                    v1 = (seg1_end - seg1_start).normalized()
                    v2 = (seg2_end - seg2_start).normalized()
                    if v1.dot(v2) < -0.7:  # Head-on
                        mid1 = (seg1_start + seg1_end) * 0.5
                        mid2 = (seg2_start + seg2_end) * 0.5
                        if mid1.distance_to(mid2) < self.safety_distance:
                            conflicts.append((peer_id, "EDGE", mid1))
                            break

            # 2. Vertex Conflict check (spatiotemporal intersection)
            check_len = min(len(my_path), len(peer_path))
            for k in range(check_len):
                if my_path[k].distance_to(peer_path[k]) < self.safety_distance:
                    conflicts.append((peer_id, "VERTEX", my_path[k]))
                    break

        return conflicts

    def update_deadlock_state(
        self,
        current_speed: float,
        dist_to_goal: float,
        nearby_peers_stationary: bool,
        current_time: Optional[float] = None
    ) -> bool:
        """
        Monitors velocity. If robot speed < threshold for > 3.0s while goal is not reached,
        and nearby peers are also stalled, flags a deadlock.
        """
        now = current_time if current_time is not None else time.time()

        if current_speed > self.speed_threshold or dist_to_goal < 0.2:
            self.last_moving_time = now
            self.is_in_deadlock = False
            return False

        stall_duration = now - self.last_moving_time
        if stall_duration > self.deadlock_timeout_s and nearby_peers_stationary:
            self.is_in_deadlock = True
            return True

        return False

    def negotiate_priority(
        self,
        peer_id: str,
        my_priority: float,
        peer_priority: float
    ) -> Tuple[str, str]:
        """
        Negotiates resolution action based on composite priority.
        Ties are broken deterministically by robot_id string.
        Returns (my_action, peer_action) where action in ['PROCEED', 'YIELD', 'REPLAN'].
        """
        if my_priority > peer_priority:
            return "PROCEED", "YIELD"
        elif my_priority < peer_priority:
            return "YIELD", "PROCEED"
        else:
            # Deterministic tie-break: lower string ID wins
            if self.robot_id < peer_id:
                return "PROCEED", "YIELD"
            else:
                return "YIELD", "PROCEED"

    def request_corridor_token(self, corridor_id: str, current_time: Optional[float] = None) -> bool:
        """
        Attempts to acquire single-lane corridor token.
        Returns True if token acquired, False if waiting.
        """
        now = current_time if current_time is not None else time.time()
        holding_peer = self.corridor_tokens.get(corridor_id)

        if holding_peer is None or holding_peer == self.robot_id:
            self.corridor_tokens[corridor_id] = self.robot_id
            self.token_request_timestamps[corridor_id] = now
            return True

        # Token held by another robot
        return False

    def release_corridor_token(self, corridor_id: str) -> None:
        if self.corridor_tokens.get(corridor_id) == self.robot_id:
            del self.corridor_tokens[corridor_id]


# ==============================================================================
# ROS 2 Conflict Resolver Node
# ==============================================================================

def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import Twist, PoseStamped, Point
        from nav_msgs.msg import Path
        from std_msgs.msg import String
    except ImportError:
        print("[ConflictResolver] rclpy not detected; run in pure Python or ROS 2 container.")
        return

    rclpy.init(args=args)

    class ConflictResolverNode(Node):
        def __init__(self):
            super().__init__('conflict_resolver')
            self.declare_parameter('robot_id', 'robot_1')
            self.declare_parameter('safety_dist', 0.8)

            self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
            safety_dist = self.declare_parameter('safety_dist', 0.8).get_parameter_value().double_value

            self.resolver = ConflictResolver(self.robot_id, safety_distance=safety_dist)
            self.current_pos = Vector2D(0.0, 0.0)
            self.current_speed = 0.0
            self.goal_pos: Optional[Vector2D] = None
            self.battery_pct = 100.0
            self.planned_path: List[Vector2D] = []
            self.peer_intents: Dict[str, RobotIntentData] = {}

            # Publishers & Subscribers
            self.conflict_pub = self.create_publisher(String, f'/fleet/{self.robot_id}/conflict', 10)
            self.replan_pub = self.create_publisher(Point, f'/fleet/{self.robot_id}/replan_trigger', 10)

            self.create_subscription(PoseStamped, f'/{self.robot_id}/pose', self.pose_callback, 10)
            self.create_subscription(Path, f'/fleet/{self.robot_id}/plan', self.plan_callback, 10)
            self.create_subscription(String, '/fleet/conflict_broadcast', self.conflict_broadcast_callback, 10)

            self.timer = self.create_timer(0.2, self.resolution_loop)  # 5 Hz
            self.get_logger().info(f"ConflictResolver active for {self.robot_id}")

        def pose_callback(self, msg: PoseStamped):
            new_pos = Vector2D(msg.pose.position.x, msg.pose.position.y)
            dt = 0.1
            self.current_speed = (new_pos - self.current_pos).norm() / dt
            self.current_pos = new_pos

        def plan_callback(self, msg: Path):
            self.planned_path = [Vector2D(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
            if self.planned_path:
                self.goal_pos = self.planned_path[-1]

        def conflict_broadcast_callback(self, msg: String):
            # Peer broadcast conflict info
            pass

        def resolution_loop(self):
            if not self.goal_pos:
                return

            dist_to_goal = self.current_pos.distance_to(self.goal_pos)
            my_priority = self.resolver.compute_composite_priority(dist_to_goal, 0.8, self.battery_pct)

            # Check deadlock
            is_deadlock = self.resolver.update_deadlock_state(
                self.current_speed, dist_to_goal, nearby_peers_stationary=True
            )

            if is_deadlock:
                self.get_logger().warn(f"Deadlock detected for {self.robot_id}! Initiating priority negotiation.")
                # Broadcast deadlock conflict
                msg = String()
                msg.data = f"DEADLOCK:{self.robot_id}:{my_priority}:{self.current_pos.x},{self.current_pos.y}"
                self.conflict_pub.publish(msg)

    node = ConflictResolverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
