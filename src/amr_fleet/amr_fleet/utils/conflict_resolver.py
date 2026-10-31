#!/usr/bin/env python3
"""
Conflict Resolution & Deadlock Handling Node.
Detects edge conflicts, vertex conflicts, and stationary deadlocks (>3s).
Executes dynamic composite priority negotiation and token-based aisle reservations.
"""

from __future__ import annotations
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass

from amr_fleet.utils import Vector2D, ConflictData, RobotIntentData


@dataclass
class CorridorZone:
    """Defined narrow corridor / choke point requiring mutex token access."""
    id: str
    name: str
    min_x: float
    min_y: float
    max_x: float
    max_y: float
    holding_point_a: Vector2D
    holding_point_b: Vector2D

    def contains(self, pt: Vector2D) -> bool:
        """Check if 2D point falls inside corridor boundary."""
        return self.min_x <= pt.x <= self.max_x and self.min_y <= pt.y <= self.max_y


class ConflictResolver:
    """Decentralized conflict detection and priority negotiation engine."""
    def __init__(
        self,
        robot_id: str,
        safety_distance: float = 0.8,
        deadlock_timeout_s: float = 3.0,
        speed_threshold: float = 0.05
    ) -> None:
        """Initialize conflict resolver parameters and corridor registry."""
        self.robot_id = robot_id
        self.safety_distance = safety_distance
        self.deadlock_timeout_s = deadlock_timeout_s
        self.speed_threshold = speed_threshold

        self.last_moving_time = time.time()
        self.is_in_deadlock = False
        self.active_conflicts: Dict[str, ConflictData] = {}

        self.corridor_tokens: Dict[str, str] = {}
        self.token_request_timestamps: Dict[str, float] = {}

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
        """Compute composite priority score based on distance, urgency, and battery state."""
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
        """Detect edge conflicts, vertex conflicts, and corridor contention."""
        conflicts: List[Tuple[str, str, Vector2D]] = []
        if not my_path:
            return conflicts

        for peer_id, peer_intent in peer_intents.items():
            if peer_id == self.robot_id:
                continue

            peer_path = peer_intent.planned_path
            if not peer_path:
                continue

            for i in range(len(my_path) - 1):
                seg1_start = my_path[i]
                seg1_end = my_path[i + 1]
                for j in range(len(peer_path) - 1):
                    seg2_start = peer_path[j]
                    seg2_end = peer_path[j + 1]

                    v1 = (seg1_end - seg1_start).normalized()
                    v2 = (seg2_end - seg2_start).normalized()
                    if v1.dot(v2) < -0.7:
                        mid1 = (seg1_start + seg1_end) * 0.5
                        mid2 = (seg2_start + seg2_end) * 0.5
                        if mid1.distance_to(mid2) < self.safety_distance:
                            conflicts.append((peer_id, "EDGE", mid1))
                            break

            check_len = min(len(my_path), len(peer_path))
            for k in range(check_len):
                if my_path[k].distance_to(peer_path[k]) < self.safety_distance:
                    conflicts.append((peer_id, "VERTEX", my_path[k]))
                    break

        return conflicts

    def compute_nearby_peers_stationary(
        self,
        my_pos: Vector2D,
        peer_positions: Dict[str, Vector2D],
        peer_speeds: Dict[str, float],
        radius: float = 2.5,
        speed_threshold: Optional[float] = None
    ) -> bool:
        """
        Compute whether nearby peers within radius are stationary.
        Returns False if no peers are within radius or if any nearby peer is moving (> threshold).
        Returns True only if at least one peer is nearby and ALL nearby peers are stationary.
        """
        thresh = speed_threshold if speed_threshold is not None else self.speed_threshold
        nearby_count = 0
        for peer_id, pos in peer_positions.items():
            if peer_id == self.robot_id:
                continue
            if my_pos.distance_to(pos) <= radius:
                nearby_count += 1
                spd = peer_speeds.get(peer_id, 0.0)
                if spd > thresh:
                    return False
        return nearby_count > 0

    def update_deadlock_state(
        self,
        current_speed: float,
        dist_to_goal: float,
        nearby_peers_stationary: bool,
        current_time: Optional[float] = None
    ) -> bool:
        """Flag deadlock if robot remains stationary for > 3.0s with goal not reached."""
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
        """Negotiate action based on priority score, breaking ties lexicographically."""
        if my_priority > peer_priority:
            return "PROCEED", "YIELD"
        elif my_priority < peer_priority:
            return "YIELD", "PROCEED"
        else:
            if self.robot_id < peer_id:
                return "PROCEED", "YIELD"
            else:
                return "YIELD", "PROCEED"

    def request_corridor_token(self, corridor_id: str, current_time: Optional[float] = None) -> bool:
        """Attempt to acquire mutual exclusion token for single-lane aisle passage."""
        now = current_time if current_time is not None else time.time()
        holding_peer = self.corridor_tokens.get(corridor_id)

        if holding_peer is None or holding_peer == self.robot_id:
            self.corridor_tokens[corridor_id] = self.robot_id
            self.token_request_timestamps[corridor_id] = now
            return True

        return False

    def release_corridor_token(self, corridor_id: str) -> None:
        """Release previously acquired corridor passage token."""
        if self.corridor_tokens.get(corridor_id) == self.robot_id:
            del self.corridor_tokens[corridor_id]
            self.token_request_timestamps.pop(corridor_id, None)

    def prune_stale_tokens(self, timeout_s: float = 10.0, current_time: Optional[float] = None) -> List[str]:
        """Prune tokens held by peers that haven't refreshed within timeout_s."""
        now = current_time if current_time is not None else time.time()
        pruned = []
        for corridor_id, holding_peer in list(self.corridor_tokens.items()):
            if holding_peer != self.robot_id:
                req_time = self.token_request_timestamps.get(corridor_id, 0.0)
                if now - req_time > timeout_s:
                    del self.corridor_tokens[corridor_id]
                    self.token_request_timestamps.pop(corridor_id, None)
                    pruned.append(corridor_id)
        return pruned


class ConflictResolverNode:
    """ROS 2 Node wrapper for decentralized conflict resolution."""
    def __init__(self, node: Any) -> None:
        """Initialize conflict resolver subscribers, publishers, and timer loop."""
        self.node = node
        self.node.declare_parameter('robot_id', 'robot_1')
        self.node.declare_parameter('safety_dist', 0.8)

        self.robot_id = self.node.get_parameter('robot_id').get_parameter_value().string_value
        safety_dist = self.node.get_parameter('safety_dist').get_parameter_value().double_value

        self.resolver = ConflictResolver(self.robot_id, safety_distance=safety_dist)
        self.current_pos = Vector2D(0.0, 0.0)
        self.current_speed = 0.0
        self.last_pose_time = time.time()
        self.goal_pos: Optional[Vector2D] = None
        self.battery_pct = 100.0
        self.planned_path: List[Vector2D] = []
        self.peer_intents: Dict[str, RobotIntentData] = {}
        self.peer_positions: Dict[str, Vector2D] = {}
        self.peer_speeds: Dict[str, float] = {}
        self.peer_last_seen: Dict[str, float] = {}

        from geometry_msgs.msg import PoseStamped, Point
        from nav_msgs.msg import Path
        from std_msgs.msg import String, Float32

        self.conflict_pub = self.node.create_publisher(String, f'/fleet/{self.robot_id}/conflict', 10)
        self.fleet_conflict_pub = self.node.create_publisher(String, '/fleet/conflicts', 10)
        self.replan_pub = self.node.create_publisher(Point, f'/fleet/{self.robot_id}/replan_trigger', 10)
        self.priority_pub = self.node.create_publisher(Float32, f'/fleet/{self.robot_id}/priority', 10)
        self.token_pub = self.node.create_publisher(String, f'/fleet/{self.robot_id}/token_status', 10)

        self.node.create_subscription(PoseStamped, f'/{self.robot_id}/pose', self.pose_callback, 10)
        self.node.create_subscription(Path, f'/fleet/{self.robot_id}/plan', self.plan_callback, 10)
        self.node.create_subscription(Float32, f'/fleet/{self.robot_id}/battery', self.battery_callback, 10)
        self.node.create_subscription(String, '/fleet/conflicts', self.conflict_broadcast_callback, 10)

        self.timer = self.node.create_timer(0.2, self.resolution_loop)
        self.node.get_logger().info(f"ConflictResolver active for {self.robot_id}")

    def pose_callback(self, msg: Any) -> None:
        """Update robot pose and estimate instantaneous linear velocity."""
        new_pos = Vector2D(msg.pose.position.x, msg.pose.position.y)
        now = time.time()
        dt = max(0.01, now - self.last_pose_time)
        self.current_speed = (new_pos - self.current_pos).norm() / dt
        self.last_pose_time = now
        self.current_pos = new_pos

    def battery_callback(self, msg: Any) -> None:
        """Update robot battery reading."""
        self.battery_pct = float(msg.data)

    def plan_callback(self, msg: Any) -> None:
        """Update active path and target goal point."""
        self.planned_path = [Vector2D(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        if self.planned_path:
            self.goal_pos = self.planned_path[-1]

    def conflict_broadcast_callback(self, msg: Any) -> None:
        """Handle peer broadcast deadlock messages and token requests."""
        from geometry_msgs.msg import Point
        try:
            parts = msg.data.split(':')
            cmd = parts[0]
            if cmd == "DEADLOCK":
                peer_id = parts[1]
                peer_pri = float(parts[2])
                if peer_id != self.robot_id and self.goal_pos:
                    dist_to_goal = self.current_pos.distance_to(self.goal_pos)
                    my_priority = self.resolver.compute_composite_priority(dist_to_goal, 0.8, self.battery_pct)
                    my_act, _ = self.resolver.negotiate_priority(peer_id, my_priority, peer_pri)
                    self.node.get_logger().info(f"Deadlock negotiation with {peer_id}: My action = {my_act}")
                    if my_act == "YIELD":
                        pt = Point()
                        pt.x = self.current_pos.x
                        pt.y = self.current_pos.y
                        self.replan_pub.publish(pt)
            elif cmd == "TOKEN_REQ":
                peer_id = parts[1]
                corridor_id = parts[2]
                if peer_id != self.robot_id:
                    self.resolver.corridor_tokens[corridor_id] = peer_id
            elif cmd == "TOKEN_REL":
                peer_id = parts[1]
                corridor_id = parts[2]
                if self.resolver.corridor_tokens.get(corridor_id) == peer_id:
                    del self.resolver.corridor_tokens[corridor_id]
        except Exception as e:
            self.node.get_logger().debug(f"Conflict broadcast parse: {e}")

    def request_corridor_token(self, corridor_id: str) -> bool:
        """Request corridor token, broadcast TOKEN_REQ to fleet, and publish status."""
        from std_msgs.msg import String
        holding_peer = self.resolver.corridor_tokens.get(corridor_id)
        if holding_peer is not None and holding_peer != self.robot_id:
            return False

        dist_to_goal = self.current_pos.distance_to(self.goal_pos) if self.goal_pos else 5.0
        my_priority = self.resolver.compute_composite_priority(dist_to_goal, 0.8, self.battery_pct)

        # Broadcast TOKEN_REQ to fleet
        req_msg = String()
        req_msg.data = f"TOKEN_REQ:{self.robot_id}:{corridor_id}:{my_priority:.3f}"
        self.fleet_conflict_pub.publish(req_msg)

        granted = self.resolver.request_corridor_token(corridor_id)
        if granted:
            msg = String()
            msg.data = f"{corridor_id}:HELD"
            self.token_pub.publish(msg)
        return granted

    def release_corridor_token(self, corridor_id: str) -> None:
        """Release corridor token, broadcast TOKEN_REL to fleet, and publish status."""
        from std_msgs.msg import String
        if self.resolver.corridor_tokens.get(corridor_id) == self.robot_id:
            self.resolver.release_corridor_token(corridor_id)

            rel_msg = String()
            rel_msg.data = f"TOKEN_REL:{self.robot_id}:{corridor_id}"
            self.fleet_conflict_pub.publish(rel_msg)

            msg = String()
            msg.data = f"{corridor_id}:RELEASED"
            self.token_pub.publish(msg)

    def update_peer_state(self, peer_id: str, pos: Vector2D, speed: float) -> None:
        """Update tracked position and velocity for peer AMR."""
        if peer_id == self.robot_id:
            return
        now = time.time()
        self.peer_positions[peer_id] = pos
        self.peer_speeds[peer_id] = speed
        self.peer_last_seen[peer_id] = now

    def prune_stale_peers(self, timeout_s: float = 10.0, current_time: Optional[float] = None) -> List[str]:
        """Prune tracking records for peer robots that have ceased communication."""
        now = current_time if current_time is not None else time.time()
        stale = [peer_id for peer_id, last_time in list(self.peer_last_seen.items()) if now - last_time > timeout_s]
        for peer_id in stale:
            self.peer_positions.pop(peer_id, None)
            self.peer_speeds.pop(peer_id, None)
            self.peer_last_seen.pop(peer_id, None)
            self.peer_intents.pop(peer_id, None)
        self.resolver.prune_stale_tokens(timeout_s=timeout_s, current_time=now)
        return stale

    def resolution_loop(self) -> None:
        """Periodic conflict and deadlock monitoring loop."""
        from std_msgs.msg import String, Float32
        self.prune_stale_peers(timeout_s=10.0)
        if not self.goal_pos:
            return

        dist_to_goal = self.current_pos.distance_to(self.goal_pos)
        my_priority = self.resolver.compute_composite_priority(dist_to_goal, 0.8, self.battery_pct)

        p_msg = Float32()
        p_msg.data = float(my_priority)
        self.priority_pub.publish(p_msg)

        nearby_stationary = self.resolver.compute_nearby_peers_stationary(
            self.current_pos, self.peer_positions, self.peer_speeds
        )

        is_deadlock = self.resolver.update_deadlock_state(
            self.current_speed, dist_to_goal, nearby_peers_stationary=nearby_stationary
        )

        if is_deadlock:
            self.node.get_logger().warn(f"Deadlock detected for {self.robot_id}! Initiating priority negotiation.")
            msg = String()
            msg.data = f"DEADLOCK:{self.robot_id}:{my_priority}:{self.current_pos.x:.2f},{self.current_pos.y:.2f}"
            self.conflict_pub.publish(msg)
            self.fleet_conflict_pub.publish(msg)


def main(args=None) -> None:
    """Entrypoint for conflict resolver node."""
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        print("[ConflictResolver] rclpy not detected; run in pure Python or ROS 2 container.")
        return

    rclpy.init(args=args)
    node = Node('conflict_resolver')
    _ = ConflictResolverNode(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
