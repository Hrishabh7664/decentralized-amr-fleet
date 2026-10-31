#!/usr/bin/env python3
"""
Decentralized Auction-Based Task Allocator with Battery Awareness.
Any robot can act as an auctioneer to broadcast tasks and collect bids.
Includes blocked aisle re-auctioning and charging threshold exclusions.
"""

from __future__ import annotations
import time
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass, field

from amr_fleet.utils import Vector2D, TaskBidData


@dataclass
class WarehouseTask:
    """Representation of an order or material transfer task in the warehouse."""
    task_id: str
    pickup_location: Vector2D
    dropoff_location: Vector2D
    urgency: float = 0.5
    assigned_robot_id: Optional[str] = None
    status: str = "PENDING"
    created_at: float = field(default_factory=time.time)


class TaskAllocator:
    """Peer-to-peer auction protocol engine for decentralized AMR task allocation."""
    def __init__(
        self,
        robot_id: str,
        nominal_speed: float = 0.6,
        bid_timeout_s: float = 0.5
    ) -> None:
        """Initialize the task allocator with robot ID and auction parameters."""
        self.robot_id = robot_id
        self.nominal_speed = nominal_speed
        self.bid_timeout_s = bid_timeout_s
        self.current_task: Optional[WarehouseTask] = None
        self.task_queue: List[WarehouseTask] = []
        self.active_auctions: Dict[str, Dict[str, Any]] = {}

    def compute_bid(
        self,
        task: WarehouseTask,
        current_pos: Vector2D,
        battery_percentage: float,
        congestion_factor: float = 0.0
    ) -> TaskBidData:
        """Compute marginal cost to perform task, excluding robots below 20% battery unless urgent."""
        is_urgent = task.urgency >= 0.9
        if battery_percentage < 20.0 and not is_urgent:
            return TaskBidData(
                task_id=task.task_id,
                robot_id=self.robot_id,
                bid_cost=float('inf'),
                estimated_distance=0.0,
                battery_percentage=battery_percentage,
                queue_length=len(self.task_queue),
                eligible=False
            )

        dist_to_pickup = current_pos.distance_to(task.pickup_location)
        delivery_dist = task.pickup_location.distance_to(task.dropoff_location)
        total_dist = dist_to_pickup + delivery_dist
        travel_time = total_dist / max(0.1, self.nominal_speed)

        w_time = 0.50
        w_battery = 0.25
        w_queue = 0.15
        w_congestion = 0.10

        battery_penalty = 1.0 - (battery_percentage / 100.0)
        queue_penalty = len(self.task_queue) * 15.0

        cost = (
            (w_time * travel_time) +
            (w_battery * battery_penalty * 100.0) +
            (w_queue * queue_penalty) +
            (w_congestion * congestion_factor * 20.0)
        )

        return TaskBidData(
            task_id=task.task_id,
            robot_id=self.robot_id,
            bid_cost=cost,
            estimated_distance=total_dist,
            battery_percentage=battery_percentage,
            queue_length=len(self.task_queue),
            eligible=True
        )

    def start_auction(self, task: WarehouseTask, current_time: Optional[float] = None) -> None:
        """Acts as auctioneer and opens bidding for a task."""
        now = current_time if current_time is not None else time.time()
        task.status = "AUCTIONING"
        self.active_auctions[task.task_id] = {
            "task": task,
            "start_time": now,
            "bids": {},
            "resolved": False
        }

    def receive_bid(self, bid: TaskBidData) -> None:
        """Register an eligible bid received from a peer robot."""
        if bid.task_id in self.active_auctions:
            auction = self.active_auctions[bid.task_id]
            if not auction["resolved"] and bid.eligible:
                auction["bids"][bid.robot_id] = bid

    def resolve_auction(
        self,
        task_id: str,
        current_time: Optional[float] = None
    ) -> Optional[Tuple[str, float]]:
        """Determines lowest bidder once auction window expires, breaking ties lexicographically."""
        now = current_time if current_time is not None else time.time()
        if task_id not in self.active_auctions:
            return None

        auction = self.active_auctions[task_id]
        if auction["resolved"]:
            return None

        if (now - auction["start_time"]) < self.bid_timeout_s:
            return None

        auction["resolved"] = True
        bids: Dict[str, TaskBidData] = auction["bids"]
        if not bids:
            auction["task"].status = "PENDING"
            return None

        winner_id = min(bids.keys(), key=lambda r: (bids[r].bid_cost, r))
        winning_cost = bids[winner_id].bid_cost

        task = auction["task"]
        task.assigned_robot_id = winner_id
        task.status = "ASSIGNED"

        if winner_id == self.robot_id:
            self.current_task = task

        return (winner_id, winning_cost)

    def handle_blocked_aisle_re_auction(self, blocked_task_id: str) -> Optional[WarehouseTask]:
        """Cancels current assignment and releases task for re-auction if route is blocked."""
        if self.current_task and self.current_task.task_id == blocked_task_id:
            re_task = self.current_task
            re_task.assigned_robot_id = None
            re_task.status = "PENDING"
            self.current_task = None
            return re_task
        return None


class TaskAllocatorNode:
    """ROS 2 Node wrapper for decentralized task allocation."""
    def __init__(self, node: Any) -> None:
        """Initialize ROS 2 subscribers, publishers, and auction resolution loop."""
        self.node = node
        self.node.declare_parameter('robot_id', 'robot_1')
        self.robot_id = self.node.get_parameter('robot_id').get_parameter_value().string_value

        self.allocator = TaskAllocator(self.robot_id)
        self.current_pos = Vector2D(0.0, 0.0)
        self.battery_percentage = 100.0

        from std_msgs.msg import String, Float32
        from geometry_msgs.msg import PoseStamped, Point

        self.bid_pub = self.node.create_publisher(String, f'/fleet/{self.robot_id}/bid', 10)
        self.fleet_bid_pub = self.node.create_publisher(String, '/fleet/bids', 10)
        self.assignment_pub = self.node.create_publisher(String, '/fleet/task_assignments', 10)
        self.goal_pub = self.node.create_publisher(Point, f'/fleet/{self.robot_id}/goal', 10)

        self.node.create_subscription(PoseStamped, f'/{self.robot_id}/pose', self.pose_callback, 10)
        self.node.create_subscription(Float32, f'/fleet/{self.robot_id}/battery', self.battery_callback, 10)
        self.node.create_subscription(String, '/fleet/new_tasks', self.new_task_callback, 10)
        self.node.create_subscription(String, '/fleet/bids', self.peer_bid_callback, 10)
        self.node.create_subscription(String, '/fleet/task_assignments', self.assignment_callback, 10)

        self.timer = self.node.create_timer(0.1, self.auction_tick)
        self.node.get_logger().info(f"TaskAllocator initialized for {self.robot_id}")

    def pose_callback(self, msg: Any) -> None:
        """Update robot current 2D pose."""
        self.current_pos = Vector2D(msg.pose.position.x, msg.pose.position.y)

    def battery_callback(self, msg: Any) -> None:
        """Update robot battery percentage."""
        self.battery_percentage = float(msg.data)

    def new_task_callback(self, msg: Any) -> None:
        """Process incoming task, compute bid, and announce to fleet mesh."""
        from std_msgs.msg import String
        try:
            parts = msg.data.split(':')
            task_id = parts[0]
            px, py = [float(v) for v in parts[1].split(',')]
            dx, dy = [float(v) for v in parts[2].split(',')]
            urgency = float(parts[3]) if len(parts) > 3 else 0.5

            task = WarehouseTask(
                task_id=task_id,
                pickup_location=Vector2D(px, py),
                dropoff_location=Vector2D(dx, dy),
                urgency=urgency
            )

            bid = self.allocator.compute_bid(task, self.current_pos, self.battery_percentage)
            if bid.eligible:
                bid_msg = String()
                bid_msg.data = f"{bid.task_id},{bid.robot_id},{bid.bid_cost:.2f},{bid.estimated_distance:.2f},{self.battery_percentage:.1f}"
                self.bid_pub.publish(bid_msg)
                self.fleet_bid_pub.publish(bid_msg)

            if task_id not in self.allocator.active_auctions:
                self.allocator.start_auction(task)
                if bid.eligible:
                    self.allocator.receive_bid(bid)
        except Exception as e:
            self.node.get_logger().warn(f"Failed to process task: {e}")

    def peer_bid_callback(self, msg: Any) -> None:
        """Process incoming bids submitted by peer AMRs."""
        try:
            parts = msg.data.split(',')
            if len(parts) >= 3:
                task_id = parts[0]
                robot_id = parts[1]
                bid_cost = float(parts[2])
                dist = float(parts[3]) if len(parts) > 3 else 0.0
                bat = float(parts[4]) if len(parts) > 4 else 100.0
                bid_data = TaskBidData(
                    task_id=task_id,
                    robot_id=robot_id,
                    bid_cost=bid_cost,
                    estimated_distance=dist,
                    battery_percentage=bat,
                    eligible=(bid_cost < float('inf'))
                )
                self.allocator.receive_bid(bid_data)
        except Exception as e:
            self.node.get_logger().debug(f"Failed to parse peer bid: {e}")

    def assignment_callback(self, msg: Any) -> None:
        """Handle winning task awards."""
        from geometry_msgs.msg import Point
        try:
            parts = msg.data.split(':')
            if len(parts) >= 2:
                task_id = parts[0]
                winner_id = parts[1]
                if winner_id == self.robot_id and len(parts) >= 3:
                    px, py = [float(v) for v in parts[2].split(',')]
                    self.node.get_logger().info(f"Task {task_id} assigned to {self.robot_id}! Moving to ({px}, {py}).")
                    goal = Point()
                    goal.x = px
                    goal.y = py
                    self.goal_pub.publish(goal)
        except Exception as e:
            self.node.get_logger().debug(f"Assignment callback parse: {e}")

    def auction_tick(self) -> None:
        """Periodically resolve expired auction windows."""
        from std_msgs.msg import String
        from geometry_msgs.msg import Point
        for task_id in list(self.allocator.active_auctions.keys()):
            result = self.allocator.resolve_auction(task_id)
            if result:
                winner_id, cost = result
                auction = self.allocator.active_auctions[task_id]
                task = auction["task"]
                self.node.get_logger().info(f"Task {task_id} awarded to {winner_id} with bid {cost:.2f}")

                assign_msg = String()
                assign_msg.data = f"{task_id}:{winner_id}:{task.pickup_location.x:.2f},{task.pickup_location.y:.2f}"
                self.assignment_pub.publish(assign_msg)

                if winner_id == self.robot_id:
                    goal = Point()
                    goal.x = task.pickup_location.x
                    goal.y = task.pickup_location.y
                    self.goal_pub.publish(goal)


def main(args=None) -> None:
    """Entrypoint for task allocator node."""
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        print("[TaskAllocator] rclpy not detected; run in ROS 2 container.")
        return

    rclpy.init(args=args)
    node = Node('task_allocator')
    _ = TaskAllocatorNode(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
