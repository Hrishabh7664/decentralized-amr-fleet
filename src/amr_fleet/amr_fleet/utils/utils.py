"""
Utility classes, geometric primitives, QoS profiles, and multi-hop relay logic.
Provides pure Python implementations and fallbacks for environments without compiled ROS 2 msgs.
"""

from __future__ import annotations
import math
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict, Any


# ==============================================================================
# 1. 2D Geometric Primitives
# ==============================================================================

@dataclass
class Vector2D:
    """A 2D spatial vector supporting standard algebraic and geometric operations."""
    x: float = 0.0
    y: float = 0.0

    def __add__(self, other: Vector2D) -> Vector2D:
        return Vector2D(self.x + other.x, self.y + other.y)

    def __sub__(self, other: Vector2D) -> Vector2D:
        return Vector2D(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> Vector2D:
        return Vector2D(self.x * scalar, self.y * scalar)

    def __rmul__(self, scalar: float) -> Vector2D:
        return self.__mul__(scalar)

    def __truediv__(self, scalar: float) -> Vector2D:
        if abs(scalar) < 1e-9:
            raise ZeroDivisionError("Division by zero in Vector2D")
        return Vector2D(self.x / scalar, self.y / scalar)

    def __neg__(self) -> Vector2D:
        return Vector2D(-self.x, -self.y)

    def dot(self, other: Vector2D) -> float:
        """Dot product of two 2D vectors."""
        return self.x * other.x + self.y * other.y

    def det(self, other: Vector2D) -> float:
        """2D determinant / cross product: self.x * other.y - self.y * other.x."""
        return self.x * other.y - self.y * other.x

    def norm_sq(self) -> float:
        """Squared Euclidean norm."""
        return self.x * self.x + self.y * self.y

    def norm(self) -> float:
        """Euclidean norm."""
        return math.hypot(self.x, self.y)

    def normalized(self) -> Vector2D:
        """Return unit vector or zero vector if length is negligible."""
        n = self.norm()
        if n < 1e-9:
            return Vector2D(0.0, 0.0)
        return Vector2D(self.x / n, self.y / n)

    def distance_to(self, other: Vector2D) -> float:
        """Euclidean distance to another Vector2D."""
        return (self - other).norm()

    def rotate(self, angle_rad: float) -> Vector2D:
        """Rotate vector counter-clockwise by angle in radians."""
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)
        return Vector2D(self.x * cos_a - self.y * sin_a, self.x * sin_a + self.y * cos_a)

    def to_tuple(self) -> Tuple[float, float]:
        """Convert vector to (x, y) tuple."""
        return (self.x, self.y)


@dataclass
class Pose2D:
    """2D robot pose with planar position and heading angle in radians."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0

    @property
    def position(self) -> Vector2D:
        """Get planar position as Vector2D."""
        return Vector2D(self.x, self.y)

    def distance_to(self, other: Pose2D | Vector2D) -> float:
        """Distance to another pose or vector position."""
        if isinstance(other, Pose2D):
            return math.hypot(self.x - other.x, self.y - other.y)
        return math.hypot(self.x - other.x, self.y - other.y)


@dataclass
class Twist2D:
    """Planar velocity command with 2D linear vector and angular yaw velocity."""
    linear: Vector2D = field(default_factory=Vector2D)
    angular: float = 0.0


def quaternion_to_yaw(q_x: float, q_y: float, q_z: float, q_w: float) -> float:
    """Extract planar yaw heading angle in radians from quaternion components."""
    siny_cosp = 2.0 * (q_w * q_z + q_x * q_y)
    cosy_cosp = 1.0 - 2.0 * (q_y * q_y + q_z * q_z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    """Convert yaw heading angle into quaternion (x, y, z, w)."""
    half_yaw = yaw * 0.5
    return (0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw))


def normalize_angle(angle: float) -> float:
    """Wrap angle into interval [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


# ==============================================================================
# 2. Grid & Coordinate Mapping
# ==============================================================================

def world_to_grid(
    world_x: float,
    world_y: float,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    resolution: float = 0.5
) -> Tuple[int, int]:
    """Convert continuous Cartesian coordinates to integer grid cell indices."""
    gx = int(math.floor((world_x - origin_x) / resolution))
    gy = int(math.floor((world_y - origin_y) / resolution))
    return (gx, gy)


def grid_to_world(
    gx: int,
    gy: int,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    resolution: float = 0.5
) -> Tuple[float, float]:
    """Convert integer grid cell indices to continuous center coordinates."""
    wx = (gx + 0.5) * resolution + origin_x
    wy = (gy + 0.5) * resolution + origin_y
    return (wx, wy)


# ==============================================================================
# 3. Pure Python Message Dataclasses
# ==============================================================================

@dataclass
class RobotStateData:
    """Telemetry data structure representing operational AMR status."""
    robot_id: str
    pose: Pose2D = field(default_factory=Pose2D)
    twist: Twist2D = field(default_factory=Twist2D)
    battery_percentage: float = 100.0
    is_charging: bool = False
    current_state: str = "IDLE"
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        """Convert telemetry to serializable dictionary."""
        return {
            "robot_id": self.robot_id,
            "pose": {"x": self.pose.x, "y": self.pose.y, "theta": self.pose.theta},
            "twist": {"vx": self.twist.linear.x, "vy": self.twist.linear.y, "omega": self.twist.angular},
            "battery_percentage": round(self.battery_percentage, 1),
            "is_charging": self.is_charging,
            "current_state": self.current_state,
            "timestamp": self.timestamp,
        }


@dataclass
class RobotIntentData:
    """Intended path, corridor token, and priority score for collision avoidance."""
    robot_id: str
    task_id: str = ""
    current_goal: Vector2D = field(default_factory=Vector2D)
    planned_path: List[Vector2D] = field(default_factory=list)
    estimated_arrival_time: float = 0.0
    priority_score: float = 0.0
    has_token: bool = False
    reserved_corridor_id: str = ""
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        """Convert intent to serializable dictionary."""
        return {
            "robot_id": self.robot_id,
            "task_id": self.task_id,
            "current_goal": {"x": self.current_goal.x, "y": self.current_goal.y},
            "planned_path": [{"x": p.x, "y": p.y} for p in self.planned_path],
            "estimated_arrival_time": round(self.estimated_arrival_time, 2),
            "priority_score": round(self.priority_score, 3),
            "has_token": self.has_token,
            "reserved_corridor_id": self.reserved_corridor_id,
            "timestamp": self.timestamp,
        }


@dataclass
class TaskBidData:
    """Marginal cost bid submitted by an AMR during decentralized task auctions."""
    task_id: str
    robot_id: str
    bid_cost: float
    estimated_distance: float
    battery_percentage: float
    queue_length: int = 0
    eligible: bool = True
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        """Convert bid data to serializable dictionary."""
        return {
            "task_id": self.task_id,
            "robot_id": self.robot_id,
            "bid_cost": round(self.bid_cost, 3),
            "estimated_distance": round(self.estimated_distance, 2),
            "battery_percentage": round(self.battery_percentage, 1),
            "queue_length": self.queue_length,
            "eligible": self.eligible,
            "timestamp": self.timestamp,
        }


@dataclass
class ConflictData:
    """Spatiotemporal collision conflict record between two robots."""
    conflict_id: str
    initiating_robot_id: str
    conflicting_robot_id: str
    conflict_type: str
    conflict_location: Vector2D = field(default_factory=Vector2D)
    initiating_priority: float = 0.0
    conflicting_priority: float = 0.0
    resolution_action: str = "PROCEED"
    resolved: bool = False
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        """Convert conflict record to serializable dictionary."""
        return {
            "conflict_id": self.conflict_id,
            "initiating_robot_id": self.initiating_robot_id,
            "conflicting_robot_id": self.conflicting_robot_id,
            "conflict_type": self.conflict_type,
            "conflict_location": {"x": self.conflict_location.x, "y": self.conflict_location.y},
            "initiating_priority": round(self.initiating_priority, 3),
            "conflicting_priority": round(self.conflicting_priority, 3),
            "resolution_action": self.resolution_action,
            "resolved": self.resolved,
            "timestamp": self.timestamp,
        }


# ==============================================================================
# 4. Multi-Hop Relay Cache
# ==============================================================================

class MessageRelayCache:
    """Suppresses redundant message flooding while enabling mesh relay across Wi-Fi gaps."""
    def __init__(self, ttl_seconds: float = 10.0, max_cache_size: int = 1000) -> None:
        """Initialize the message relay cache with time-to-live and capacity limit."""
        self.ttl = ttl_seconds
        self.max_cache_size = max_cache_size
        self._seen_messages: Dict[str, float] = {}

    def should_forward(self, message_id: str, current_time: Optional[float] = None) -> bool:
        """Returns True if message has not been seen within TTL window, false otherwise."""
        now = current_time if current_time is not None else time.time()
        self._cleanup(now)
        if message_id in self._seen_messages:
            return False
        if len(self._seen_messages) >= self.max_cache_size:
            oldest_id = min(self._seen_messages, key=self._seen_messages.get)
            del self._seen_messages[oldest_id]
        self._seen_messages[message_id] = now
        return True

    def _cleanup(self, now: float) -> None:
        """Purge expired entries from the seen messages registry."""
        expired = [msg_id for msg_id, ts in self._seen_messages.items() if now - ts > self.ttl]
        for msg_id in expired:
            del self._seen_messages[msg_id]


# ==============================================================================
# 5. ROS 2 QoS Profile Factory Helpers
# ==============================================================================

def get_state_qos() -> Any:
    """Return QoS profile for 10-20 Hz telemetry state topic."""
    try:
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        from rclpy.duration import Duration
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            deadline=Duration(seconds=0, nanoseconds=100_000_000)
        )
    except ImportError:
        return None


def get_intent_qos() -> Any:
    """Return QoS profile for intent and planned trajectory sharing."""
    try:
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
    except ImportError:
        return None


def get_bid_qos() -> Any:
    """Return QoS profile for auction bids."""
    try:
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
    except ImportError:
        return None


def get_conflict_qos() -> Any:
    """Return QoS profile for conflict negotiation."""
    try:
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
        return QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )
    except ImportError:
        return None
