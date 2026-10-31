"""
Lightweight ROS 2 Mocking Framework for Headless and Unit Testing.
Provides mock nodes, publishers, subscribers, timers, message buses,
and standard ROS 2 message dataclasses.
"""

from __future__ import annotations
import sys
import time
from typing import Dict, List, Any, Callable, Optional


class MockParameter:
    def __init__(self, value: Any):
        self._value = value

    def get_parameter_value(self) -> MockParameter:
        return self

    @property
    def string_value(self) -> str:
        return str(self._value)

    @property
    def double_value(self) -> float:
        return float(self._value)

    @property
    def integer_value(self) -> int:
        return int(self._value)

    @property
    def bool_value(self) -> bool:
        return bool(self._value)


class MockTime:
    def __init__(self, sec: int = 0, nanosec: int = 0):
        self.sec = sec
        self.nanosec = nanosec


class MockClock:
    def now(self):
        class _Msg:
            def to_msg(self):
                t = time.time()
                sec = int(t)
                nanosec = int((t - sec) * 1e9)
                return MockTime(sec, nanosec)
        return _Msg()


class MockLogger:
    def __init__(self, name: str = "mock"):
        self.name = name
        self.logs: List[Tuple[str, str]] = []

    def info(self, msg: str) -> None:
        self.logs.append(("INFO", str(msg)))

    def warn(self, msg: str) -> None:
        self.logs.append(("WARN", str(msg)))

    def error(self, msg: str) -> None:
        self.logs.append(("ERROR", str(msg)))

    def debug(self, msg: str) -> None:
        self.logs.append(("DEBUG", str(msg)))


class MockMessageBus:
    """In-memory topic bus for dispatching messages between mock ROS nodes."""
    def __init__(self):
        self.subscribers: Dict[str, List[Callable[[Any], None]]] = {}
        self.history: Dict[str, List[Any]] = {}

    def register_sub(self, topic: str, callback: Callable[[Any], None]) -> None:
        self.subscribers.setdefault(topic, []).append(callback)
        # Transient-local replay if available
        if topic in self.history and self.history[topic]:
            callback(self.history[topic][-1])

    def dispatch(self, topic: str, msg: Any) -> None:
        self.history.setdefault(topic, []).append(msg)
        for sub_topic, callbacks in self.subscribers.items():
            if sub_topic == topic:
                for cb in callbacks:
                    cb(msg)

    def clear(self):
        self.subscribers.clear()
        self.history.clear()


class MockPublisher:
    def __init__(self, topic: str, msg_type: Any, qos: Any = 10, bus: Optional[MockMessageBus] = None):
        self.topic = topic
        self.msg_type = msg_type
        self.qos = qos
        self.bus = bus
        self.published_messages: List[Any] = []

    def publish(self, msg: Any) -> None:
        self.published_messages.append(msg)
        if self.bus is not None:
            self.bus.dispatch(self.topic, msg)


class MockSubscription:
    def __init__(self, topic: str, msg_type: Any, callback: Callable[[Any], None], qos: Any = 10):
        self.topic = topic
        self.msg_type = msg_type
        self.callback = callback
        self.qos = qos


class MockTimer:
    def __init__(self, period: float, callback: Callable[[], None]):
        self.period = period
        self.callback = callback
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True

    def trigger(self) -> None:
        if not self.cancelled:
            self.callback()


class MockFuture:
    def __init__(self, result: Any = None, done: bool = True):
        self._result = result
        self._done = done

    def done(self) -> bool:
        return self._done

    def result(self) -> Any:
        return self._result


class MockClient:
    def __init__(self, srv_type: Any, srv_name: str):
        self.srv_type = srv_type
        self.srv_name = srv_name
        self.should_fail = False
        self.service_available = True

    def wait_for_service(self, timeout_sec: float = 1.0) -> bool:
        return self.service_available

    def call_async(self, request: Any) -> MockFuture:
        if self.should_fail:
            class _ResFail:
                success = False
                status_message = "Mock entity operation failed"
            return MockFuture(_ResFail())
        class _ResSuccess:
            success = True
            status_message = "Mock entity operation succeeded"
        return MockFuture(_ResSuccess())


class MockNode:
    """Mock ROS 2 Node providing standard rclpy.node.Node methods."""
    def __init__(self, name: str = "mock_node", bus: Optional[MockMessageBus] = None):
        self._name = name
        self.parameters: Dict[str, Any] = {}
        self.publishers: List[MockPublisher] = []
        self.subscriptions: List[MockSubscription] = []
        self.timers: List[MockTimer] = []
        self.clients: Dict[str, MockClient] = {}
        self.services: Dict[str, Any] = {}
        self.bus = bus or MockMessageBus()
        self._logger = MockLogger(name)
        self._clock = MockClock()

    def create_client(self, srv_type: Any, srv_name: str) -> MockClient:
        client = MockClient(srv_type, srv_name)
        self.clients[srv_name] = client
        return client

    def create_service(self, srv_type: Any, srv_name: str, callback: Callable[[Any, Any], Any]) -> Any:
        self.services[srv_name] = callback
        return callback

    def get_name(self) -> str:
        return self._name

    def declare_parameter(self, name: str, default: Any = None) -> Any:
        if name not in self.parameters:
            self.parameters[name] = default
        return self.parameters[name]

    def get_parameter(self, name: str) -> MockParameter:
        return MockParameter(self.parameters.get(name, ""))

    def set_parameter(self, name: str, val: Any) -> None:
        self.parameters[name] = val

    def create_publisher(self, msg_type: Any, topic: str, qos: Any = 10) -> MockPublisher:
        pub = MockPublisher(topic, msg_type, qos, bus=self.bus)
        self.publishers.append(pub)
        return pub

    def create_subscription(self, msg_type: Any, topic: str, callback: Callable[[Any], None], qos: Any = 10) -> MockSubscription:
        sub = MockSubscription(topic, msg_type, callback, qos)
        self.subscriptions.append(sub)
        self.bus.register_sub(topic, callback)
        return sub

    def create_timer(self, period: float, callback: Callable[[], None]) -> MockTimer:
        timer = MockTimer(period, callback)
        self.timers.append(timer)
        return timer

    def get_logger(self) -> MockLogger:
        return self._logger

    def get_clock(self) -> MockClock:
        return self._clock

    def destroy_node(self) -> None:
        pass


# ------------------------------------------------------------------------------
# Standard ROS 2 Message Classes
# ------------------------------------------------------------------------------

class Header:
    def __init__(self, frame_id: str = "map"):
        self.stamp = MockTime()
        self.frame_id = frame_id


class Float32:
    def __init__(self, data: float = 0.0):
        self.data = float(data)


class String:
    def __init__(self, data: str = ""):
        self.data = str(data)


class Vector3:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)


class Point:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)


class Quaternion:
    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, w: float = 1.0):
        self.x = float(x)
        self.y = float(y)
        self.z = float(z)
        self.w = float(w)


class Pose:
    def __init__(self):
        self.position = Point()
        self.orientation = Quaternion()


class PoseStamped:
    def __init__(self):
        self.header = Header()
        self.pose = Pose()


class Twist:
    def __init__(self):
        self.linear = Vector3()
        self.angular = Vector3()


class Odometry:
    def __init__(self):
        self.header = Header()
        class _PoseWithCov:
            def __init__(self):
                self.pose = Pose()
        class _TwistWithCov:
            def __init__(self):
                self.twist = Twist()
        self.pose = _PoseWithCov()
        self.twist = _TwistWithCov()


class Path:
    def __init__(self):
        self.header = Header()
        self.poses: List[PoseStamped] = []


class BatteryState:
    """sensor_msgs/msg/BatteryState mock."""
    def __init__(self, percentage: float = 1.0, voltage: float = 24.0, current: float = 0.0):
        self.header = Header()
        self.percentage = float(percentage)  # In ROS 2, percentage is 0.0 to 1.0
        self.voltage = float(voltage)
        self.current = float(current)


class OccupancyGrid:
    def __init__(self):
        self.header = Header()
        class _Info:
            def __init__(self):
                self.width = 100
                self.height = 100
                self.resolution = 0.2
                self.origin = Pose()
        self.info = _Info()
        self.data: List[int] = [0] * (100 * 100)


class SpawnEntity:
    class Request:
        def __init__(self):
            self.name = ""
            self.xml = ""
            self.robot_namespace = ""
            self.initial_pose = Pose()
            self.reference_frame = "world"


class DeleteEntity:
    class Request:
        def __init__(self):
            self.name = ""


class Empty:
    class Request:
        pass


class ReliabilityPolicy:
    SYSTEM_DEFAULT = 0
    RELIABLE = 1
    BEST_EFFORT = 2


class DurabilityPolicy:
    SYSTEM_DEFAULT = 0
    TRANSIENT_LOCAL = 1
    VOLATILE = 2


class HistoryPolicy:
    SYSTEM_DEFAULT = 0
    KEEP_LAST = 1
    KEEP_ALL = 2


class Duration:
    def __init__(self, seconds: int = 0, nanoseconds: int = 0):
        self.seconds = seconds
        self.nanoseconds = nanoseconds

    @property
    def nanoseconds_value(self) -> int:
        return self.seconds * 1_000_000_000 + self.nanoseconds


class QoSProfile:
    def __init__(
        self,
        history: Any = HistoryPolicy.KEEP_LAST,
        depth: int = 10,
        reliability: Any = ReliabilityPolicy.RELIABLE,
        durability: Any = DurabilityPolicy.VOLATILE,
        deadline: Optional[Duration] = None,
        lifespan: Optional[Duration] = None,
        liveliness: Any = None,
        liveliness_lease_duration: Optional[Duration] = None,
        avoid_ros_namespace_conventions: bool = False
    ):
        self.history = history
        self.depth = depth
        self.reliability = reliability
        self.durability = durability
        self.deadline = deadline
        self.lifespan = lifespan
        self.liveliness = liveliness
        self.liveliness_lease_duration = liveliness_lease_duration
        self.avoid_ros_namespace_conventions = avoid_ros_namespace_conventions

    def is_compatible_with(self, requested: QoSProfile) -> bool:
        """DDS QoS compatibility check between offered and requested policies."""
        if self.reliability == ReliabilityPolicy.BEST_EFFORT and requested.reliability == ReliabilityPolicy.RELIABLE:
            return False
        if self.durability == DurabilityPolicy.VOLATILE and requested.durability == DurabilityPolicy.TRANSIENT_LOCAL:
            return False
        return True


def install_ros_mocks():
    """Install mock ROS 2 packages into sys.modules if real ROS 2 is not present."""
    import types
    import os

    def _ensure_module(mod_name):
        if mod_name not in sys.modules:
            mod = types.ModuleType(mod_name)
            sys.modules[mod_name] = mod
        return sys.modules[mod_name]

    rclpy = _ensure_module("rclpy")
    rclpy.init = lambda args=None: None
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: None
    rclpy.spin_once = lambda node, timeout_sec=None: None
    rclpy.spin_until_future_complete = lambda node, future, timeout_sec=None: None

    rclpy_node = _ensure_module("rclpy.node")
    rclpy_node.Node = MockNode

    rclpy_qos = _ensure_module("rclpy.qos")
    rclpy_qos.QoSProfile = QoSProfile
    rclpy_qos.ReliabilityPolicy = ReliabilityPolicy
    rclpy_qos.DurabilityPolicy = DurabilityPolicy
    rclpy_qos.HistoryPolicy = HistoryPolicy

    rclpy_dur = _ensure_module("rclpy.duration")
    rclpy_dur.Duration = Duration

    std_msgs = _ensure_module("std_msgs")
    std_msgs_msg = _ensure_module("std_msgs.msg")
    std_msgs_msg.Float32 = Float32
    std_msgs_msg.String = String
    std_msgs_msg.Header = Header

    std_srvs = _ensure_module("std_srvs")
    std_srvs_srv = _ensure_module("std_srvs.srv")
    std_srvs_srv.Empty = Empty

    gazebo_msgs = _ensure_module("gazebo_msgs")
    gazebo_msgs_srv = _ensure_module("gazebo_msgs.srv")
    gazebo_msgs_srv.SpawnEntity = SpawnEntity
    gazebo_msgs_srv.DeleteEntity = DeleteEntity

    sensor_msgs = _ensure_module("sensor_msgs")
    sensor_msgs_msg = _ensure_module("sensor_msgs.msg")
    sensor_msgs_msg.BatteryState = BatteryState

    geometry_msgs = _ensure_module("geometry_msgs")
    geometry_msgs_msg = _ensure_module("geometry_msgs.msg")
    geometry_msgs_msg.Point = Point
    geometry_msgs_msg.Pose = Pose
    geometry_msgs_msg.PoseStamped = PoseStamped
    geometry_msgs_msg.Twist = Twist
    geometry_msgs_msg.Vector3 = Vector3
    geometry_msgs_msg.Quaternion = Quaternion

    nav_msgs = _ensure_module("nav_msgs")
    nav_msgs_msg = _ensure_module("nav_msgs.msg")
    nav_msgs_msg.Path = Path
    nav_msgs_msg.Odometry = Odometry
    nav_msgs_msg.OccupancyGrid = OccupancyGrid

    ament_index_python = _ensure_module("ament_index_python")
    ament_index_python_pkgs = _ensure_module("ament_index_python.packages")
    ament_index_python_pkgs.get_package_share_directory = lambda pkg: os.path.abspath(f"src/{pkg}")

