"""
Battery Monitor and Autonomous Recharging Node.
Models energy consumption based on kinetic motion and idle draw.
Enforces battery-aware coordination rules:
- Below 20%: Excluded from normal auctions.
- Below 15%: Autonomously routes to nearest charging station and broadcasts ChargingIntent.
"""

from __future__ import annotations
import math
import time
from typing import List, Optional
from dataclasses import dataclass, field

from amr_fleet.utils import Vector2D


@dataclass
class ChargingStation:
    station_id: str
    location: Vector2D
    is_occupied: bool = False
    occupied_by: Optional[str] = None


class BatteryMonitor:
    """
    Physical battery simulation and threshold governance engine.
    """
    def __init__(
        self,
        robot_id: str,
        initial_percentage: float = 100.0,
        idle_discharge_rate: float = 0.01,  # % per second
        kinetic_discharge_factor: float = 0.10,  # % per m/s traveled
        recharge_rate: float = 2.0  # % per second when docked
    ):
        self.robot_id = robot_id
        self.battery_pct = max(0.0, min(100.0, initial_percentage))
        self.idle_discharge_rate = idle_discharge_rate
        self.kinetic_discharge_factor = kinetic_discharge_factor
        self.recharge_rate = recharge_rate

        self.is_charging = False
        self.assigned_station: Optional[ChargingStation] = None

        # Predefined warehouse charging station docks
        self.charging_stations = [
            ChargingStation("station_north_west", Vector2D(-12.0, 8.0)),
            ChargingStation("station_south_west", Vector2D(-12.0, -8.0)),
            ChargingStation("station_east", Vector2D(12.0, 0.0)),
        ]

    def update_state(self, current_pos: Vector2D, linear_vel: float, dt: float) -> float:
        """
        Updates battery state for time delta dt. Returns current battery percentage.
        """
        if self.is_charging:
            self.battery_pct = min(100.0, self.battery_pct + self.recharge_rate * dt)
            if self.battery_pct >= 95.0:
                self.is_charging = False
                if self.assigned_station:
                    self.assigned_station.is_occupied = False
                    self.assigned_station.occupied_by = None
                    self.assigned_station = None
            return self.battery_pct

        # Check if arrived at assigned charging station
        if self.assigned_station and current_pos.distance_to(self.assigned_station.location) < 0.5:
            self.is_charging = True
            return self.battery_pct

        # Discharge model: Idle + Kinetic draw
        discharge = (self.idle_discharge_rate + self.kinetic_discharge_factor * abs(linear_vel)) * dt
        self.battery_pct = max(0.0, self.battery_pct - discharge)
        return self.battery_pct

    def is_auction_eligible(self, is_task_urgent: bool = False) -> bool:
        """Robots below 20% excluded unless urgent task."""
        if is_task_urgent:
            return self.battery_pct > 15.0
        return self.battery_pct >= 20.0

    def requires_charging(self) -> bool:
        """Below 15% requires autonomous route to charging dock."""
        return self.battery_pct < 15.0 and not self.is_charging

    def get_nearest_charging_station(self, current_pos: Vector2D) -> Optional[ChargingStation]:
        """Finds closest available charging station."""
        available = [st for st in self.charging_stations if not st.is_occupied or st.occupied_by == self.robot_id]
        if not available:
            # All occupied; pick closest regardless
            available = self.charging_stations

        station = min(available, key=lambda s: current_pos.distance_to(s.location))
        self.assigned_station = station
        station.is_occupied = True
        station.occupied_by = self.robot_id
        return station


# ==============================================================================
# ROS 2 Battery Monitor Node
# ==============================================================================

def main(args=None):
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import Float32, String
        from geometry_msgs.msg import PoseStamped, Point
    except ImportError:
        print("[BatteryMonitor] rclpy not detected; run in pure Python or ROS 2 container.")
        return

    rclpy.init(args=args)

    class BatteryMonitorNode(Node):
        def __init__(self):
            super().__init__('battery_monitor')
            self.declare_parameter('robot_id', 'robot_1')
            self.declare_parameter('initial_battery', 100.0)

            self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
            initial_bat = self.get_parameter('initial_battery').get_parameter_value().double_value

            self.monitor = BatteryMonitor(self.robot_id, initial_percentage=initial_bat)
            self.current_pos = Vector2D(0.0, 0.0)
            self.current_vel = 0.0

            # Publishers & Subscribers
            self.battery_pub = self.create_publisher(Float32, f'/fleet/{self.robot_id}/battery', 10)
            self.intent_pub = self.create_publisher(String, f'/fleet/{self.robot_id}/charging_intent', 10)
            self.goal_pub = self.create_publisher(Point, f'/fleet/{self.robot_id}/goal', 10)

            self.create_subscription(PoseStamped, f'/{self.robot_id}/pose', self.pose_callback, 10)
            self.timer = self.create_timer(0.5, self.monitor_tick)  # 2 Hz
            self.get_logger().info(f"BatteryMonitor started for {self.robot_id} ({initial_bat}%)")

        def pose_callback(self, msg: PoseStamped):
            new_pos = Vector2D(msg.pose.position.x, msg.pose.position.y)
            self.current_vel = (new_pos - self.current_pos).norm() / 0.1
            self.current_pos = new_pos

        def monitor_tick(self):
            pct = self.monitor.update_state(self.current_pos, self.current_vel, dt=0.5)
            msg = Float32()
            msg.data = float(pct)
            self.battery_pub.publish(msg)

            if self.monitor.requires_charging():
                station = self.monitor.get_nearest_charging_station(self.current_pos)
                if station:
                    self.get_logger().warn(
                        f"CRITICAL BATTERY {pct:.1f}%! Routing {self.robot_id} to {station.station_id}"
                    )
                    intent_msg = String()
                    intent_msg.data = f"CHARGING_INTENT:{self.robot_id}:{station.station_id}"
                    self.intent_pub.publish(intent_msg)

                    goal_msg = Point()
                    goal_msg.x = station.location.x
                    goal_msg.y = station.location.y
                    self.goal_pub.publish(goal_msg)

    node = BatteryMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
