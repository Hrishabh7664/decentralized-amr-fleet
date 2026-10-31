#!/usr/bin/env python3
"""
Battery Monitor and Autonomous Recharging Node.
Models energy consumption based on kinetic motion and idle draw.
Enforces battery-aware coordination rules:
- Below 20%: Excluded from normal auctions.
- Below 15%: Autonomously routes to nearest charging station and broadcasts ChargingIntent.
"""

from __future__ import annotations
import time
from typing import List, Optional, Any
from dataclasses import dataclass

from amr_fleet.utils import Vector2D


@dataclass
class ChargingStation:
    """Designated AMR charging dock with occupancy state tracking."""
    station_id: str
    location: Vector2D
    is_occupied: bool = False
    occupied_by: Optional[str] = None
    last_heartbeat: float = 0.0


class BatteryMonitor:
    """Physical battery simulation and threshold governance engine."""
    def __init__(
        self,
        robot_id: str,
        initial_percentage: float = 100.0,
        idle_discharge_rate: float = 0.01,
        kinetic_discharge_factor: float = 0.10,
        recharge_rate: float = 2.0
    ) -> None:
        """Initialize battery state, discharge factors, and dock registry."""
        self.robot_id = robot_id
        self.battery_pct = max(0.0, min(100.0, initial_percentage))
        self.idle_discharge_rate = idle_discharge_rate
        self.kinetic_discharge_factor = kinetic_discharge_factor
        self.recharge_rate = recharge_rate

        self.is_charging = False
        self.assigned_station: Optional[ChargingStation] = None
        self.start_charging_threshold: float = 90.0
        self.stop_charging_threshold: float = 95.0

        self.charging_stations = [
            ChargingStation("station_north_west", Vector2D(-12.0, 8.0)),
            ChargingStation("station_south_west", Vector2D(-12.0, -8.0)),
            ChargingStation("station_east", Vector2D(12.0, 0.0)),
        ]

    def update_state(self, current_pos: Vector2D, linear_vel: float, dt: float) -> float:
        """Updates battery state for time delta dt. Returns current battery percentage."""
        if self.is_charging:
            self.battery_pct = min(100.0, self.battery_pct + self.recharge_rate * dt)
            if self.battery_pct >= self.stop_charging_threshold:
                self.is_charging = False
                if self.assigned_station:
                    self.assigned_station.is_occupied = False
                    self.assigned_station.occupied_by = None
                    self.assigned_station = None
            return self.battery_pct

        if self.assigned_station and current_pos.distance_to(self.assigned_station.location) < 0.5:
            if self.battery_pct < self.start_charging_threshold:
                self.is_charging = True
                return self.battery_pct

        discharge = (self.idle_discharge_rate + self.kinetic_discharge_factor * abs(linear_vel)) * dt
        self.battery_pct = max(0.0, self.battery_pct - discharge)
        return self.battery_pct

    def is_auction_eligible(self, is_task_urgent: bool = False) -> bool:
        """Check if robot has enough battery to participate in task auctions."""
        if is_task_urgent:
            return self.battery_pct > 15.0
        return self.battery_pct >= 20.0

    def requires_charging(self) -> bool:
        """Check if robot is below critical 15% threshold and requires immediate docking."""
        return self.battery_pct < 15.0 and not self.is_charging

    def get_nearest_charging_station(self, current_pos: Vector2D) -> Optional[ChargingStation]:
        """Find the nearest available charging station dock and assign it."""
        available = [st for st in self.charging_stations if not st.is_occupied or st.occupied_by == self.robot_id]
        if not available:
            available = self.charging_stations
        station = min(available, key=lambda s: current_pos.distance_to(s.location))
        self.assign_station(station)
        return station

    def assign_station(self, station: ChargingStation) -> None:
        """Assign station to this robot and mark initial claim."""
        self.assigned_station = station
        station.is_occupied = True
        station.occupied_by = self.robot_id
        station.last_heartbeat = time.time()

    def evict_station(self) -> None:
        """Unassign and clear current station claim."""
        if self.assigned_station:
            if self.assigned_station.occupied_by == self.robot_id:
                self.assigned_station.is_occupied = False
                self.assigned_station.occupied_by = None
                self.assigned_station.last_heartbeat = 0.0
            self.assigned_station = None

    def prune_stale_leases(self, timeout_s: float = 6.0, current_time: Optional[float] = None) -> List[str]:
        """Evict occupied charging stations if peer missed heartbeats beyond timeout."""
        now = current_time if current_time is not None else time.time()
        evicted = []
        for st in self.charging_stations:
            if st.is_occupied and st.occupied_by != self.robot_id:
                if st.last_heartbeat > 0.0 and (now - st.last_heartbeat > timeout_s):
                    st.is_occupied = False
                    st.occupied_by = None
                    st.last_heartbeat = 0.0
                    evicted.append(st.station_id)
        return evicted


class BatteryMonitorNode:
    """ROS 2 Node wrapper for monitoring battery levels and dispatching recharge goals."""
    def __init__(self, node: Any) -> None:
        """Initialize battery monitor ROS 2 node."""
        self.node = node
        self.node.declare_parameter('robot_id', 'robot_1')
        self.node.declare_parameter('initial_battery', 100.0)

        self.robot_id = self.node.get_parameter('robot_id').get_parameter_value().string_value
        initial_bat = self.node.get_parameter('initial_battery').get_parameter_value().double_value

        self.monitor = BatteryMonitor(self.robot_id, initial_percentage=initial_bat)
        self.current_pos = Vector2D(0.0, 0.0)
        self.current_vel = 0.0
        self.last_pose_time: Optional[float] = None
        self.my_claim_timestamp: float = 0.0
        self.last_heartbeat_time: float = 0.0

        from std_msgs.msg import Float32, String
        from geometry_msgs.msg import PoseStamped, Point

        self.battery_pub = self.node.create_publisher(Float32, f'/fleet/{self.robot_id}/battery', 10)
        self.intent_pub = self.node.create_publisher(String, f'/fleet/{self.robot_id}/charging_intent', 10)
        self.goal_pub = self.node.create_publisher(Point, f'/fleet/{self.robot_id}/goal', 10)
        self.station_pub = self.node.create_publisher(String, '/fleet/charging_stations', 10)

        self.node.create_subscription(PoseStamped, f'/{self.robot_id}/pose', self.pose_callback, 10)
        self.node.create_subscription(String, '/fleet/charging_stations', self.station_callback, 10)

        self.timer = self.node.create_timer(0.5, self.monitor_tick)
        self.node.get_logger().info(f"BatteryMonitor started for {self.robot_id} ({initial_bat}%)")

    def pose_callback(self, msg: Any) -> None:
        """Update robot position and estimate current speed."""
        new_pos = Vector2D(msg.pose.position.x, msg.pose.position.y)
        now = time.time()
        if self.last_pose_time is None:
            self.last_pose_time = now
            self.current_pos = new_pos
            self.current_vel = 0.0
            return
        dt = max(0.01, now - self.last_pose_time)
        self.last_pose_time = now
        measured_vel = (new_pos - self.current_pos).norm() / dt
        self.current_vel = min(1.5, measured_vel)
        self.current_pos = new_pos

    def station_callback(self, msg: Any) -> None:
        """Track peer claims, heartbeats, and releases of charging stations."""
        from std_msgs.msg import String
        from geometry_msgs.msg import Point
        try:
            parts = str(msg.data).split(':')
            action, r_id, s_id = parts[0], parts[1], parts[2]
            ts = float(parts[3]) if len(parts) > 3 else 0.0
            if r_id != self.robot_id:
                for st in self.monitor.charging_stations:
                    if st.station_id == s_id:
                        if action == "HEARTBEAT":
                            st.is_occupied = True
                            st.occupied_by = r_id
                            st.last_heartbeat = time.time()
                        elif action == "CLAIM":
                            # Check contention: are we also claiming or assigned to this station?
                            if self.monitor.assigned_station and self.monitor.assigned_station.station_id == s_id:
                                # Contention arbitration: earlier timestamp wins; if tied, lower robot_id wins
                                peer_wins = False
                                if ts < self.my_claim_timestamp:
                                    peer_wins = True
                                elif ts == self.my_claim_timestamp and r_id < self.robot_id:
                                    peer_wins = True

                                if peer_wins:
                                    # Lost contention: back off and select alternative
                                    st.is_occupied = True
                                    st.occupied_by = r_id
                                    st.last_heartbeat = time.time()
                                    self.monitor.assigned_station = None
                                    alt = self.monitor.get_nearest_charging_station(self.current_pos)
                                    if alt and alt.station_id != s_id:
                                        self.my_claim_timestamp = time.time()
                                        self.monitor.assign_station(alt)
                                        claim_msg = String()
                                        claim_msg.data = f"CLAIM:{self.robot_id}:{alt.station_id}:{self.my_claim_timestamp:.2f}"
                                        self.station_pub.publish(claim_msg)
                                        intent_msg = String()
                                        intent_msg.data = f"CHARGING_INTENT:{self.robot_id}:{alt.station_id}"
                                        self.intent_pub.publish(intent_msg)
                                        goal_msg = Point()
                                        goal_msg.x = alt.location.x
                                        goal_msg.y = alt.location.y
                                        self.goal_pub.publish(goal_msg)
                            else:
                                st.is_occupied = True
                                st.occupied_by = r_id
                                st.last_heartbeat = time.time()
                        elif action == "RELEASE" and st.occupied_by == r_id:
                            st.is_occupied = False
                            st.occupied_by = None
                            st.last_heartbeat = 0.0
        except Exception as e:
            self.node.get_logger().debug(f"Failed to parse charging station update: {e}")

    def monitor_tick(self) -> None:
        """Update battery state, prune stale leases, and broadcast heartbeats."""
        from std_msgs.msg import Float32, String
        from geometry_msgs.msg import Point

        # Prune dead robot dock leases
        self.monitor.prune_stale_leases(timeout_s=6.0)

        was_charging = self.monitor.is_charging
        pct = self.monitor.update_state(self.current_pos, self.current_vel, dt=0.5)

        if was_charging and not self.monitor.is_charging and self.monitor.assigned_station:
            rel_msg = String()
            rel_msg.data = f"RELEASE:{self.robot_id}:{self.monitor.assigned_station.station_id}"
            self.station_pub.publish(rel_msg)
            self.monitor.evict_station()

        msg = Float32()
        msg.data = float(pct)
        self.battery_pub.publish(msg)

        # Broadcast periodic heartbeat if holding a station
        now = time.time()
        if self.monitor.assigned_station and (now - self.last_heartbeat_time >= 2.0):
            hb_msg = String()
            hb_msg.data = f"HEARTBEAT:{self.robot_id}:{self.monitor.assigned_station.station_id}:{now:.2f}"
            self.station_pub.publish(hb_msg)
            self.last_heartbeat_time = now

        if self.monitor.requires_charging() and not self.monitor.assigned_station:
            station = self.monitor.get_nearest_charging_station(self.current_pos)
            if station:
                self.node.get_logger().warn(
                    f"CRITICAL BATTERY {pct:.1f}%! Routing {self.robot_id} to {station.station_id}"
                )
                self.my_claim_timestamp = now
                self.monitor.assign_station(station)

                claim_msg = String()
                claim_msg.data = f"CLAIM:{self.robot_id}:{station.station_id}:{self.my_claim_timestamp:.2f}"
                self.station_pub.publish(claim_msg)

                intent_msg = String()
                intent_msg.data = f"CHARGING_INTENT:{self.robot_id}:{station.station_id}"
                self.intent_pub.publish(intent_msg)

                goal_msg = Point()
                goal_msg.x = station.location.x
                goal_msg.y = station.location.y
                self.goal_pub.publish(goal_msg)


def main(args=None) -> None:
    """Entrypoint for battery monitor node."""
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        print("[BatteryMonitor] rclpy not detected; run in pure Python or ROS 2 container.")
        return

    rclpy.init(args=args)
    node = Node('battery_monitor')
    _ = BatteryMonitorNode(node)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
