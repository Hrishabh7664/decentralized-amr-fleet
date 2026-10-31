# NOTE: Native ROS 2 runtime (rclpy C-extensions / daemon) is unavailable in this Windows host environment; using MockNode/MockMessageBus harness for in-process node regression testing.
"""
Regression tests for Section C: BatteryMonitor race conditions and state machine stability.
C1: Atomic charging station claims and contention arbitration.
C2: Occupancy lease / heartbeat timeout on robot crash.
C3: Charging state machine hysteresis (90-95%) and debounce.
"""

import os
import sys
import time
import unittest

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(project_root, 'src', 'amr_fleet'))
sys.path.insert(0, current_dir)

from mock_ros import (
    install_ros_mocks,
    MockNode,
    MockMessageBus,
    Float32,
    String,
    Point,
    PoseStamped,
)

install_ros_mocks()

from amr_fleet.utils import Vector2D
from amr_fleet.perception.battery_monitor import (
    ChargingStation,
    BatteryMonitor,
    BatteryMonitorNode,
)


class TestSectionC(unittest.TestCase):
    def setUp(self):
        self.bus = MockMessageBus()

    def test_c3_charging_hysteresis_and_debounce(self):
        """
        [C3 Regression Test]
        Verify that charging state machine does not rapidly toggle around 95%:
        - Stops charging when battery >= 95.0%
        - Does NOT restart charging while hovering between 90.0% and 95.0%
        - Only restarts charging once battery drops below 90.0%
        """
        monitor = BatteryMonitor("robot_1", initial_percentage=10.0, recharge_rate=5.0)
        station_loc = monitor.charging_stations[0].location

        # Dock at station to begin charging
        monitor.assigned_station = monitor.charging_stations[0]
        monitor.update_state(current_pos=station_loc, linear_vel=0.0, dt=1.0)
        self.assertTrue(monitor.is_charging, "Should be charging at dock")

        # Charge up to 95.2% -> should stop charging
        monitor.battery_pct = 95.2
        monitor.update_state(current_pos=station_loc, linear_vel=0.0, dt=0.5)
        self.assertFalse(monitor.is_charging, "Should have stopped charging at >= 95%")

        # Hover trace around 95% while remaining docked at station
        monitor.assigned_station = monitor.charging_stations[0]
        toggle_count = 0
        was_charging = monitor.is_charging

        hover_trace = [94.8, 95.1, 94.9, 95.2, 94.7]
        for val in hover_trace:
            monitor.battery_pct = val
            monitor.assigned_station = monitor.charging_stations[0]
            monitor.update_state(current_pos=station_loc, linear_vel=0.0, dt=0.5)
            if monitor.is_charging != was_charging:
                toggle_count += 1
                was_charging = monitor.is_charging

        # In un-fixed code, charging restarts at 94.8% and toggles 4 times!
        # With hysteresis, it must stay False throughout the hover trace (0 toggles)
        self.assertEqual(toggle_count, 0, f"Charging toggled {toggle_count} times around 95%!")
        self.assertFalse(monitor.is_charging)

        # Drop below 90% (e.g. 89.5%) -> Now charging can resume
        monitor.battery_pct = 89.5
        monitor.assigned_station = monitor.charging_stations[0]
        monitor.update_state(current_pos=station_loc, linear_vel=0.0, dt=0.5)
        self.assertTrue(monitor.is_charging, "Charging should resume once dropped below 90.0%")

    def test_c1_atomic_station_claims_and_contention_arbitration(self):
        """
        [C1 Regression Test]
        Verify that simultaneous station claims are arbitrated:
        - When two robots attempt to claim the same dock, contention arbitration ensures
          only one robot commits to it.
        - The contested robot backs off and selects an alternate station.
        - Exactly one dock is assigned per robot with zero duplicate assignments.
        """
        node_1 = MockNode("battery_monitor_1", bus=self.bus)
        node_1.declare_parameter("robot_id", "robot_1")
        node_1.declare_parameter("initial_battery", 10.0)

        node_2 = MockNode("battery_monitor_2", bus=self.bus)
        node_2.declare_parameter("robot_id", "robot_2")
        node_2.declare_parameter("initial_battery", 10.0)

        bm_1 = BatteryMonitorNode(node_1)
        bm_2 = BatteryMonitorNode(node_2)

        # Both robots independently select the closest station_east before claims cross on the wire
        st_east_1 = [s for s in bm_1.monitor.charging_stations if s.station_id == "station_east"][0]
        st_east_2 = [s for s in bm_2.monitor.charging_stations if s.station_id == "station_east"][0]

        bm_1.monitor.assigned_station = st_east_1
        bm_1.my_claim_timestamp = 100.0
        bm_2.monitor.assigned_station = st_east_2
        bm_2.my_claim_timestamp = 100.2

        # Robot 1 sends CLAIM at t=100.0s (winner), Robot 2 sends CLAIM at t=100.2s (loser)
        claim_msg_1 = String(data="CLAIM:robot_1:station_east:100.0")
        claim_msg_2 = String(data="CLAIM:robot_2:station_east:100.2")

        # Deliver Robot 1's claim to Robot 2
        bm_2.station_callback(claim_msg_1)

        # In un-fixed code, Robot 2 never backs off; its assigned_station remains station_east!
        self.assertNotEqual(
            bm_2.monitor.assigned_station.station_id, "station_east",
            "Robot 2 failed to back off and re-select upon losing claim contention!"
        )
        self.assertIsNotNone(bm_2.monitor.assigned_station)

    def test_c2_dock_lease_heartbeat_timeout_on_crash(self):
        """
        [C2 Regression Test]
        Verify that dock occupancy requires active heartbeats:
        - When an occupying robot crashes (stops publishing heartbeats), other monitors
          evict the stale reservation after the lease timeout.
        """
        monitor = BatteryMonitor("robot_2")
        st = monitor.charging_stations[0]

        # Robot 1 claimed station at t = 100.0s
        base_time = 100.0
        st.is_occupied = True
        st.occupied_by = "robot_1"
        st.last_heartbeat = base_time

        # At t = 104s (within 6s lease): station remains occupied
        monitor.prune_stale_leases(timeout_s=6.0, current_time=base_time + 4.0)
        self.assertTrue(st.is_occupied)
        self.assertEqual(st.occupied_by, "robot_1")

        # At t = 108s (> 6s lease, robot_1 crashed): lease expires and station is evicted
        evicted = monitor.prune_stale_leases(timeout_s=6.0, current_time=base_time + 8.0)
        self.assertIn(st.station_id, evicted)
        self.assertFalse(st.is_occupied, "Station dock was not freed after missed heartbeats!")
        self.assertIsNone(st.occupied_by)


if __name__ == '__main__':
    unittest.main()
