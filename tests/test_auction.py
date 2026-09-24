"""
Unit Tests for Decentralized Auction Protocol and Battery-Aware Task Allocation.
"""

import os
import sys
import unittest

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(project_root, 'src', 'amr_fleet'))

from amr_fleet.utils import Vector2D
from amr_fleet.task_allocator import TaskAllocator, WarehouseTask
from amr_fleet.battery_monitor import BatteryMonitor


class TestAuctionTaskAllocation(unittest.TestCase):
    def setUp(self):
        self.alloc_r1 = TaskAllocator("robot_1", nominal_speed=0.6)
        self.alloc_r2 = TaskAllocator("robot_2", nominal_speed=0.6)
        self.alloc_r3 = TaskAllocator("robot_3", nominal_speed=0.6)

    def test_bid_cost_travel_distance(self):
        """Test closer robot submits lower bid cost."""
        task = WarehouseTask(
            task_id="task_101",
            pickup_location=Vector2D(5.0, 5.0),
            dropoff_location=Vector2D(5.0, 0.0),
            urgency=0.5
        )

        # Robot 1 is at (4.0, 5.0) -> dist to pickup = 1m
        bid_r1 = self.alloc_r1.compute_bid(task, current_pos=Vector2D(4.0, 5.0), battery_percentage=90.0)
        # Robot 2 is at (-5.0, -5.0) -> dist to pickup = ~14m
        bid_r2 = self.alloc_r2.compute_bid(task, current_pos=Vector2D(-5.0, -5.0), battery_percentage=90.0)

        self.assertTrue(bid_r1.eligible)
        self.assertTrue(bid_r2.eligible)
        self.assertLess(bid_r1.bid_cost, bid_r2.bid_cost, "Closer robot should submit a lower bid")

    def test_battery_exclusion_threshold(self):
        """Test robot below 20% battery is excluded from normal task bidding."""
        normal_task = WarehouseTask(
            task_id="task_normal",
            pickup_location=Vector2D(0.0, 0.0),
            dropoff_location=Vector2D(2.0, 2.0),
            urgency=0.4
        )

        bid_low_bat = self.alloc_r1.compute_bid(normal_task, current_pos=Vector2D(0.0, 0.0), battery_percentage=18.5)
        self.assertFalse(bid_low_bat.eligible, "Robot with 18.5% battery must be excluded from non-urgent tasks")
        self.assertEqual(bid_low_bat.bid_cost, float('inf'))

        # If task is URGENT (>= 0.9), robot with >15% can participate
        urgent_task = WarehouseTask(
            task_id="task_urgent",
            pickup_location=Vector2D(0.0, 0.0),
            dropoff_location=Vector2D(2.0, 2.0),
            urgency=0.95
        )
        bid_urgent = self.alloc_r1.compute_bid(urgent_task, current_pos=Vector2D(0.0, 0.0), battery_percentage=18.5)
        self.assertTrue(bid_urgent.eligible, "Urgent task should allow emergency participation down to 15%")

    def test_auctioneer_award_to_lowest_bidder(self):
        """Test auctioneer collects bids and awards task to lowest cost bidder."""
        task = WarehouseTask(
            task_id="task_202",
            pickup_location=Vector2D(0.0, 0.0),
            dropoff_location=Vector2D(4.0, 0.0)
        )

        # Robot 1 acts as auctioneer
        self.alloc_r1.start_auction(task, current_time=100.0)

        bid_r1 = self.alloc_r1.compute_bid(task, Vector2D(5.0, 0.0), battery_percentage=80.0)
        bid_r2 = self.alloc_r2.compute_bid(task, Vector2D(1.0, 0.0), battery_percentage=85.0)  # Closer -> lower cost

        self.alloc_r1.receive_bid(bid_r1)
        self.alloc_r1.receive_bid(bid_r2)

        # Before timeout (0.5s) -> None
        res_early = self.alloc_r1.resolve_auction("task_202", current_time=100.2)
        self.assertIsNone(res_early)

        # After timeout -> winner is robot_2
        res_winner = self.alloc_r1.resolve_auction("task_202", current_time=100.6)
        self.assertIsNotNone(res_winner)
        winner_id, cost = res_winner
        self.assertEqual(winner_id, "robot_2")

    def test_critical_battery_dock_routing(self):
        """Test battery monitor triggers autonomous dock routing below 15%."""
        bm = BatteryMonitor("robot_1", initial_percentage=14.5)
        self.assertTrue(bm.requires_charging())

        station = bm.get_nearest_charging_station(Vector2D(0.0, 0.0))
        self.assertIsNotNone(station)
        self.assertTrue(station.is_occupied)
        self.assertEqual(station.occupied_by, "robot_1")

    def test_blocked_aisle_re_auction(self):
        """Test cancelling task and returning it to auction pool on aisle blockage."""
        task = WarehouseTask("task_blocked", Vector2D(0.0, 0.0), Vector2D(5.0, 5.0))
        self.alloc_r1.current_task = task

        re_task = self.alloc_r1.handle_blocked_aisle_re_auction("task_blocked")
        self.assertIsNotNone(re_task)
        self.assertEqual(re_task.status, "PENDING")
        self.assertIsNone(self.alloc_r1.current_task)


if __name__ == '__main__':
    unittest.main()
