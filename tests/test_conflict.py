"""
Unit Tests for Conflict Resolution, Priority Negotiation, and Deadlock Handling.
"""

import os
import sys
import unittest
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(project_root, 'src', 'amr_fleet'))

from amr_fleet.utils import Vector2D, RobotIntentData
from amr_fleet.conflict_resolver import ConflictResolver


class TestConflictResolver(unittest.TestCase):
    def setUp(self):
        self.resolver_r1 = ConflictResolver("robot_1", safety_distance=0.8)
        self.resolver_r2 = ConflictResolver("robot_2", safety_distance=0.8)

    def test_composite_priority_scoring(self):
        """Test composite priority correctly balances distance, urgency, and battery."""
        # Robot A is 1m from goal, high urgency, 50% battery
        p_close = self.resolver_r1.compute_composite_priority(distance_to_goal=1.0, task_urgency=0.9, battery_percentage=50.0)
        # Robot B is 10m from goal, same urgency and battery
        p_far = self.resolver_r1.compute_composite_priority(distance_to_goal=10.0, task_urgency=0.9, battery_percentage=50.0)
        self.assertGreater(p_close, p_far, "Closer robot should have higher priority")

        # Robot C has critically low battery (10%) vs healthy (90%)
        p_low_bat = self.resolver_r1.compute_composite_priority(distance_to_goal=5.0, task_urgency=0.5, battery_percentage=10.0)
        p_high_bat = self.resolver_r1.compute_composite_priority(distance_to_goal=5.0, task_urgency=0.5, battery_percentage=90.0)
        self.assertGreater(p_low_bat, p_high_bat, "Low battery robot should receive priority bonus to avoid stranding")

    def test_priority_negotiation_and_tie_breaking(self):
        """Test negotiation actions and deterministic tie-breaking."""
        # R1 higher priority -> R1 PROCEED, R2 YIELD
        r1_act, r2_act = self.resolver_r1.negotiate_priority("robot_2", my_priority=0.85, peer_priority=0.60)
        self.assertEqual(r1_act, "PROCEED")
        self.assertEqual(r2_act, "YIELD")

        # Equal priority -> Deterministic tie-break by ID ("robot_1" < "robot_2")
        r1_tie, r2_tie = self.resolver_r1.negotiate_priority("robot_2", my_priority=0.70, peer_priority=0.70)
        self.assertEqual(r1_tie, "PROCEED")
        self.assertEqual(r2_tie, "YIELD")

    def test_edge_and_vertex_conflict_detection(self):
        """Test detection of head-on trajectory conflicts."""
        my_path = [Vector2D(-2.0, 0.0), Vector2D(0.0, 0.0), Vector2D(2.0, 0.0)]
        peer_path = [Vector2D(2.0, 0.0), Vector2D(0.0, 0.0), Vector2D(-2.0, 0.0)]

        peer_intent = {
            "robot_2": RobotIntentData(robot_id="robot_2", planned_path=peer_path)
        }

        conflicts = self.resolver_r1.detect_trajectory_conflicts(my_path, peer_intent)
        self.assertGreater(len(conflicts), 0, "Should detect head-on edge/vertex conflict")
        peer_id, c_type, loc = conflicts[0]
        self.assertEqual(peer_id, "robot_2")
        self.assertIn(c_type, ["EDGE", "VERTEX"])

    def test_deadlock_detection_timeout(self):
        """Test deadlock triggers after stationary stall > 3.0 seconds."""
        base_time = 100.0
        self.resolver_r1.last_moving_time = base_time

        # At t = 101s (1s stalled): no deadlock
        is_dl = self.resolver_r1.update_deadlock_state(
            current_speed=0.0, dist_to_goal=5.0, nearby_peers_stationary=True, current_time=base_time + 1.0
        )
        self.assertFalse(is_dl)

        # At t = 103.5s (>3.0s stalled): deadlock detected!
        is_dl = self.resolver_r1.update_deadlock_state(
            current_speed=0.0, dist_to_goal=5.0, nearby_peers_stationary=True, current_time=base_time + 3.5
        )
        self.assertTrue(is_dl)

    def test_corridor_token_reservation(self):
        """Test token acquisition and mutual exclusion."""
        corridor_id = "corridor_main"

        # Robot 1 requests token -> Granted
        granted_1 = self.resolver_r1.request_corridor_token(corridor_id)
        self.assertTrue(granted_1)

        # Robot 2 requests same token while held by Robot 1 -> Denied / Wait
        self.resolver_r2.corridor_tokens = self.resolver_r1.corridor_tokens
        granted_2 = self.resolver_r2.request_corridor_token(corridor_id)
        self.assertFalse(granted_2)

        # Robot 1 releases token -> Robot 2 can acquire
        self.resolver_r1.release_corridor_token(corridor_id)
        self.resolver_r2.corridor_tokens = self.resolver_r1.corridor_tokens
        granted_2_retry = self.resolver_r2.request_corridor_token(corridor_id)
        self.assertTrue(granted_2_retry)


if __name__ == '__main__':
    unittest.main()
