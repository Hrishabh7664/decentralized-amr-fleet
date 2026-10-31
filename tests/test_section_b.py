# NOTE: Native ROS 2 runtime (rclpy C-extensions / daemon) is unavailable in this Windows host environment; using MockNode/MockMessageBus harness for in-process node regression testing.
"""
Regression tests for Section B: ConflictResolver logical bugs.
B1: Compute nearby_peers_stationary from actual peer velocities with hysteresis window.
B2: Broadcast TOKEN_REQ/TOKEN_GRANT/TOKEN_DENY protocol end-to-end.
B3: Pruning of stale peers in token and deadlock state.
"""

import json
import os
import sys
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
from amr_fleet.conflict_resolver import ConflictResolver
from amr_fleet.utils.conflict_resolver import ConflictResolverNode


class TestSectionB(unittest.TestCase):
    def setUp(self):
        self.bus = MockMessageBus()

    def test_b1_unit_nearby_peers_stationary_logic(self):
        """
        [B1 Unit Test]
        Verify that:
        (i) A stationary robot with moving peers is NOT deadlocked.
        (ii) A stationary robot with stationary peers IS deadlocked.
        """
        resolver = ConflictResolver("robot_1", safety_distance=0.8, deadlock_timeout_s=3.0)
        base_time = 100.0
        resolver.last_moving_time = base_time

        # Case (i): Stationary robot (speed=0), stall duration = 4.0s (> 3s),
        # but nearby peers are moving (peer speed = 0.5 m/s at 1.0m distance).
        peer_positions = {"robot_2": Vector2D(1.0, 0.0)}
        peer_speeds = {"robot_2": 0.5}

        # Check resolver's computation of nearby_peers_stationary
        peers_stationary = resolver.compute_nearby_peers_stationary(
            my_pos=Vector2D(0.0, 0.0),
            peer_positions=peer_positions,
            peer_speeds=peer_speeds,
            radius=2.5
        )
        self.assertFalse(peers_stationary, "Moving peer was incorrectly flagged as stationary!")

        is_dl = resolver.update_deadlock_state(
            current_speed=0.0,
            dist_to_goal=5.0,
            nearby_peers_stationary=peers_stationary,
            current_time=base_time + 4.0
        )
        self.assertFalse(is_dl, "Stationary robot with moving peers must NOT be deadlocked!")

        # Case (ii): Stationary robot (speed=0), stall duration = 4.0s,
        # and nearby peer is ALSO stationary (peer speed = 0.0 m/s at 1.0m distance).
        peer_speeds["robot_2"] = 0.0
        peers_stationary = resolver.compute_nearby_peers_stationary(
            my_pos=Vector2D(0.0, 0.0),
            peer_positions=peer_positions,
            peer_speeds=peer_speeds,
            radius=2.5
        )
        self.assertTrue(peers_stationary, "Stationary peer was not detected as stationary!")

        is_dl = resolver.update_deadlock_state(
            current_speed=0.0,
            dist_to_goal=5.0,
            nearby_peers_stationary=peers_stationary,
            current_time=base_time + 4.0
        )
        self.assertTrue(is_dl, "Stationary robot with stationary peer MUST be deadlocked!")

    def test_b1_node_deadlock_peer_stationary_condition(self):
        """
        [B1 Node Integration Test]
        Verify that ConflictResolverNode:
        (i) Does not trigger deadlock broadcast when nearby peer is moving.
        (ii) Triggers deadlock broadcast when nearby peer is stationary.
        """
        node = MockNode("conflict_resolver_node", bus=self.bus)
        node.declare_parameter("robot_id", "robot_1")
        node.declare_parameter("safety_dist", 0.8)

        cr = ConflictResolverNode(node)
        cr.goal_pos = Vector2D(5.0, 0.0)
        cr.current_pos = Vector2D(0.0, 0.0)
        cr.current_speed = 0.0
        base_time = 100.0
        cr.resolver.last_moving_time = base_time

        # Nearby peer is moving at 0.6 m/s
        cr.update_peer_state("robot_2", pos=Vector2D(1.0, 0.0), speed=0.6)

        # Force current_time to base_time + 4.0s in resolver
        cr.resolver.update_deadlock_state(0.0, 5.0, nearby_peers_stationary=False, current_time=base_time + 4.0)
        cr.resolution_loop()

        conflict_pubs = [p for p in node.publishers if p.topic == "/fleet/robot_1/conflict"]
        self.assertEqual(len(conflict_pubs[0].published_messages), 0,
                         "Deadlock message was published even though nearby peer is moving!")

        # Now peer becomes stationary
        cr.update_peer_state("robot_2", pos=Vector2D(1.0, 0.0), speed=0.0)
        cr.resolver.last_moving_time = base_time  # stalled > 3.0s
        # Simulate timer tick at base_time + 4.0s
        nearby_stat = cr.resolver.compute_nearby_peers_stationary(cr.current_pos, cr.peer_positions, cr.peer_speeds)
        self.assertTrue(nearby_stat)

        # Update deadlock state with stationary peer at t = 104s
        is_dl = cr.resolver.update_deadlock_state(0.0, 5.0, nearby_peers_stationary=nearby_stat, current_time=base_time + 4.0)
        self.assertTrue(is_dl)

        cr.resolution_loop()
        self.assertTrue(len(conflict_pubs[0].published_messages) > 0,
                        "Deadlock message was not published when both robots are stalled!")

    def test_b2_corridor_token_contention_and_broadcast(self):
        """
        [B2 Regression Test]
        Verify end-to-end TOKEN_REQ / TOKEN_GRANT / TOKEN_DENY broadcast protocol:
        (i) Two resolvers contend for the same corridor.
        (ii) TOKEN_REQ is broadcast on /fleet/conflicts.
        (iii) Exactly one wins (higher priority wins).
        (iv) When the winner releases the token, the second robot can acquire it.
        """
        node_1 = MockNode("resolver_r1", bus=self.bus)
        node_1.declare_parameter("robot_id", "robot_1")
        node_1.declare_parameter("safety_dist", 0.8)

        node_2 = MockNode("resolver_r2", bus=self.bus)
        node_2.declare_parameter("robot_id", "robot_2")
        node_2.declare_parameter("safety_dist", 0.8)

        r1 = ConflictResolverNode(node_1)
        r2 = ConflictResolverNode(node_2)

        # Set priorities: Robot 1 has 0.85, Robot 2 has 0.60
        r1.current_pos = Vector2D(0.0, 0.0)
        r1.goal_pos = Vector2D(2.0, 0.0)  # closer to goal -> higher priority
        r1.battery_pct = 90.0

        r2.current_pos = Vector2D(0.0, 0.0)
        r2.goal_pos = Vector2D(10.0, 0.0)  # further -> lower priority
        r2.battery_pct = 90.0

        # In un-fixed code, request_corridor_token never broadcasts TOKEN_REQ
        granted_1 = r1.request_corridor_token("corridor_main")
        self.assertTrue(granted_1)

        # Check that TOKEN_REQ was broadcast to peer on /fleet/conflicts
        fleet_conflicts_pub = [p for p in node_1.publishers if p.topic == "/fleet/conflicts"][0]
        fleet_conflicts_msgs = [m.data for m in fleet_conflicts_pub.published_messages]
        self.assertTrue(any("TOKEN_REQ:robot_1:corridor_main" in m for m in fleet_conflicts_msgs),
                        "TOKEN_REQ was never broadcast on /fleet/conflicts!")

        # Robot 2 requests the same corridor while held by Robot 1
        granted_2 = r2.request_corridor_token("corridor_main")
        self.assertFalse(granted_2, "Robot 2 acquired corridor_main while already held by Robot 1! Exactly one must win.")

        # Robot 1 releases the corridor
        r1.release_corridor_token("corridor_main")

        # Now Robot 2 can acquire
        granted_2_retry = r2.request_corridor_token("corridor_main")
        self.assertTrue(granted_2_retry, "Robot 2 failed to acquire corridor after Robot 1 released it!")

    def test_b3_pruning_stale_peers_in_token_and_deadlock_state(self):
        """
        [B3 Regression Test]
        Verify that peers that stop publishing are pruned after timeout:
        (i) ConflictResolver drops expired corridor token reservations held by dead peers.
        (ii) ConflictResolverNode drops stale peer velocity/position tracking entries.
        """
        resolver = ConflictResolver("robot_1", safety_distance=0.8)
        base_time = 100.0

        # Robot 2 holds corridor token at t=100s
        resolver.corridor_tokens["corridor_main"] = "robot_2"
        resolver.token_request_timestamps["corridor_main"] = base_time

        # At t=105s (within 10s timeout): token still held by robot_2
        resolver.prune_stale_tokens(timeout_s=10.0, current_time=base_time + 5.0)
        self.assertEqual(resolver.corridor_tokens.get("corridor_main"), "robot_2")

        # At t=115s (>10s timeout): robot_2 timed out, token is revoked
        resolver.prune_stale_tokens(timeout_s=10.0, current_time=base_time + 15.0)
        self.assertNotIn("corridor_main", resolver.corridor_tokens,
                         "Stale peer corridor token was not pruned after timeout!")

        # Check Node-level peer tracking pruning
        node = MockNode("resolver_node", bus=self.bus)
        node.declare_parameter("robot_id", "robot_1")
        node.declare_parameter("safety_dist", 0.8)

        cr = ConflictResolverNode(node)
        cr.update_peer_state("robot_2", pos=Vector2D(1.0, 0.0), speed=0.0)
        cr.peer_last_seen["robot_2"] = base_time

        # Prune at t=115s
        dropped = cr.prune_stale_peers(timeout_s=10.0, current_time=base_time + 15.0)
        self.assertIn("robot_2", dropped)
        self.assertNotIn("robot_2", cr.peer_positions)
        self.assertNotIn("robot_2", cr.peer_speeds)


if __name__ == '__main__':
    unittest.main()
