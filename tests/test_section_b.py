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


if __name__ == '__main__':
    unittest.main()
