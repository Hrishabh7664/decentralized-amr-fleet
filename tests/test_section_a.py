# NOTE: Native ROS 2 runtime (rclpy C-extensions / daemon) is unavailable in this Windows host environment; using MockNode/MockMessageBus harness for in-process node regression testing.
"""
Regression tests for Section A: Topic plumbing mismatches.
A1: StatePublisher battery topic alignment and BatteryState/Float32 handling.
A2: StatePublisher charging intent subscription and state mapping.
A3: IntentPublisher plan topic alignment.
A4: IntentPublisher goal topic alignment.
A5: ConflictResolver priority and token_status publishing to IntentPublisher.
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
    BatteryState,
    Float32,
    String,
    Point,
    Path,
    PoseStamped,
)

install_ros_mocks()

from amr_fleet.utils import Vector2D
from amr_fleet.perception.state_publisher import StatePublisherNode
from amr_fleet.perception.battery_monitor import BatteryMonitorNode


class TestSectionA(unittest.TestCase):
    def setUp(self):
        self.bus = MockMessageBus()

    def test_a1_state_publisher_receives_battery_state(self):
        """
        [A1 Regression Test]
        Verify that StatePublisher correctly subscribes to /fleet/{robot_id}/battery,
        handles both Float32 and BatteryState messages, and reflects the updated battery
        percentage in /fleet/{robot_id}/state.
        """
        node = MockNode("state_publisher_node", bus=self.bus)
        node.declare_parameter("robot_id", "robot_1")
        node.declare_parameter("publish_rate_hz", 20.0)

        sp = StatePublisherNode(node)

        # Before any battery message, battery is initial 100.0%
        self.assertEqual(sp.battery_pct, 100.0)

        # Publish a BatteryState message on the monitor's topic /fleet/robot_1/battery
        battery_msg = BatteryState(percentage=0.72)
        # Dispatch through message bus
        self.bus.dispatch("/fleet/robot_1/battery", battery_msg)

        # Assert that StatePublisher updated its internal battery to 72.0%
        self.assertAlmostEqual(sp.battery_pct, 72.0, delta=0.1,
                               msg="Battery percentage did not update from BatteryState message!")

        # Trigger state publishing and inspect published output
        sp.publish_state()
        state_pubs = [p for p in node.publishers if p.topic == "/fleet/robot_1/state"]
        self.assertTrue(len(state_pubs) > 0)
        last_msg = state_pubs[0].published_messages[-1]
        payload = json.loads(last_msg.data)
        self.assertAlmostEqual(payload["battery"], 72.0, delta=0.1,
                               msg="Published /fleet/robot_1/state did not reflect BatteryState percentage!")

    def test_a1b_state_publisher_receives_float32(self):
        """
        [A1b Regression Test]
        Verify that StatePublisher correctly receives Float32 telemetry messages on
        /fleet/{robot_id}/battery and updates /fleet/{robot_id}/state.
        """
        node = MockNode("state_publisher_node", bus=self.bus)
        node.declare_parameter("robot_id", "robot_1")
        node.declare_parameter("publish_rate_hz", 20.0)

        sp = StatePublisherNode(node)
        self.assertEqual(sp.battery_pct, 100.0)

        # Dispatch Float32 on the monitor's published topic
        self.bus.dispatch("/fleet/robot_1/battery", Float32(data=45.5))
        self.assertAlmostEqual(sp.battery_pct, 45.5, delta=0.1)

        sp.publish_state()
        state_pubs = [p for p in node.publishers if p.topic == "/fleet/robot_1/state"]
        last_msg = state_pubs[0].published_messages[-1]
        payload = json.loads(last_msg.data)
        self.assertAlmostEqual(payload["battery"], 45.5, delta=0.1)

    def test_a2_charging_intent_maps_to_charging_state(self):
        """
        [A2 Regression Test]
        Verify that StatePublisher subscribes to /fleet/{robot_id}/charging_intent,
        and when a charging intent message arrives, updates is_charging to True
        and current_state to 'CHARGING'.
        """
        node = MockNode("state_publisher_node", bus=self.bus)
        node.declare_parameter("robot_id", "robot_1")
        node.declare_parameter("publish_rate_hz", 20.0)

        sp = StatePublisherNode(node)
        self.assertEqual(sp.current_state, "IDLE")
        self.assertFalse(sp.is_charging)

        # Dispatch charging intent on /fleet/robot_1/charging_intent (as published by battery_monitor)
        intent_msg = String(data="CHARGING_INTENT:robot_1:station_north_west")
        self.bus.dispatch("/fleet/robot_1/charging_intent", intent_msg)

        # In un-fixed code, StatePublisher does not subscribe to /fleet/robot_1/charging_intent
        # so current_state never becomes CHARGING and is_charging remains False.
        self.assertTrue(sp.is_charging, "is_charging was not set to True upon charging intent!")
        self.assertEqual(sp.current_state, "CHARGING", "current_state did not become CHARGING!")

        sp.publish_state()
        state_pubs = [p for p in node.publishers if p.topic == "/fleet/robot_1/state"]
        last_msg = state_pubs[0].published_messages[-1]
        payload = json.loads(last_msg.data)
        self.assertEqual(payload["state"], "CHARGING")

    def test_a3_intent_publisher_subscribes_to_plan(self):
        """
        [A3 Regression Test]
        Verify that IntentPublisher receives Path on both /fleet/{robot_id}/plan and
        legacy /{robot_id}/plan, updates planned_path, and outputs the path in /fleet/{id}/intent.
        """
        from amr_fleet.utils.intent_publisher import IntentPublisherNode

        node = MockNode("intent_publisher_node", bus=self.bus)
        node.declare_parameter("robot_id", "robot_1")
        node.declare_parameter("publish_rate_hz", 2.0)

        ip = IntentPublisherNode(node)
        self.assertEqual(len(ip.planned_path), 0)

        path_msg = Path()
        for x, y in [(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]:
            ps = PoseStamped()
            ps.pose.position.x = x
            ps.pose.position.y = y
            path_msg.poses.append(ps)

        # In un-fixed code, publishing on legacy /{robot_id}/plan fails to update planned_path
        self.bus.dispatch("/robot_1/plan", path_msg)
        self.assertEqual(len(ip.planned_path), 3, "planned_path was empty after publishing on /robot_1/plan!")

        ip.publish_intent()
        intent_pubs = [p for p in node.publishers if p.topic == "/fleet/robot_1/intent"]
        self.assertTrue(len(intent_pubs) > 0)
        last_msg = intent_pubs[0].published_messages[-1]
        payload = json.loads(last_msg.data)
        self.assertEqual(len(payload["path"]), 3)

    def test_a4_intent_publisher_subscribes_to_goal(self):
        """
        [A4 Regression Test]
        Verify that IntentPublisher receives goals dispatched on /fleet/{robot_id}/goal
        as well as legacy /{robot_id}/goal, updating current_goal from (0,0) to target coordinates.
        """
        from amr_fleet.utils.intent_publisher import IntentPublisherNode

        node = MockNode("intent_publisher_node", bus=self.bus)
        node.declare_parameter("robot_id", "robot_1")
        node.declare_parameter("publish_rate_hz", 2.0)

        ip = IntentPublisherNode(node)
        self.assertEqual(ip.current_goal.x, 0.0)
        self.assertEqual(ip.current_goal.y, 0.0)

        # Dispatch real goal from fleet controller/task allocator topic
        self.bus.dispatch("/fleet/robot_1/goal", Point(x=12.5, y=-4.0))
        self.assertAlmostEqual(ip.current_goal.x, 12.5, delta=0.01)
        self.assertAlmostEqual(ip.current_goal.y, -4.0, delta=0.01)

        # Dispatch on legacy topic
        self.bus.dispatch("/robot_1/goal", Point(x=15.0, y=-2.0))
        self.assertAlmostEqual(ip.current_goal.x, 15.0, delta=0.01)
        self.assertAlmostEqual(ip.current_goal.y, -2.0, delta=0.01)

        ip.publish_intent()
        intent_pubs = [p for p in node.publishers if p.topic == "/fleet/robot_1/intent"]
        last_msg = intent_pubs[0].published_messages[-1]
        payload = json.loads(last_msg.data)
        self.assertAlmostEqual(payload["goal"]["x"], 15.0, delta=0.01)
        self.assertAlmostEqual(payload["goal"]["y"], -2.0, delta=0.01)

    def test_a5_conflict_resolver_publishes_priority_and_token_status(self):
        """
        [A5 Regression Test]
        Verify that ConflictResolver publishes dynamic priority on /fleet/{robot_id}/priority
        and corridor token status on /fleet/{robot_id}/token_status, and IntentPublisher
        receives and broadcasts both in /fleet/{robot_id}/intent.
        """
        from amr_fleet.utils.conflict_resolver import ConflictResolverNode
        from amr_fleet.utils.intent_publisher import IntentPublisherNode

        cr_node = MockNode("conflict_resolver_node", bus=self.bus)
        cr_node.declare_parameter("robot_id", "robot_1")
        cr_node.declare_parameter("safety_dist", 0.8)

        ip_node = MockNode("intent_publisher_node", bus=self.bus)
        ip_node.declare_parameter("robot_id", "robot_1")
        ip_node.declare_parameter("publish_rate_hz", 2.0)

        cr = ConflictResolverNode(cr_node)
        ip = IntentPublisherNode(ip_node)

        # In un-fixed code, ConflictResolverNode does not have priority_pub or token_pub
        priority_pubs = [p for p in cr_node.publishers if p.topic == "/fleet/robot_1/priority"]
        token_pubs = [p for p in cr_node.publishers if p.topic == "/fleet/robot_1/token_status"]
        self.assertTrue(len(priority_pubs) > 0, "ConflictResolverNode lacks /fleet/{robot_id}/priority publisher!")
        self.assertTrue(len(token_pubs) > 0, "ConflictResolverNode lacks /fleet/{robot_id}/token_status publisher!")

        # Set goal and battery to trigger resolution_loop priority calculation
        cr.goal_pos = Vector2D(5.0, 0.0)
        cr.current_pos = Vector2D(0.0, 0.0)  # dist_to_goal = 5.0
        cr.battery_pct = 80.0
        cr.resolution_loop()

        # Check that priority was published and received by intent publisher
        self.assertNotEqual(ip.priority_score, 0.5, "IntentPublisher priority was not updated by ConflictResolver!")

        # Trigger token acquisition and check token publication
        cr.request_corridor_token("corridor_main")
        self.assertTrue(ip.has_token, "IntentPublisher has_token was not updated to True!")
        self.assertEqual(ip.reserved_corridor_id, "corridor_main")


if __name__ == '__main__':
    unittest.main()

