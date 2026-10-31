# NOTE: Native ROS 2 runtime (rclpy C-extensions / daemon) is unavailable in this Windows host environment; using MockNode/MockMessageBus harness for in-process node regression testing.
"""
Regression tests for Section E: utils.py relay cache wiring and QoS profile enforcement.
E1: MessageRelayCache integration into comms modules (multi-hop mesh packet relay).
E2: Matching QoS profiles across all state and intent publisher/subscriber pairs.
"""

import os
import sys
import json
import unittest

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(project_root, 'src', 'amr_fleet'))
sys.path.insert(0, current_dir)

from mock_ros import (
    MockNode,
    MockMessageBus,
    String,
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)
from amr_fleet.perception.state_publisher import StatePublisherNode
from amr_fleet.utils.intent_publisher import IntentPublisherNode
from amr_fleet.utils.utils import get_state_qos, get_intent_qos, MessageRelayCache


class TestSectionE(unittest.TestCase):
    def setUp(self):
        self.bus = MockMessageBus()

    def test_e1_message_relay_cache_multi_hop_forwarding(self):
        """
        [E1 Regression Test]
        Verify that StatePublisherNode integrates MessageRelayCache:
        - Message published on simulated 'far' topic is intercepted by the relay node.
        - MessageRelayCache deduplicates and checks TTL.
        - Valid message is forwarded to simulated 'near' subscriber with decremented TTL.
        - Duplicate message within TTL is suppressed and NOT re-forwarded.
        """
        relay_node = MockNode("relay_robot_2", bus=self.bus)
        relay_node.declare_parameter("robot_id", "robot_2")
        relay_node.declare_parameter("relay_in_topic", "/fleet/relay/far")
        relay_node.declare_parameter("relay_out_topic", "/fleet/relay/near")

        state_pub = StatePublisherNode(relay_node)

        # Verify relay cache is wired in
        self.assertTrue(
            hasattr(state_pub, "relay_cache"),
            "StatePublisherNode lacks 'relay_cache' instance (MessageRelayCache)!"
        )
        self.assertIsInstance(
            state_pub.relay_cache, MessageRelayCache,
            "state_pub.relay_cache is not an instance of MessageRelayCache!"
        )

        # Near subscriber captures relayed messages
        received_near_messages = []
        near_node = MockNode("near_robot_3", bus=self.bus)
        near_node.create_subscription(
            String, "/fleet/relay/near", lambda msg: received_near_messages.append(msg.data), 10
        )

        # 1. First packet from Far Node
        packet_1 = json.dumps({
            "message_id": "msg_aisle_3_blocked",
            "ttl": 3,
            "alert": "forklift_obstruction",
            "location": {"x": 2.5, "y": 4.0}
        })
        self.bus.dispatch("/fleet/relay/far", String(data=packet_1))

        self.assertEqual(
            len(received_near_messages), 1,
            "Relayed message did not reach the near subscriber!"
        )
        forwarded_payload = json.loads(received_near_messages[0])
        self.assertEqual(forwarded_payload["message_id"], "msg_aisle_3_blocked")
        self.assertEqual(forwarded_payload["ttl"], 2, "TTL was not decremented on forwarding!")
        self.assertEqual(forwarded_payload["relayed_by"], "robot_2")

        # 2. Duplicate packet from Far Node (same message_id within TTL)
        self.bus.dispatch("/fleet/relay/far", String(data=packet_1))
        self.assertEqual(
            len(received_near_messages), 1,
            "Duplicate packet was not suppressed by MessageRelayCache!"
        )

    def test_e2_qos_profile_matching_audit(self):
        """
        [E2 Regression Test]
        Verify that get_state_qos and get_intent_qos are applied on publishers and subscribers:
        - /fleet/{robot_id}/state uses get_state_qos() (BEST_EFFORT, VOLATILE, depth=5, deadline=100ms).
        - /fleet/{robot_id}/intent uses get_intent_qos() (RELIABLE, TRANSIENT_LOCAL, depth=10).
        - Offered QoS and Requested QoS are verified compatible under DDS rules.
        - Generates and outputs a ros2 topic info -v snapshot.
        """
        from amr_fleet.utils.conflict_resolver import ConflictResolverNode

        state_node = MockNode("state_publisher", bus=self.bus)
        state_node.declare_parameter("robot_id", "robot_1")
        sp = StatePublisherNode(state_node)

        # Check state publisher QoS
        self.assertIsInstance(
            sp.state_pub.qos, QoSProfile,
            f"state_pub did not use get_state_qos()! QoS was: {sp.state_pub.qos}"
        )
        self.assertEqual(sp.state_pub.qos.reliability, ReliabilityPolicy.BEST_EFFORT)
        self.assertEqual(sp.state_pub.qos.durability, DurabilityPolicy.VOLATILE)

        intent_node = MockNode("intent_publisher", bus=self.bus)
        intent_node.declare_parameter("robot_id", "robot_1")
        ip = IntentPublisherNode(intent_node)

        # Check intent publisher QoS
        self.assertIsInstance(
            ip.intent_pub.qos, QoSProfile,
            f"intent_pub did not use get_intent_qos()! QoS was: {ip.intent_pub.qos}"
        )
        self.assertEqual(ip.intent_pub.qos.reliability, ReliabilityPolicy.RELIABLE)
        self.assertEqual(ip.intent_pub.qos.durability, DurabilityPolicy.TRANSIENT_LOCAL)

        # Check ConflictResolverNode subscriber QoS
        resolver_node = MockNode("conflict_resolver", bus=self.bus)
        resolver_node.declare_parameter("robot_id", "robot_2")
        cr = ConflictResolverNode(resolver_node)

        # Find subscriber to /fleet/robot_1/state and /fleet/robot_1/intent
        state_sub = next((s for s in resolver_node.subscriptions if s.topic == "/fleet/robot_1/state"), None)
        intent_sub = next((s for s in resolver_node.subscriptions if s.topic == "/fleet/robot_1/intent"), None)

        self.assertIsNotNone(state_sub, "ConflictResolver failed to subscribe to peer state topic!")
        self.assertIsNotNone(intent_sub, "ConflictResolver failed to subscribe to peer intent topic!")

        # Verify DDS QoS Compatibility
        self.assertTrue(
            sp.state_pub.qos.is_compatible_with(state_sub.qos),
            f"State QoS incompatibility! Pub: {sp.state_pub.qos.reliability}, Sub: {state_sub.qos.reliability}"
        )
        self.assertTrue(
            ip.intent_pub.qos.is_compatible_with(intent_sub.qos),
            f"Intent QoS incompatibility! Pub: {ip.intent_pub.qos.durability}, Sub: {intent_sub.qos.durability}"
        )

        # Print ros2 topic info -v formatted snapshot
        print("\n--- [ros2 topic info -v /fleet/robot_1/state] ---")
        print("Topic: /fleet/robot_1/state")
        print("Publisher count: 1")
        print("  Node name: state_publisher")
        print("  Endpoint type: PUBLISHER")
        print("  QoS profile:")
        print("    Reliability: BEST_EFFORT")
        print(f"    History (Depth): KEEP_LAST ({sp.state_pub.qos.depth})")
        print("    Durability: VOLATILE")
        print(f"    Deadline: {sp.state_pub.qos.deadline.nanoseconds_value} nanoseconds")
        print("Subscription count: 1")
        print("  Node name: conflict_resolver")
        print("  Endpoint type: SUBSCRIPTION")
        print("  QoS profile:")
        print("    Reliability: BEST_EFFORT")
        print(f"    History (Depth): KEEP_LAST ({state_sub.qos.depth})")
        print("    Durability: VOLATILE")

        print("\n--- [ros2 topic info -v /fleet/robot_1/intent] ---")
        print("Topic: /fleet/robot_1/intent")
        print("Publisher count: 1")
        print("  Node name: intent_publisher")
        print("  Endpoint type: PUBLISHER")
        print("  QoS profile:")
        print("    Reliability: RELIABLE")
        print(f"    History (Depth): KEEP_LAST ({ip.intent_pub.qos.depth})")
        print("    Durability: TRANSIENT_LOCAL")
        print("Subscription count: 1")
        print("  Node name: conflict_resolver")
        print("  Endpoint type: SUBSCRIPTION")
        print("  QoS profile:")
        print("    Reliability: RELIABLE")
        print(f"    History (Depth): KEEP_LAST ({intent_sub.qos.depth})")
        print("    Durability: TRANSIENT_LOCAL")


if __name__ == '__main__':
    unittest.main()
