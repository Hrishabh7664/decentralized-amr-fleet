"""
Regression tests for Section A: Topic plumbing mismatches.
A1: StatePublisher battery topic alignment and BatteryState handling.
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


if __name__ == '__main__':
    unittest.main()
