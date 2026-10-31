#!/usr/bin/env python3
"""
Headless Demonstration & Topic Echo for Section A.
Exercises:
1) state_publisher + battery_monitor
2) intent_publisher + global_planner + conflict_resolver
Echos published messages to verify real value updates.
"""

import json
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(project_root, 'src', 'amr_fleet'))
sys.path.insert(0, os.path.join(project_root, 'tests'))

from mock_ros import (
    install_ros_mocks,
    MockNode,
    MockMessageBus,
    PoseStamped,
    Point,
    OccupancyGrid,
)
install_ros_mocks()

from amr_fleet.utils import Vector2D
from amr_fleet.perception.state_publisher import StatePublisherNode
from amr_fleet.perception.battery_monitor import BatteryMonitorNode
from amr_fleet.planning.global_planner import GlobalPlannerNode
from amr_fleet.utils.conflict_resolver import ConflictResolverNode
from amr_fleet.utils.intent_publisher import IntentPublisherNode


def run_group1():
    print("=" * 70)
    print("GROUP 1: state_publisher + battery_monitor topic echo")
    print("=" * 70)
    bus = MockMessageBus()

    # Track echoed messages
    echoed = {}
    def make_echo(topic):
        def _cb(msg):
            echoed.setdefault(topic, []).append(msg)
        return _cb

    bus.register_sub("/fleet/robot_1/battery", make_echo("/fleet/robot_1/battery"))
    bus.register_sub("/fleet/robot_1/charging_intent", make_echo("/fleet/robot_1/charging_intent"))
    bus.register_sub("/fleet/robot_1/state", make_echo("/fleet/robot_1/state"))

    # Instantiate nodes
    bm_node = MockNode("battery_monitor", bus=bus)
    bm_node.declare_parameter("robot_id", "robot_1")
    bm_node.declare_parameter("initial_battery", 14.5)  # Critical (<15%) to trigger charging intent
    bm = BatteryMonitorNode(bm_node)

    sp_node = MockNode("state_publisher", bus=bus)
    sp_node.declare_parameter("robot_id", "robot_1")
    sp_node.declare_parameter("publish_rate_hz", 20.0)
    sp = StatePublisherNode(sp_node)

    # Battery monitor tick (publishes battery and charging intent)
    bm.monitor_tick()

    # State publisher tick (publishes state)
    sp.publish_state()

    print("\n--- [TOPIC ECHO] /fleet/robot_1/battery ---")
    for msg in echoed.get("/fleet/robot_1/battery", []):
        print(f"data: {msg.data:.2f}")

    print("\n--- [TOPIC ECHO] /fleet/robot_1/charging_intent ---")
    for msg in echoed.get("/fleet/robot_1/charging_intent", []):
        print(f'data: "{msg.data}"')

    print("\n--- [TOPIC ECHO] /fleet/robot_1/state ---")
    for msg in echoed.get("/fleet/robot_1/state", []):
        parsed = json.loads(msg.data)
        print(json.dumps(parsed, indent=2))


def run_group2():
    print("\n" + "=" * 70)
    print("GROUP 2: intent_publisher + global_planner + conflict_resolver topic echo")
    print("=" * 70)
    bus = MockMessageBus()

    echoed = {}
    def make_echo(topic):
        def _cb(msg):
            echoed.setdefault(topic, []).append(msg)
        return _cb

    bus.register_sub("/fleet/robot_1/goal", make_echo("/fleet/robot_1/goal"))
    bus.register_sub("/fleet/robot_1/plan", make_echo("/fleet/robot_1/plan"))
    bus.register_sub("/fleet/robot_1/priority", make_echo("/fleet/robot_1/priority"))
    bus.register_sub("/fleet/robot_1/token_status", make_echo("/fleet/robot_1/token_status"))
    bus.register_sub("/fleet/robot_1/intent", make_echo("/fleet/robot_1/intent"))

    gp_node = MockNode("global_planner", bus=bus)
    gp_node.declare_parameter("robot_id", "robot_1")
    gp_node.declare_parameter("rolling_horizon_s", 8.0)
    gp_node.declare_parameter("nominal_speed", 0.5)
    gp = GlobalPlannerNode(gp_node)

    cr_node = MockNode("conflict_resolver", bus=bus)
    cr_node.declare_parameter("robot_id", "robot_1")
    cr_node.declare_parameter("safety_dist", 0.8)
    cr = ConflictResolverNode(cr_node)

    ip_node = MockNode("intent_publisher", bus=bus)
    ip_node.declare_parameter("robot_id", "robot_1")
    ip_node.declare_parameter("publish_rate_hz", 2.0)
    ip = IntentPublisherNode(ip_node)

    # 1. Provide map and current pose to global planner
    map_msg = OccupancyGrid()
    gp.map_callback(map_msg)

    pose_msg = PoseStamped()
    pose_msg.pose.position.x = 0.0
    pose_msg.pose.position.y = 0.0
    gp.pose_callback(pose_msg)
    cr.pose_callback(pose_msg)

    # 2. Dispatch a Goal on /fleet/robot_1/goal
    goal_msg = Point(x=5.0, y=0.0, z=0.0)
    bus.dispatch("/fleet/robot_1/goal", goal_msg)

    # 3. Conflict resolver requests corridor token and updates resolution loop
    cr.request_corridor_token("corridor_main")
    cr.battery_pct = 85.0
    cr.resolution_loop()

    # 4. Intent publisher publishes intent
    ip.publish_intent()

    print("\n--- [TOPIC ECHO] /fleet/robot_1/goal ---")
    for msg in echoed.get("/fleet/robot_1/goal", []):
        print(f"x: {msg.x:.2f}, y: {msg.y:.2f}, z: {msg.z:.2f}")

    print("\n--- [TOPIC ECHO] /fleet/robot_1/plan ---")
    for msg in echoed.get("/fleet/robot_1/plan", []):
        print(f"poses count: {len(msg.poses)}")
        for i, ps in enumerate(msg.poses[:3]):
            print(f"  [{i}] x: {ps.pose.position.x:.2f}, y: {ps.pose.position.y:.2f}")
        if len(msg.poses) > 3:
            print(f"  ... ({len(msg.poses) - 3} more waypoints)")

    print("\n--- [TOPIC ECHO] /fleet/robot_1/priority ---")
    for msg in echoed.get("/fleet/robot_1/priority", []):
        print(f"data: {msg.data:.3f}")

    print("\n--- [TOPIC ECHO] /fleet/robot_1/token_status ---")
    for msg in echoed.get("/fleet/robot_1/token_status", []):
        print(f'data: "{msg.data}"')

    print("\n--- [TOPIC ECHO] /fleet/robot_1/intent ---")
    for msg in echoed.get("/fleet/robot_1/intent", []):
        parsed = json.loads(msg.data)
        print(json.dumps(parsed, indent=2))


if __name__ == '__main__':
    run_group1()
    run_group2()
