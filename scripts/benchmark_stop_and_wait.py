"""
Benchmark Baseline: Stop-and-Wait Collision Avoidance.
Faithfully models industrial stop-and-wait / zone-locking behavior:
When two robots have overlapping corridor paths or converge on a choke point,
the lower-priority robot must wait outside the interaction zone until the higher-priority
robot has completely cleared the passage.
Outputs benchmark results to CSV for comparison against the decentralized framework.
"""

from __future__ import annotations
import os
import sys
import csv
import math
import time
from dataclasses import dataclass
from typing import List, Dict, Tuple

# Add src/amr_fleet to python path for standalone execution
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(project_root, 'src', 'amr_fleet'))

from amr_fleet.utils import Vector2D, Pose2D
from amr_fleet.global_planner import AStarPlanner


@dataclass
class BaselineAgent:
    id: str
    position: Vector2D
    goal: Vector2D
    path: List[Vector2D]
    speed: float = 0.6  # m/s
    is_waiting: bool = False
    completed: bool = False
    completion_time: float = 0.0
    collisions: int = 0
    priority: float = 0.5
    waiting_time: float = 0.0


def run_scenario(name: str, agents_config: List[Dict], max_steps: int = 4000, dt: float = 0.05) -> Dict:
    planner = AStarPlanner()
    if "Blocked_Aisle" in name:
        # Mid-run aisle blockage at x=0, y=2
        planner.set_blocked_region(-1.0, 1.0, 1.0, 3.0, blocked=True)

    agents: List[BaselineAgent] = []
    for cfg in agents_config:
        start = Vector2D(*cfg['start'])
        goal = Vector2D(*cfg['goal'])
        path = planner.plan_path(start, goal)
        agents.append(BaselineAgent(
            id=cfg['id'],
            position=start,
            goal=goal,
            path=path,
            priority=cfg.get('priority', 0.5)
        ))

    sim_time = 0.0
    deadlock_count = 0
    total_collisions = 0
    collision_pairs = set()

    # Zone locking / Corridor detection: shared central corridor [-6 to 6] on X
    for step in range(max_steps):
        sim_time += dt
        all_done = True

        for i, a1 in enumerate(agents):
            if a1.completed:
                continue
            all_done = False

            # Check if another higher-priority robot is occupying or traversing the corridor/zone
            must_wait = False
            for j, a2 in enumerate(agents):
                if i == j or a2.completed:
                    continue

                # Head-on corridor contention or close-proximity conflict zone (within 2.5m)
                dist = a1.position.distance_to(a2.position)
                
                # Check physical collision
                if dist < 0.38:
                    pair = tuple(sorted([a1.id, a2.id]))
                    if pair not in collision_pairs:
                        total_collisions += 1
                        collision_pairs.add(pair)

            # Stop-and-Wait Rule:
            # 1. Single-lane shared corridor zone reservation (X in [-8.0, 8.0], |Y| < 1.5)
            in_corridor_track = (abs(a1.position.y) < 1.5)
            if in_corridor_track:
                for j, a2 in enumerate(agents):
                    if i == j or a2.completed:
                        continue
                    # Peer is also on the central corridor track
                    if abs(a2.position.y) < 1.5:
                        peer_in_corridor = (-8.0 <= a2.position.x <= 8.0)
                        higher_priority = (a1.priority < a2.priority) or (a1.priority == a2.priority and a1.id > a2.id)
                        
                        # If peer is inside corridor or approaching with higher priority, hold outside
                        if peer_in_corridor and higher_priority:
                            # If a1 is still outside entrance or at boundary, wait outside
                            if a1.position.x <= -7.5 or a1.position.x >= 7.5:
                                must_wait = True
                                break
                            # If both caught inside, lower priority yields / stops
                            elif higher_priority:
                                must_wait = True
                                break

            # 2. General proximity conflict zone (within 2.0m)
            if not must_wait:
                for j, a2 in enumerate(agents):
                    if i == j or a2.completed:
                        continue
                    dist = a1.position.distance_to(a2.position)
                    if dist < 2.0:
                        if (a1.priority < a2.priority) or (a1.priority == a2.priority and a1.id > a2.id):
                            must_wait = True
                            break

            # 3. Industrial Safety Field / Bumper: Halt if peer is directly in path within 0.6m
            if not must_wait and a1.path:
                move_dir = (a1.path[0] - a1.position).normalized()
                for j, a2 in enumerate(agents):
                    if i == j or a2.completed:
                        continue
                    dist = a1.position.distance_to(a2.position)
                    if dist < 0.60:
                        to_peer = a2.position - a1.position
                        if to_peer.norm() > 1e-4 and move_dir.dot(to_peer.normalized()) > 0.5:
                            must_wait = True
                            break

            a1.is_waiting = must_wait

            if must_wait:
                a1.waiting_time += dt
                # Stationary in stop-and-wait
                continue

            if a1.path:
                target = a1.path[0]
                to_target = target - a1.position
                dist_wp = to_target.norm()

                if dist_wp < 0.25:
                    a1.path.pop(0)
                    if not a1.path:
                        a1.completed = True
                        a1.completion_time = sim_time
                else:
                    move_dist = min(dist_wp, a1.speed * dt)
                    a1.position = a1.position + to_target.normalized() * move_dist

        if all_done:
            break

    total_time = sim_time
    avg_time = sum(a.completion_time if a.completed else total_time for a in agents) / len(agents)

    return {
        "scenario": name,
        "method": "Stop_and_Wait_Baseline",
        "agent_count": len(agents),
        "total_completion_time_s": round(total_time, 2),
        "avg_agent_time_s": round(avg_time, 2),
        "collisions": total_collisions,
        "deadlocks": deadlock_count,
        "completed_count": sum(1 for a in agents if a.completed)
    }


def main():
    scenarios = [
        {
            "name": "Overlapping_Paths_3_Robots",
            "agents": [
                {"id": "robot_1", "start": (-10.0, 0.0), "goal": (10.0, 0.0), "priority": 0.8},
                {"id": "robot_2", "start": (10.0, 0.0), "goal": (-10.0, 0.0), "priority": 0.4},
                {"id": "robot_3", "start": (0.0, 8.0), "goal": (0.0, -8.0), "priority": 0.6},
            ]
        },
        {
            "name": "Narrow_Intersection_4_Robots",
            "agents": [
                {"id": "robot_1", "start": (-8.0, 0.0), "goal": (8.0, 0.0), "priority": 0.9},
                {"id": "robot_2", "start": (8.0, 0.0), "goal": (-8.0, 0.0), "priority": 0.3},
                {"id": "robot_3", "start": (0.0, 7.0), "goal": (0.0, -7.0), "priority": 0.7},
                {"id": "robot_4", "start": (0.0, -7.0), "goal": (0.0, 7.0), "priority": 0.5},
            ]
        },
        {
            "name": "Blocked_Aisle_Rerouting",
            "agents": [
                {"id": "robot_1", "start": (-10.0, 2.0), "goal": (10.0, 2.0), "priority": 0.8},
                {"id": "robot_2", "start": (10.0, 2.0), "goal": (-10.0, 2.0), "priority": 0.5},
                {"id": "robot_3", "start": (0.0, 6.0), "goal": (0.0, -6.0), "priority": 0.7},
            ]
        },
        {
            "name": "Battery_Aware_Continuous_Tasks",
            "agents": [
                {"id": "robot_1", "start": (-10.0, 0.0), "goal": (8.0, 6.0), "priority": 0.85},
                {"id": "robot_2", "start": (10.0, 0.0), "goal": (-8.0, -6.0), "priority": 0.45},
                {"id": "robot_3", "start": (0.0, 8.0), "goal": (0.0, -8.0), "priority": 0.65},
                {"id": "robot_4", "start": (-8.0, -6.0), "goal": (8.0, -6.0), "priority": 0.55},
                {"id": "robot_5", "start": (8.0, 6.0), "goal": (-10.0, 0.0), "priority": 0.30},
            ]
        }
    ]

    results_dir = os.path.join(project_root, 'results')
    os.makedirs(results_dir, exist_ok=True)
    csv_file = os.path.join(results_dir, 'benchmark_stop_and_wait.csv')

    print("=================================================================")
    print("RUNNING BENCHMARK: STOP-AND-WAIT BASELINE")
    print("=================================================================")

    results = []
    for sc in scenarios:
        res = run_scenario(sc["name"], sc["agents"])
        results.append(res)
        print(f"Scenario: {res['scenario']}")
        print(f"  Total Time: {res['total_completion_time_s']}s | Avg Agent Time: {res['avg_agent_time_s']}s | Collisions: {res['collisions']}")

    with open(csv_file, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=results[0].keys())
        writer.writeheader()
        writer.writerows(results)

    print(f"\nBaseline results written to: {csv_file}")


if __name__ == '__main__':
    main()
