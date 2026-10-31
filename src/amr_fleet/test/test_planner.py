"""
Unit Tests for A* Global Planner and Rolling Horizon Module.
Tests path optimality, obstacle clearance, rolling horizon windowing, and blockage replanning.
"""

import os
import sys
import unittest

# Ensure src/amr_fleet is on sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(project_root, 'src', 'amr_fleet'))

from amr_fleet.utils import Vector2D, world_to_grid, grid_to_world
from amr_fleet.global_planner import AStarPlanner


class TestGlobalPlanner(unittest.TestCase):
    def setUp(self):
        self.planner = AStarPlanner(
            grid_width=50,
            grid_height=50,
            resolution=0.5,
            origin_x=-12.5,
            origin_y=-12.5,
            robot_radius=0.35,
            safety_margin=0.15
        )

    def test_straight_line_path(self):
        """Test unobstructed straight line path between two free points."""
        start = Vector2D(-5.0, 0.0)
        goal = Vector2D(5.0, 0.0)
        path = self.planner.plan_path(start, goal)

        self.assertGreater(len(path), 1, "Planner should return non-empty waypoint list")
        self.assertAlmostEqual(path[0].x, start.x, places=2)
        self.assertAlmostEqual(path[0].y, start.y, places=2)
        self.assertAlmostEqual(path[-1].x, goal.x, places=2)
        self.assertAlmostEqual(path[-1].y, goal.y, places=2)

    def test_obstacle_avoidance(self):
        """Test path circumvents an obstacle placed directly between start and goal."""
        start = Vector2D(-4.0, 0.0)
        goal = Vector2D(4.0, 0.0)

        # Place a 2m x 2m obstacle right at the center (x=0, y=0)
        self.planner.set_blocked_region(-1.0, -1.0, 1.0, 1.0, blocked=True)

        path = self.planner.plan_path(start, goal)
        self.assertGreater(len(path), 1)

        # Verify no waypoint penetrates the obstacle zone
        for pt in path:
            in_obs = (-1.0 <= pt.x <= 1.0) and (-1.0 <= pt.y <= 1.0)
            self.assertFalse(in_obs, f"Waypoint {pt} penetrates obstacle!")

    def test_rolling_horizon_extraction(self):
        """Test that rolling horizon correctly truncates path to target time window."""
        full_path = [
            Vector2D(0.0, 0.0),
            Vector2D(5.0, 0.0),
            Vector2D(10.0, 0.0),
            Vector2D(15.0, 0.0)
        ]
        # At 0.5 m/s, an 8s horizon corresponds to 4.0 meters
        rolling_path = self.planner.extract_rolling_horizon(
            full_path, horizon_time_s=8.0, nominal_speed_mps=0.5
        )

        self.assertGreater(len(rolling_path), 1)
        total_dist = sum(
            rolling_path[i - 1].distance_to(rolling_path[i])
            for i in range(1, len(rolling_path))
        )
        self.assertAlmostEqual(total_dist, 4.0, delta=0.2)

    def test_blocked_aisle_replanning(self):
        """Test re-routing when an aisle is dynamically blocked."""
        start = Vector2D(-5.0, 2.0)
        goal = Vector2D(5.0, 2.0)

        initial_path = self.planner.plan_path(start, goal)
        self.assertTrue(len(initial_path) > 0)

        # Block the aisle
        self.planner.set_blocked_region(-1.0, 1.5, 1.0, 2.5, blocked=True)
        rerouted_path = self.planner.plan_path(start, goal)

        self.assertTrue(len(rerouted_path) > 0)
        # Rerouted path must detour away from y = 2.0 at x = 0.0
        for pt in rerouted_path:
            if abs(pt.x) < 0.8:
                self.assertTrue(abs(pt.y - 2.0) > 0.5, "Detour should deviate from y=2.0")


if __name__ == '__main__':
    unittest.main()
