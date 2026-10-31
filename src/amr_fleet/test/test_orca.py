"""
Unit Tests for 2D Optimal Reciprocal Collision Avoidance (ORCA).
Tests half-plane generation, 2D linear programming, static obstacle constraints,
and fallback safe-stop behaviors.
"""

import os
import sys
import unittest
import math

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(project_root, 'src', 'amr_fleet'))

from amr_fleet.utils import Vector2D
from amr_fleet.local_planner_orca import ORCAPlanner, AgentState, Line2D


class TestORCALocalPlanner(unittest.TestCase):
    def setUp(self):
        self.orca = ORCAPlanner(
            time_step=0.05,
            time_horizon=2.5,
            time_horizon_obst=1.0,
            robot_radius=0.35,
            max_speed=0.8,
            safety_margin=0.05
        )

    def test_orca_halfplane_head_on(self):
        """Test reciprocal half-plane for two agents moving directly toward each other."""
        pos_a = Vector2D(-1.0, 0.0)
        vel_a = Vector2D(0.5, 0.0)
        pos_b = Vector2D(1.0, 0.0)
        vel_b = Vector2D(-0.5, 0.0)

        line = self.orca.compute_orca_halfplane(
            pos_a, vel_a, pos_b, vel_b, radius_b=0.40
        )

        self.assertIsInstance(line, Line2D)
        # Displacement vector should push velocity off the X axis
        self.assertNotEqual(line.direction.norm(), 0.0)

    def test_orca_halfplane_diverging(self):
        """Test half-plane when agents are moving away from each other."""
        pos_a = Vector2D(-1.0, 0.0)
        vel_a = Vector2D(-0.5, 0.0)
        pos_b = Vector2D(1.0, 0.0)
        vel_b = Vector2D(0.5, 0.0)

        pref_vel = Vector2D(-0.5, 0.0)
        neighbors = [AgentState(id="b", position=pos_b, velocity=vel_b, radius=0.40)]

        new_vel = self.orca.compute_velocity(pos_a, vel_a, pref_vel, neighbors)
        # Should maintain preferred diverging velocity without restriction
        self.assertAlmostEqual(new_vel.x, pref_vel.x, delta=0.05)
        self.assertAlmostEqual(new_vel.y, pref_vel.y, delta=0.05)

    def test_static_obstacle_avoidance(self):
        """Test 100% responsibility static obstacle constraint."""
        pos_a = Vector2D(0.0, 0.0)
        vel_a = Vector2D(0.6, 0.0)
        pref_vel = Vector2D(0.6, 0.0)
        obs_pos = Vector2D(0.8, 0.0)  # Obstacle directly ahead

        new_vel = self.orca.compute_velocity(
            pos_a, vel_a, pref_vel, neighbors=[], static_obstacles=[obs_pos]
        )

        # Output velocity must deflect away from (0.6, 0.0) or stop
        self.assertTrue(new_vel.x < 0.6 or abs(new_vel.y) > 0.05)

    def test_linear_program_speed_limit(self):
        """Test that linear program strictly respects max_speed limit."""
        pos = Vector2D(0.0, 0.0)
        vel = Vector2D(0.0, 0.0)
        pref_vel = Vector2D(2.0, 2.0)  # Far exceeds max_speed = 0.8

        new_vel = self.orca.compute_velocity(pos, vel, pref_vel, neighbors=[])
        self.assertLessEqual(new_vel.norm(), self.orca.max_speed + 1e-5)

    def test_safe_stop_fallback(self):
        """Test fallback when constraints are mutually contradictory."""
        # Create mutually contradictory half-planes that leave no feasible velocity
        lines = [
            Line2D(point=Vector2D(0.5, 0.0), direction=Vector2D(0.0, 1.0)),
            Line2D(point=Vector2D(-0.5, 0.0), direction=Vector2D(0.0, -1.0))
        ]
        fallback_vel = self.orca._fallback_safe_stop(lines, Vector2D(0.5, 0.0))
        self.assertIsInstance(fallback_vel, Vector2D)
        self.assertLessEqual(fallback_vel.norm(), self.orca.max_speed)


if __name__ == '__main__':
    unittest.main()
