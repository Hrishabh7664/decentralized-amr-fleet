# NOTE: Native ROS 2 runtime (rclpy C-extensions / daemon) is unavailable in this Windows host environment; using MockNode/MockMessageBus harness for in-process node regression testing.
"""
Regression tests for Section D: fleet_controller async spawning and lifecycle management.
D1: Await spawn_client future before registering robot entity.
D2: Guard subprocess.Popen for single-robot control stack with try/except.
D3: Await delete_client future before removing robot entity from fleet registry.
"""

import os
import sys
import unittest
from unittest.mock import patch

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(project_root, 'src', 'amr_fleet'))
sys.path.insert(0, current_dir)

from mock_ros import (
    install_ros_mocks,
    String,
)

install_ros_mocks()

from amr_fleet.control.fleet_controller import FleetControllerNode


class TestSectionD(unittest.TestCase):
    def setUp(self):
        # Instantiate FleetControllerNode
        # Patch subprocess.run for xacro so it doesn't execute real binary
        with patch("subprocess.run") as mock_run:
            mock_run.return_value.stdout = "<robot name='mock'></robot>"
            self.fc = FleetControllerNode()

    def test_d1_spawn_client_failure_does_not_register_robot(self):
        """
        [D1 Regression Test]
        Verify that add_robot_callback awaits the spawn_client future:
        - When the spawn service call fails (success=False), the robot MUST NOT be registered
          in active_robots or dispatched.
        """
        self.fc.spawn_client.should_fail = True
        initially_active = list(self.fc.active_robots)
        self.assertNotIn("robot_4", initially_active)

        with patch("subprocess.run") as mock_run, patch("subprocess.Popen") as mock_popen:
            mock_run.return_value.stdout = "<robot name='robot_4'></robot>"
            self.fc.add_robot_callback(String(data="robot_4"))

            # In un-fixed code, call_async is un-awaited and robot_4 is registered anyway!
            self.assertNotIn(
                "robot_4", self.fc.active_robots,
                "robot_4 was registered in active_robots even though Gazebo spawn service failed!"
            )
            mock_popen.assert_not_called()

    def test_d2_broken_subprocess_launch_is_guarded(self):
        """
        [D2 Regression Test]
        Verify that subprocess.Popen failure during single-robot stack launch is guarded:
        - Handled cleanly via try/except without crashing the node.
        - Unrolls the active robot registration so the fleet state remains consistent.
        """
        self.fc.spawn_client.should_fail = False

        with patch("subprocess.run") as mock_run, \
             patch("subprocess.Popen", side_effect=OSError("Deliberately broken binary not found")):
            mock_run.return_value.stdout = "<robot name='robot_5'></robot>"

            # Should not raise uncaught exception
            try:
                self.fc.add_robot_callback(String(data="robot_5"))
            except OSError:
                self.fail("subprocess.Popen failure was unguarded and raised an uncaught OSError!")

            # Robot registration should be rolled back
            self.assertNotIn("robot_5", self.fc.active_robots,
                             "Failed subprocess spawn did not roll back robot registration!")

    def test_d3_delete_client_failure_does_not_unregister_robot(self):
        """
        [D3 Regression Test]
        Verify that remove_robot_callback awaits the delete_client future:
        - When delete_entity fails, the robot entity remains registered in active_robots.
        """
        self.fc.delete_client.should_fail = True
        self.assertIn("robot_3", self.fc.active_robots)

        self.fc.remove_robot_callback(String(data="robot_3"))

        # In un-fixed code, robot_3 is popped from active_robots unconditionally
        self.assertIn(
            "robot_3", self.fc.active_robots,
            "robot_3 was removed from active_robots even though Gazebo delete service failed!"
        )


if __name__ == '__main__':
    unittest.main()
