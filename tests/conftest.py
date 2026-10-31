"""
Pytest configuration and global fixtures.
Installs ROS mocks into sys.modules so that all tests and imported modules can locate rclpy.
"""

import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(project_root, 'src', 'amr_fleet'))
sys.path.insert(0, current_dir)

from mock_ros import install_ros_mocks
install_ros_mocks()
