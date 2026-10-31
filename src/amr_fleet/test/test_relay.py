"""
Unit Tests for Multi-Hop MessageRelayCache.
Tests message deduplication, TTL expiration, cache pruning, and storm suppression.
"""

import os
import sys
import unittest
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(current_dir)
sys.path.insert(0, os.path.join(project_root, 'src', 'amr_fleet'))

from amr_fleet.utils import MessageRelayCache


class TestMessageRelayCache(unittest.TestCase):
    def setUp(self):
        self.cache = MessageRelayCache(ttl_seconds=1.0, max_cache_size=5)

    def test_first_message_allowed(self):
        """Unseen message should be allowed for forwarding."""
        self.assertTrue(self.cache.should_forward("msg_001", current_time=100.0))

    def test_duplicate_message_suppressed(self):
        """Identical message ID within TTL must be dropped to prevent broadcast storms."""
        self.cache.should_forward("msg_001", current_time=100.0)
        self.assertFalse(self.cache.should_forward("msg_001", current_time=100.2))
        self.assertFalse(self.cache.should_forward("msg_001", current_time=100.8))

    def test_ttl_expiration(self):
        """Message should be re-allowed after TTL expiration."""
        self.cache.should_forward("msg_001", current_time=100.0)
        # After TTL = 1.0s, at t=101.5s, message can be processed again
        self.assertTrue(self.cache.should_forward("msg_001", current_time=101.5))

    def test_cache_capacity_pruning(self):
        """Cache should drop oldest entry when exceeding max_cache_size."""
        t = 100.0
        for i in range(5):
            self.cache.should_forward(f"msg_{i}", current_time=t + i * 0.01)

        # 6th message causes pruning of msg_0
        self.cache.should_forward("msg_5", current_time=t + 0.1)
        self.assertNotIn("msg_0", self.cache._seen_messages)


if __name__ == '__main__':
    unittest.main()
