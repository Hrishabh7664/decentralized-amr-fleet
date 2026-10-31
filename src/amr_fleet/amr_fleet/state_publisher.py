#!/usr/bin/env python3
"""Compatibility wrapper for amr_fleet.perception.state_publisher."""
from amr_fleet.perception.state_publisher import (
    StatePublisherNode,
    main,
)

__all__ = [
    'StatePublisherNode',
    'main',
]

if __name__ == '__main__':
    main()
