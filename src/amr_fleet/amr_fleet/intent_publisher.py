#!/usr/bin/env python3
"""Compatibility wrapper for amr_fleet.utils.intent_publisher."""
from amr_fleet.utils.intent_publisher import (
    IntentPublisherNode,
    main,
)

__all__ = [
    'IntentPublisherNode',
    'main',
]

if __name__ == '__main__':
    main()
