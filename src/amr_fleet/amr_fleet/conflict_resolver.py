#!/usr/bin/env python3
"""Compatibility wrapper for amr_fleet.utils.conflict_resolver."""
from amr_fleet.utils.conflict_resolver import (
    CorridorZone,
    ConflictResolver,
    ConflictResolverNode,
    main,
)

__all__ = [
    'CorridorZone',
    'ConflictResolver',
    'ConflictResolverNode',
    'main',
]

if __name__ == '__main__':
    main()
