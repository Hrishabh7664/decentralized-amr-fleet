#!/usr/bin/env python3
"""Compatibility wrapper for amr_fleet.planning.global_planner."""
from amr_fleet.planning.global_planner import (
    AStarPlanner,
    GlobalPlannerNode,
    main,
)

__all__ = [
    'AStarPlanner',
    'GlobalPlannerNode',
    'main',
]

if __name__ == '__main__':
    main()
