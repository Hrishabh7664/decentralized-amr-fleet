#!/usr/bin/env python3
"""Compatibility wrapper for amr_fleet.control.fleet_controller."""
from amr_fleet.control.fleet_controller import (
    DEFAULT_WAYPOINTS,
    DEFAULT_SPAWN_POSES,
    FleetControllerNode,
    main,
)

__all__ = [
    'DEFAULT_WAYPOINTS',
    'DEFAULT_SPAWN_POSES',
    'FleetControllerNode',
    'main',
]

if __name__ == '__main__':
    main()
