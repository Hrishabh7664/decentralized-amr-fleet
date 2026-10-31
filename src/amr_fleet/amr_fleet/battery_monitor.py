#!/usr/bin/env python3
"""Compatibility wrapper for amr_fleet.perception.battery_monitor."""
from amr_fleet.perception.battery_monitor import (
    ChargingStation,
    BatteryMonitor,
    BatteryMonitorNode,
    main,
)

__all__ = [
    'ChargingStation',
    'BatteryMonitor',
    'BatteryMonitorNode',
    'main',
]

if __name__ == '__main__':
    main()
