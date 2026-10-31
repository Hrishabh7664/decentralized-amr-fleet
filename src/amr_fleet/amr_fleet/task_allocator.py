#!/usr/bin/env python3
"""Compatibility wrapper for amr_fleet.planning.task_allocator."""
from amr_fleet.planning.task_allocator import (
    WarehouseTask,
    TaskAllocator,
    TaskAllocatorNode,
    main,
)

__all__ = [
    'WarehouseTask',
    'TaskAllocator',
    'TaskAllocatorNode',
    'main',
]

if __name__ == '__main__':
    main()
