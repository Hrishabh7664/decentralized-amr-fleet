"""
Decentralized AMR Fleet Coordination, Planning, and Control Stack.
Provides modular subpackages: control, planning, perception, and utils.
"""

from . import control
from . import planning
from . import perception
from . import utils

__version__ = "1.0.0"

__all__ = [
    'control',
    'planning',
    'perception',
    'utils',
]
