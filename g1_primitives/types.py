"""
Type definitions for G1 Primitives package.
"""

from enum import Enum, auto
from dataclasses import dataclass
from typing import Dict, Any


class MotionType(Enum):
    """Enumeration of available motion primitive types."""
    GRAB = auto()
    RELEASE = auto()
    HOLD_POSITION = auto()
    DUAL_ARM_MOVEMENT = auto()


@dataclass
class Motion:
    """Data class representing a motion primitive with its parameters."""
    type: MotionType
    params: Dict[str, Any]
    duration: float = 5.0
