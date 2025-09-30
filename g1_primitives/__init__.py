"""
G1 Primitives - High-level motion primitives for Unitree G1 humanoid robot

This package provides clean, reusable motion primitives for the Unitree G1 robot:
- GRAB_SMOOTH: Smooth hand grasping motion
- HOLD_POSITION: Maintain current arm and hand positions
- MOVE_AND_TILT_SMOOTH: Move single hand with wrist rotation
- MOVE_AND_TILT_DUAL_SMOOTH: Move both hands with independent wrist rotations

Usage:
    from g1_primitives import G1Primitives
    
    primitives = G1Primitives(arm_controller, hand_controller, ik_solver, hand_arrays)
    primitives.grab_smooth(hand='left', duration=2.0)
    primitives.move_and_tilt_smooth(hand='right', position=[0.3, -0.2, 0.15], angle_deg=-90.0, duration=3.0)
"""

from .primitives import G1Primitives
from .types import MotionType

__version__ = "1.0.0"
__author__ = "Davide Buoso"
__email__ = "davide.buoso@polito.it"

__all__ = ['G1Primitives', 'MotionType']
