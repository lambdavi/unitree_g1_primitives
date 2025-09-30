#!/usr/bin/env python3
"""
Basic usage example for G1 Primitives package.

This example shows how to use the G1 primitives for common robot tasks.
Note: This is a template - you'll need to integrate with your actual robot setup.

IMPORTANT: Make sure the package is installed before running this example:
    pip install -e .
"""

import numpy as np
import pinocchio as pin
from multiprocessing import Array
import time

# Import your robot controllers (these would be your actual imports)
# from robot_controllers.robot_arm_ik import G1_29_ArmIK
# from robot_controllers.robot_arm_close import G1_29_ArmController
# from robot_controllers.robot_hand_unitree import Dex3_1_Controller

from g1_primitives import G1Primitives


def setup_robot_controllers():
    """
    Set up robot controllers and state variables.
    This is a template - replace with your actual initialization code.
    """
    # Initialize your controllers here
    # arm_controller = G1_29_ArmController(network_interface="lo", domain_id=1)
    # ik_solver = G1_29_ArmIK(Unit_Test=False, Visualization=True)
    
    # Initialize hand control arrays
    left_hand_array = Array('d', 7)
    right_hand_array = Array('d', 7)
    
    # Initialize hand controller (this would be your actual initialization)
    # hand_controller = Dex3_1_Controller(
    #     left_hand_array=left_hand_array,
    #     right_hand_array=right_hand_array,
    #     # ... other parameters
    # )
    
    # Set up wrist positions
    wrist_positions = {
        'left': pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, 0.15, 0.1])),
        'right': pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, -0.15, 0.1]))
    }
    
    # Initialize state tracking
    last_hand_sol_tauff = np.zeros(14)
    hand_state = {'left': 'open', 'right': 'open'}
    
    # For this example, we'll create mock objects
    class MockController:
        def get_current_dual_arm_q(self):
            return np.zeros(14)
        
        def get_current_dual_arm_dq(self):
            return np.zeros(14)
        
        def ctrl_dual_arm(self, q, tauff):
            print(f"Mock: Controlling arms with q={q[:3]}..., tauff={tauff[:3]}...")
    
    class MockIK:
        def solve_ik(self, left_wrist, right_wrist, current_q, current_dq):
            # Return mock solution
            return np.zeros(14), np.zeros(14)
    
    arm_controller = MockController()
    ik_solver = MockIK()
    hand_controller = None  # Mock hand controller
    
    return (arm_controller, hand_controller, ik_solver, left_hand_array, right_hand_array,
            wrist_positions, last_hand_sol_tauff, hand_state)


def example_basic_grasping():
    """Example: Basic grasping sequence with left hand."""
    print("=== Basic Grasping Example ===")
    
    # Set up robot (replace with your actual setup)
    (arm_controller, hand_controller, ik_solver, left_hand_array, right_hand_array,
     wrist_positions, last_hand_sol_tauff, hand_state) = setup_robot_controllers()
    
    # Initialize primitives
    primitives = G1Primitives(
        arm_controller=arm_controller,
        hand_controller=hand_controller,
        ik_solver=ik_solver,
        left_hand_array=left_hand_array,
        right_hand_array=right_hand_array,
        wrist_positions=wrist_positions,
        last_hand_sol_tauff=last_hand_sol_tauff,
        hand_state=hand_state
    )
    
    print("1. Grasping with left hand...")
    primitives.grab(hand='left', duration=1.0, verbose=True)
    
    print("2. Holding position...")
    primitives.hold_position(duration=0.5, verbose=True)
    
    print("3. Moving to new position with tilt...")
    primitives.dual_arm_movement(
        left_hand_pos=[0.3, 0.1, 0.15],
        left_angle_deg=45.0,
        duration=1.0,
        verbose=True
    )
    
    print("Basic grasping example completed!")


def example_dual_arm_coordination():
    """Example: Dual-arm coordination."""
    print("\n=== Dual-Arm Coordination Example ===")
    
    # Set up robot (replace with your actual setup)
    (arm_controller, hand_controller, ik_solver, left_hand_array, right_hand_array,
     wrist_positions, last_hand_sol_tauff, hand_state) = setup_robot_controllers()
    
    # Initialize primitives
    primitives = G1Primitives(
        arm_controller=arm_controller,
        hand_controller=hand_controller,
        ik_solver=ik_solver,
        left_hand_array=left_hand_array,
        right_hand_array=right_hand_array,
        wrist_positions=wrist_positions,
        last_hand_sol_tauff=last_hand_sol_tauff,
        hand_state=hand_state
    )
    
    print("1. Moving both hands to initial positions...")
    primitives.dual_arm_movement(
        left_hand_pos=[0.3, 0.2, 0.15],
        right_hand_pos=[0.3, -0.2, 0.15],
        left_angle_deg=0.0,
        right_angle_deg=0.0,
        duration=1.0,
        verbose=True
    )
    
    print("2. Tilting both hands...")
    primitives.dual_arm_movement(
        left_hand_pos_delta=[0.0, 0.0, 0.0],  # No position change
        right_hand_pos_delta=[0.0, 0.0, 0.0],  # No position change
        left_angle_deg=60.0,
        right_angle_deg=-60.0,
        duration=1.0,
        verbose=True
    )
    
    print("3. Grasping with both hands...")
    primitives.grab(hand='left', duration=0.5, verbose=True)
    primitives.grab(hand='right', duration=0.5, verbose=True)
    
    print("4. Final hold...")
    primitives.hold_position(duration=0.5, verbose=True)
    
    print("Dual-arm coordination example completed!")


def example_pick_and_place():
    """Example: Pick-and-place sequence using left hand."""
    print("\n=== Pick-and-Place Example ===")
    
    # Set up robot (replace with your actual setup)
    (arm_controller, hand_controller, ik_solver, left_hand_array, right_hand_array,
     wrist_positions, last_hand_sol_tauff, hand_state) = setup_robot_controllers()
    
    # Initialize primitives
    primitives = G1Primitives(
        arm_controller=arm_controller,
        hand_controller=hand_controller,
        ik_solver=ik_solver,
        left_hand_array=left_hand_array,
        right_hand_array=right_hand_array,
        wrist_positions=wrist_positions,
        last_hand_sol_tauff=last_hand_sol_tauff,
        hand_state=hand_state
    )
    
    # Pick and place target positions (example coordinates)
    pick_pos = [0.30, 0.12, 0.05]
    place_pos = [0.40, -0.10, 0.06]
    approach_offset = 0.10  # Lift/approach offset in meters
    
    # Ensure hand starts open
    hand_state['left'] = 'open'
    
    print("1. Move above pick position (pre-grasp)...")
    primitives.dual_arm_movement(
        left_hand_pos=[pick_pos[0], pick_pos[1], pick_pos[2] + approach_offset],
        left_angle_deg=0.0,
        duration=1.0,
        verbose=True
    )
    
    print("2. Move down to pick...")
    primitives.dual_arm_movement(
        left_hand_pos=pick_pos,
        left_angle_deg=0.0,
        duration=0.8,
        verbose=True
    )
    
    print("3. Close hand to grasp...")
    primitives.grab(hand='left', duration=0.8, verbose=True)
    
    print("4. Lift object...")
    primitives.dual_arm_movement(
        left_hand_pos=[pick_pos[0], pick_pos[1], pick_pos[2] + approach_offset],
        left_angle_deg=0.0,
        duration=0.8,
        verbose=True
    )
    
    print("5. Move above place position...")
    primitives.dual_arm_movement(
        left_hand_pos=[place_pos[0], place_pos[1], place_pos[2] + approach_offset],
        left_angle_deg=0.0,
        duration=1.2,
        verbose=True
    )
    
    print("6. Move down to place...")
    primitives.dual_arm_movement(
        left_hand_pos=place_pos,
        left_angle_deg=0.0,
        duration=0.8,
        verbose=True
    )
    
    print("7. Open hand to release...")
    primitives.release(hand='left', duration=0.3, verbose=True)
    
    print("8. Retract up from place position...")
    primitives.dual_arm_movement(
        left_hand_pos=[place_pos[0], place_pos[1], place_pos[2] + approach_offset],
        left_angle_deg=0.0,
        duration=0.8,
        verbose=True
    )
    
    print("Pick-and-place sequence completed!")


if __name__ == "__main__":
    print("G1 Primitives - Basic Usage Examples")
    print("=" * 50)
    
    try:
        # Run examples
        example_basic_grasping()
        example_dual_arm_coordination()
        example_pick_and_place()
        
        print("\n" + "=" * 50)
        print("All examples completed successfully!")
        print("\nNote: These examples use mock controllers.")
        print("To use with real robot, replace setup_robot_controllers() with your actual initialization.")
        
    except Exception as e:
        print(f"Error running examples: {e}")
        print("Make sure you have all dependencies installed and robot controllers properly set up.")
