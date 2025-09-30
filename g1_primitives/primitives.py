"""
G1 Primitives - Core implementation of motion primitives for Unitree G1 robot.

This module contains the main G1Primitives class that provides high-level
motion primitives for the Unitree G1 humanoid robot.
"""

import time
import numpy as np
from typing import Dict, Any, Optional, Union, List
from multiprocessing import Array

# Try to import pinocchio, but make it optional
try:
    import pinocchio as pin
    HAS_PINOCCHIO = True
except ImportError:
    HAS_PINOCCHIO = False
    # Create a simple mock for SE3 and Quaternion if pinocchio is not available
    class MockSE3:
        def __init__(self, rotation=None, translation=None):
            self.rotation = rotation if rotation is not None else np.eye(3)
            self.translation = translation if translation is not None else np.zeros(3)
        
        def homogeneous(self):
            H = np.eye(4)
            H[:3, :3] = self.rotation
            H[:3, 3] = self.translation
            return H
    
    class MockQuaternion:
        def __init__(self, w, x, y, z):
            self.w, self.x, self.y, self.z = w, x, y, z
            # Create rotation matrix from quaternion
            norm = np.sqrt(w*w + x*x + y*y + z*z)
            w, x, y, z = w/norm, x/norm, y/norm, z/norm
            self.rotation = np.array([
                [1-2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
                [2*(x*y + w*z), 1-2*(x*x + z*z), 2*(y*z - w*x)],
                [2*(x*z - w*y), 2*(y*z + w*x), 1-2*(x*x + y*y)]
            ])
    
    # Create aliases
    pin = type('MockPin', (), {'SE3': MockSE3, 'Quaternion': MockQuaternion})()

from .types import MotionType, Motion


class G1Primitives:
    """
    High-level motion primitives for Unitree G1 humanoid robot.
    
    This class provides clean, reusable motion primitives that abstract away
    the complexity of low-level robot control.
    
    Args:
        arm_controller: G1_29_ArmController instance for arm control
        hand_controller: Dex3_1_Controller instance for hand control  
        ik_solver: G1_29_ArmIK instance for inverse kinematics
        left_hand_array: Multiprocessing Array for left hand joint positions
        right_hand_array: Multiprocessing Array for right hand joint positions
        wrist_positions: Dict with current left/right wrist SE3 poses (pinocchio.SE3 or compatible)
        last_hand_sol_tauff: Last computed joint torques (14 elements)
        hand_state: Dict tracking open/closed state for each hand
    """
    
    def __init__(self, 
                 arm_controller,
                 hand_controller,
                 ik_solver,
                 left_hand_array: Array,
                 right_hand_array: Array,
                 wrist_positions: Dict[str, pin.SE3],
                 last_hand_sol_tauff: np.ndarray,
                 hand_state: Dict[str, str]):
        """
        Initialize the G1 primitives controller.
        
        Args:
            arm_controller: G1_29_ArmController instance
            hand_controller: Dex3_1_Controller instance
            ik_solver: G1_29_ArmIK instance
            left_hand_array: Multiprocessing Array for left hand (7 elements)
            right_hand_array: Multiprocessing Array for right hand (7 elements)
            wrist_positions: Dict with 'left' and 'right' SE3 poses
            last_hand_sol_tauff: Array of last computed torques (14 elements)
            hand_state: Dict with 'open'/'closed' state for each hand
        """
        self.arm_controller = arm_controller
        self.hand_controller = hand_controller
        self.ik = ik_solver
        self.left_hand_array = left_hand_array
        self.right_hand_array = right_hand_array
        self.wrist_positions = wrist_positions
        self.last_hand_sol_tauff = last_hand_sol_tauff
        self.hand_state = hand_state
        
        # Hand joint configurations
        self.closed_pos = {
            'left': np.array([0.0, 0.3, 1, -0.7, -1., -0.7, -1]),
            'right': np.array([0.0, -0.3, -1, 0.8, 1., 0.8, 1])
        }
        self.open_pos = {
            'left': np.zeros(7),
            'right': np.zeros(7)
        }
        
        # Pre-grasp configurations for smooth closing
        self.pre_close_pos = {
            'left': np.array([0.0, 0.5, 0, -1.5, 0, -1.5, 0]),
            'right': np.array([0.0, -0.5, 0, 1.5, 0, 1.5, 0])
        }
        
        # Pre-grasp arm configuration storage
        self.pre_grasp_q = {"left": None, "right": None}
    
    def _solve_ik_iterative(self, left_wrist, right_wrist, current_lr_arm_motor_q=None, current_lr_arm_motor_dq=None, iters=5):
        """Solve inverse kinematics iteratively for better convergence."""
        for _ in range(iters):
            sol_q, sol_tauff = self.ik.solve_ik(left_wrist, right_wrist, current_lr_arm_motor_q, current_lr_arm_motor_dq)
            current_lr_arm_motor_q = sol_q
        return sol_q, sol_tauff
    
    def _send_commands_to_arms(self, target_q, sol_tauff, tolerance=1e-2):
        """Send commands to arms until target position is reached."""
        while True:
            current_q = self.arm_controller.get_current_dual_arm_q()
            if np.linalg.norm(current_q - target_q) > tolerance:
                self.arm_controller.ctrl_dual_arm(target_q, sol_tauff)
            else:
                break
    
    def grab_smooth(self, hand: str = 'right', duration: float = 2.0, verbose: bool = False):
        """
        Smoothly close the specified hand with interpolated joint positions.
        
        Args:
            hand: 'left' or 'right' hand to close
            duration: Duration of the closing motion in seconds
            verbose: Whether to print progress information
            
        Returns:
            bool: True when motion is complete
        """
        if verbose:
            print(f"[GRAB_SMOOTH] Starting smooth grab with {hand} hand for {duration}s")
        
        closed_pos = self.closed_pos[hand]
        rate_hz = 50
        n_steps = int(duration * rate_hz)
        period = 1.0 / rate_hz
        
        # Save pre-grasp q for potential use
        if hand == 'left':
            self.pre_grasp_q['left'] = self.arm_controller.get_current_dual_arm_q()
        else:
            self.pre_grasp_q['right'] = self.arm_controller.get_current_dual_arm_q()
        
        # Get current hand position
        if hand == 'left':
            arr = self.left_hand_array
        else:
            arr = self.right_hand_array

        start_pos = np.array(arr[:]).copy()
        traj = np.linspace(start_pos, closed_pos, n_steps)

        for i, q in enumerate(traj):
            arr[:] = q
            if verbose and i % 10 == 0:
                print(f"[GRAB_SMOOTH] Step {i}/{n_steps}, q: {q}")
            time.sleep(period)
        
        self.hand_state[hand] = 'closed'
        
        # Ensure final position is maintained
        for _ in range(10):
            with self.left_hand_array.get_lock():
                self.left_hand_array[:] = self.closed_pos['left'] if self.hand_state['left'] == 'closed' else self.open_pos['left']
            with self.right_hand_array.get_lock():
                self.right_hand_array[:] = self.closed_pos['right'] if self.hand_state['right'] == 'closed' else self.open_pos['right']
            time.sleep(0.01)
        
        if verbose:
            print(f"[GRAB_SMOOTH] Completed closing {hand} hand")
        return True
    
    def hold_position(self, duration: float = 1.0, verbose: bool = False):
        """
        Hold current arm and hand positions for specified duration.
        
        Args:
            duration: Duration to hold position in seconds
            verbose: Whether to print progress information
            
        Returns:
            bool: True when motion is complete
        """
        if verbose:
            print(f"[HOLD_POSITION] Holding position for {duration}s")
        
        arm_q = self.arm_controller.get_current_dual_arm_q()
        start_time = time.time()
        
        while time.time() - start_time < duration:
            self.arm_controller.ctrl_dual_arm(arm_q, self.last_hand_sol_tauff)
            with self.left_hand_array.get_lock():
                self.left_hand_array[:] = self.closed_pos['left'] if self.hand_state['left'] == 'closed' else self.open_pos['left']
            with self.right_hand_array.get_lock():
                self.right_hand_array[:] = self.closed_pos['right'] if self.hand_state['right'] == 'closed' else self.open_pos['right']
            time.sleep(0.01)
        
        if verbose:
            print(f"[HOLD_POSITION] Completed holding position")
        return True
    
    
    def move_and_tilt_dual_smooth(self,
                                left_hand_pos: Optional[List[float]] = None,
                                right_hand_pos: Optional[List[float]] = None,
                                left_hand_pos_delta: Optional[List[float]] = None,
                                right_hand_pos_delta: Optional[List[float]] = None,
                                left_angle_deg: float = 0.0,
                                right_angle_deg: float = 0.0,
                                duration: float = 3.0,
                                verbose: bool = False):
        """
        Smoothly move both hands to positions and tilt their wrists by given angles.
        
        Args:
            left_hand_pos: Target position [x, y, z] for left hand (overrides delta)
            right_hand_pos: Target position [x, y, z] for right hand (overrides delta)
            left_hand_pos_delta: Delta position [dx, dy, dz] for left hand
            right_hand_pos_delta: Delta position [dx, dy, dz] for right hand
            left_angle_deg: Left wrist rotation angle in degrees
            right_angle_deg: Right wrist rotation angle in degrees
            duration: Duration of the motion in seconds
            verbose: Whether to print progress information
            
        Returns:
            bool: True when motion is complete
        """
        if verbose:
            print(f"[MOVE_AND_TILT_DUAL_SMOOTH] Moving both hands with L:{left_angle_deg}° R:{right_angle_deg}° for {duration}s")
        
        # Support optional deltas: if *_delta is provided and explicit position is not,
        # add delta to current wrist position
        current_left_pos = self.wrist_positions['left'].translation
        current_right_pos = self.wrist_positions['right'].translation

        if left_hand_pos is not None:
            left_target_pos = np.array(left_hand_pos)
        else:
            left_delta = left_hand_pos_delta if left_hand_pos_delta is not None else [0.0, 0.0, 0.0]
            left_target_pos = np.array(current_left_pos) + np.array(left_delta, dtype=float)

        if right_hand_pos is not None:
            right_target_pos = np.array(right_hand_pos)
        else:
            right_delta = right_hand_pos_delta if right_hand_pos_delta is not None else [0.0, 0.0, 0.0]
            right_target_pos = np.array(current_right_pos) + np.array(right_delta, dtype=float)
        
        left_angle_rad = np.deg2rad(left_angle_deg)
        right_angle_rad = np.deg2rad(right_angle_deg)

        # Create quaternions for wrist rotations
        lch, lsh = np.cos(left_angle_rad / 2.0), np.sin(left_angle_rad / 2.0)
        rch, rsh = np.cos(right_angle_rad / 2.0), np.sin(right_angle_rad / 2.0)

        left_quat = pin.Quaternion(lch, lsh, 0, 0)
        right_quat = pin.Quaternion(rch, rsh, 0, 0)

        L_tf_target = pin.SE3(left_quat, left_target_pos)
        R_tf_target = pin.SE3(right_quat, right_target_pos)

        current_q = self.arm_controller.get_current_dual_arm_q()
        current_dq = self.arm_controller.get_current_dual_arm_dq()
        sol_q, sol_tauff = self._solve_ik_iterative(L_tf_target.homogeneous, R_tf_target.homogeneous, current_q, current_dq)

        rate_hz = 50
        n_steps = int(duration * rate_hz)
        q_traj = np.linspace(current_q, sol_q, n_steps)
        period = 1.0 / rate_hz

        for i, q in enumerate(q_traj):
            self.arm_controller.ctrl_dual_arm(q, sol_tauff)
            with self.left_hand_array.get_lock():
                self.left_hand_array[:] = self.closed_pos['left'] if self.hand_state['left'] == 'closed' else self.open_pos['left']
            with self.right_hand_array.get_lock():
                self.right_hand_array[:] = self.closed_pos['right'] if self.hand_state['right'] == 'closed' else self.open_pos['right']
            if verbose and i % 10 == 0:
                print(f"[MOVE_AND_TILT_DUAL_SMOOTH] Step {i}/{n_steps}, q: {q}")
            time.sleep(period)

        # Final convergence
        thresh = 0.3
        self._send_commands_to_arms(sol_q, sol_tauff, thresh)

        # Update stored wrist positions
        self.wrist_positions['left'] = L_tf_target
        self.wrist_positions['right'] = R_tf_target
        self.last_hand_sol_tauff = sol_tauff
        
        if verbose:
            print(f"[MOVE_AND_TILT_DUAL_SMOOTH] Completed")
        return True
