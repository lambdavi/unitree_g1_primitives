#!/usr/bin/env python3
"""
Tests for G1 Primitives package.
"""

import pytest
import numpy as np
import pinocchio as pin
from multiprocessing import Array
from unittest.mock import Mock, patch

from g1_primitives import G1Primitives, MotionType


class TestG1Primitives:
    """Test cases for G1Primitives class."""
    
    def setup_method(self):
        """Set up test fixtures before each test method."""
        # Create mock controllers
        self.mock_arm_controller = Mock()
        self.mock_hand_controller = Mock()
        self.mock_ik_solver = Mock()
        
        # Create hand arrays
        self.left_hand_array = Array('d', 7)
        self.right_hand_array = Array('d', 7)
        
        # Initialize arrays to zero
        self.left_hand_array[:] = np.zeros(7)
        self.right_hand_array[:] = np.zeros(7)
        
        # Set up wrist positions
        self.wrist_positions = {
            'left': pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, 0.15, 0.1])),
            'right': pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, -0.15, 0.1]))
        }
        
        # Initialize state tracking
        self.last_hand_sol_tauff = np.zeros(14)
        self.tilted_status = {'left': [False, 0.0], 'right': [False, 0.0]}
        self.hand_state = {'left': 'open', 'right': 'open'}
        
        # Mock IK solver behavior
        self.mock_ik_solver.solve_ik.return_value = (np.zeros(14), np.zeros(14))
        self.mock_arm_controller.get_current_dual_arm_q.return_value = np.zeros(14)
        
        # Create primitives instance
        self.primitives = G1Primitives(
            arm_controller=self.mock_arm_controller,
            hand_controller=self.mock_hand_controller,
            ik_solver=self.mock_ik_solver,
            left_hand_array=self.left_hand_array,
            right_hand_array=self.right_hand_array,
            wrist_positions=self.wrist_positions,
            last_hand_sol_tauff=self.last_hand_sol_tauff,
            tilted_status=self.tilted_status,
            hand_state=self.hand_state
        )
    
    def test_initialization(self):
        """Test that G1Primitives initializes correctly."""
        assert self.primitives.arm_controller == self.mock_arm_controller
        assert self.primitives.hand_controller == self.mock_hand_controller
        assert self.primitives.ik == self.mock_ik_solver
        assert self.primitives.left_hand_array == self.left_hand_array
        assert self.primitives.right_hand_array == self.right_hand_array
        assert self.primitives.wrist_positions == self.wrist_positions
        assert self.primitives.hand_state == self.hand_state
    
    def test_hand_configurations(self):
        """Test that hand configurations are set correctly."""
        # Test closed positions
        assert self.primitives.closed_pos['left'].shape == (7,)
        assert self.primitives.closed_pos['right'].shape == (7,)
        
        # Test open positions
        assert np.allclose(self.primitives.open_pos['left'], np.zeros(7))
        assert np.allclose(self.primitives.open_pos['right'], np.zeros(7))
        
        # Test pre-close positions
        assert self.primitives.pre_close_pos['left'].shape == (7,)
        assert self.primitives.pre_close_pos['right'].shape == (7,)
    
    @patch('time.sleep')
    @patch('time.time')
    def test_grab_smooth_left_hand(self, mock_time, mock_sleep):
        """Test smooth grabbing with left hand."""
        # Mock time.time to return increasing values
        mock_time.side_effect = [0.0, 0.02, 0.04, 0.06, 0.08, 0.1]
        
        result = self.primitives.grab_smooth(hand='left', duration=0.1, verbose=False)
        
        assert result is True
        assert self.primitives.hand_state['left'] == 'closed'
        
        # Verify that arm controller was called
        self.mock_arm_controller.get_current_dual_arm_q.assert_called()
    
    @patch('time.sleep')
    @patch('time.time')
    def test_grab_smooth_right_hand(self, mock_time, mock_sleep):
        """Test smooth grabbing with right hand."""
        # Mock time.time to return increasing values
        mock_time.side_effect = [0.0, 0.02, 0.04, 0.06, 0.08, 0.1]
        
        result = self.primitives.grab_smooth(hand='right', duration=0.1, verbose=False)
        
        assert result is True
        assert self.primitives.hand_state['right'] == 'closed'
    
    @patch('time.sleep')
    @patch('time.time')
    def test_hold_position(self, mock_time, mock_sleep):
        """Test holding position."""
        # Mock time.time to return increasing values
        mock_time.side_effect = [0.0, 0.01, 0.02, 0.03, 0.04, 0.05]
        
        result = self.primitives.hold_position(duration=0.05, verbose=False)
        
        assert result is True
        self.mock_arm_controller.ctrl_dual_arm.assert_called()
    
    @patch('time.sleep')
    @patch('time.time')
    def test_move_and_tilt_smooth_left_hand(self, mock_time, mock_sleep):
        """Test move and tilt with left hand."""
        # Mock time.time to return increasing values
        mock_time.side_effect = [0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12]
        
        target_pos = [0.3, 0.2, 0.15]
        angle_deg = 45.0
        
        result = self.primitives.move_and_tilt_smooth(
            hand='left',
            position=target_pos,
            angle_deg=angle_deg,
            duration=0.1,
            verbose=False
        )
        
        assert result is True
        assert self.primitives.tilted_status['left'][0] is True  # Should be tilted
        assert abs(self.primitives.tilted_status['left'][1] - np.deg2rad(angle_deg)) < 1e-6
        
        # Verify IK solver was called
        self.mock_ik_solver.solve_ik.assert_called()
    
    @patch('time.sleep')
    @patch('time.time')
    def test_move_and_tilt_smooth_right_hand(self, mock_time, mock_sleep):
        """Test move and tilt with right hand."""
        # Mock time.time to return increasing values
        mock_time.side_effect = [0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12]
        
        target_pos = [0.3, -0.2, 0.15]
        angle_deg = -45.0
        
        result = self.primitives.move_and_tilt_smooth(
            hand='right',
            position=target_pos,
            angle_deg=angle_deg,
            duration=0.1,
            verbose=False
        )
        
        assert result is True
        assert self.primitives.tilted_status['right'][0] is True  # Should be tilted
        assert abs(self.primitives.tilted_status['right'][1] - np.deg2rad(angle_deg)) < 1e-6
    
    @patch('time.sleep')
    @patch('time.time')
    def test_move_and_tilt_dual_smooth(self, mock_time, mock_sleep):
        """Test dual arm move and tilt."""
        # Mock time.time to return increasing values
        mock_time.side_effect = [0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12]
        
        left_pos = [0.3, 0.2, 0.15]
        right_pos = [0.3, -0.2, 0.15]
        left_angle = 30.0
        right_angle = -30.0
        
        result = self.primitives.move_and_tilt_dual_smooth(
            left_hand_pos=left_pos,
            right_hand_pos=right_pos,
            left_angle_deg=left_angle,
            right_angle_deg=right_angle,
            duration=0.1,
            verbose=False
        )
        
        assert result is True
        # Verify IK solver was called
        self.mock_ik_solver.solve_ik.assert_called()
    
    @patch('time.sleep')
    @patch('time.time')
    def test_move_and_tilt_dual_smooth_with_deltas(self, mock_time, mock_sleep):
        """Test dual arm move and tilt with position deltas."""
        # Mock time.time to return increasing values
        mock_time.side_effect = [0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.12]
        
        left_delta = [0.05, 0.0, -0.02]
        right_delta = [0.05, 0.0, -0.02]
        
        result = self.primitives.move_and_tilt_dual_smooth(
            left_hand_pos_delta=left_delta,
            right_hand_pos_delta=right_delta,
            left_angle_deg=0.0,
            right_angle_deg=0.0,
            duration=0.1,
            verbose=False
        )
        
        assert result is True
        # Verify IK solver was called
        self.mock_ik_solver.solve_ik.assert_called()
    
    def test_solve_ik_iterative(self):
        """Test iterative IK solving."""
        left_wrist = np.eye(4)
        right_wrist = np.eye(4)
        current_q = np.zeros(14)
        current_dq = np.zeros(14)
        
        sol_q, sol_tauff = self.primitives._solve_ik_iterative(
            left_wrist, right_wrist, current_q, current_dq, iters=3
        )
        
        # Should have called solve_ik 3 times
        assert self.mock_ik_solver.solve_ik.call_count == 3
        assert sol_q is not None
        assert sol_tauff is not None
    
    def test_send_commands_to_arms_convergence(self):
        """Test arm command sending with convergence."""
        # Mock get_current_dual_arm_q to return different values initially, then converge
        self.mock_arm_controller.get_current_dual_arm_q.side_effect = [
            np.array([0.1, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.array([0.01, 0.01, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
            np.zeros(14)  # Converged
        ]
        
        target_q = np.zeros(14)
        sol_tauff = np.zeros(14)
        
        # This should not raise an exception and should converge
        self.primitives._send_commands_to_arms(target_q, sol_tauff, tolerance=0.05)
        
        # Should have called ctrl_dual_arm at least once
        assert self.mock_arm_controller.ctrl_dual_arm.call_count >= 1


class TestMotionType:
    """Test cases for MotionType enum."""
    
    def test_motion_type_values(self):
        """Test that MotionType enum has expected values."""
        assert MotionType.GRAB_SMOOTH is not None
        assert MotionType.HOLD_POSITION is not None
        assert MotionType.MOVE_AND_TILT_SMOOTH is not None
        assert MotionType.MOVE_AND_TILT_DUAL_SMOOTH is not None


if __name__ == "__main__":
    pytest.main([__file__])
