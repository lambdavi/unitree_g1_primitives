# G1 Primitives

High-level motion primitives for the Unitree G1 humanoid robot. This package provides clean, reusable motion primitives that abstract away the complexity of low-level robot control.

## Features

- **GRAB**: Smooth hand grasping motion with interpolated joint positions
- **HOLD_POSITION**: Maintain current arm and hand positions for specified duration
- **DUAL_ARM_MOVEMENT**: Move both hands simultaneously with independent wrist rotations
  (Single-hand moves are handled by the dual API by passing only one hand's target)

## Installation

### From Source

```bash
# First, install pinocchio with conda
 conda install conda-forge::pinocchio

# Clone and install the package
git clone https://github.com/lambdavi/g1-primitives.git
cd g1-primitives
pip install -e .
```

### Development Installation

For development with testing and linting tools:

```bash
# First, install pinocchio with conda
 conda install conda-forge::pinocchio

# Then install the package with dev dependencies
pip install -e .[dev]
```

### From PyPI (when published)

```bash
# First, install pinocchio with conda
 conda install conda-forge::pinocchio

# Then install the package
pip install g1-primitives
```

## Dependencies

- Python >= 3.8
- numpy >= 1.20.0
- scipy >= 1.7.0
- pinocchio >= 2.6.0 (install with conda)

### Installing Pinocchio

Pinocchio must be installed using conda, not pip:

```bash
 conda install conda-forge::pinocchio
```


## Quick Start

```python
from g1_primitives import G1Primitives
import numpy as np
import pinocchio as pin

# Initialize your robot controllers (these are your existing controllers)
# arm_controller = G1_29_ArmController(...)
# hand_controller = Dex3_1_Controller(...)
# ik_solver = G1_29_ArmIK(...)
# left_hand_array = Array('d', 7)
# right_hand_array = Array('d', 7)

# Set up wrist positions and state tracking
wrist_positions = {
    'left': pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, 0.15, 0.1])),
    'right': pin.SE3(pin.Quaternion(1, 0, 0, 0), np.array([0.25, -0.15, 0.1]))
}
last_hand_sol_tauff = np.zeros(14)
hand_state = {'left': 'open', 'right': 'open'}

# Initialize the primitives controller
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

# Use the primitives
primitives.grab(hand='left', duration=2.0)
primitives.dual_arm_movement(
    right_hand_pos=[0.3, -0.2, 0.15],
    right_angle_deg=-90.0,
    duration=3.0
)
primitives.hold_position(duration=1.0)
```

## API Reference

### G1Primitives

The main class that provides motion primitives for the G1 robot.

#### Constructor

```python
G1Primitives(arm_controller, hand_controller, ik_solver, left_hand_array, right_hand_array, wrist_positions, last_hand_sol_tauff, hand_state)
```

**Parameters:**
- `arm_controller`: G1_29_ArmController instance for arm control
- `hand_controller`: Dex3_1_Controller instance for hand control
- `ik_solver`: G1_29_ArmIK instance for inverse kinematics
- `left_hand_array`: Multiprocessing Array for left hand joint positions (7 elements)
- `right_hand_array`: Multiprocessing Array for right hand joint positions (7 elements)
- `wrist_positions`: Dict with current left/right wrist SE3 poses
- `last_hand_sol_tauff`: Last computed joint torques (14 elements)
- `hand_state`: Dict tracking open/closed state for each hand

#### Methods

##### grab(hand='right', duration=2.0, verbose=False)

Smoothly close the specified hand with interpolated joint positions.

**Parameters:**
- `hand` (str): 'left' or 'right' hand to close
- `duration` (float): Duration of the closing motion in seconds
- `verbose` (bool): Whether to print progress information

**Returns:**
- `bool`: True when motion is complete

##### hold_position(duration=1.0, verbose=False)

Hold current arm and hand positions for specified duration.

**Parameters:**
- `duration` (float): Duration to hold position in seconds
- `verbose` (bool): Whether to print progress information

**Returns:**
- `bool`: True when motion is complete

##### dual_arm_movement(left_hand_pos=None, right_hand_pos=None, left_hand_pos_delta=None, right_hand_pos_delta=None, left_angle_deg=0.0, right_angle_deg=0.0, duration=3.0, verbose=False)

Smoothly move both hands to positions and tilt their wrists by given angles.

**Parameters:**
- `left_hand_pos` (List[float], optional): Target position [x, y, z] for left hand (overrides delta)
- `right_hand_pos` (List[float], optional): Target position [x, y, z] for right hand (overrides delta)
- `left_hand_pos_delta` (List[float], optional): Delta position [dx, dy, dz] for left hand
- `right_hand_pos_delta` (List[float], optional): Delta position [dx, dy, dz] for right hand
- `left_angle_deg` (float): Left wrist rotation angle in degrees
- `right_angle_deg` (float): Right wrist rotation angle in degrees
- `duration` (float): Duration of the motion in seconds
- `verbose` (bool): Whether to print progress information

**Returns:**
- `bool`: True when motion is complete

## Examples

### Basic Grasping Sequence

```python
# Grasp with left hand
primitives.grab(hand='left', duration=2.0)

# Move to pouring position (single-hand via dual API)
primitives.dual_arm_movement(
    left_hand_pos=[0.3, 0.1, 0.2],
    left_angle_deg=90.0,
    duration=3.0
)

# Hold position while pouring
primitives.hold_position(duration=5.0)
```

### Dual-Arm Coordination

```python
# Move both hands simultaneously
primitives.dual_arm_movement(
    left_hand_pos=[0.3, 0.15, 0.2],
    right_hand_pos=[0.3, -0.15, 0.2],
    left_angle_deg=45.0,
    right_angle_deg=-45.0,
    duration=4.0
)

# Grasp with both hands
primitives.grab(hand='left', duration=1.5)
primitives.grab(hand='right', duration=1.5)
```

### Delta Movements

```python
# Move relative to current position
primitives.dual_arm_movement(
    left_hand_pos_delta=[0.05, 0.0, -0.02],
    right_hand_pos_delta=[0.05, 0.0, -0.02],
    left_angle_deg=0.0,
    right_angle_deg=0.0,
    duration=2.0
)
```