# G1 Primitives

High-level motion primitives for the Unitree G1 humanoid robot. This package provides clean, reusable motion primitives that abstract away the complexity of low-level robot control.

## Features

- **GRAB_SMOOTH**: Smooth hand grasping motion with interpolated joint positions
- **HOLD_POSITION**: Maintain current arm and hand positions for specified duration
- **MOVE_AND_TILT_DUAL_SMOOTH**: Move both hands simultaneously with independent wrist rotations
  (Single-hand moves are handled by the dual API by passing only one hand's target)

## Installation

### From Source

```bash
# First, install pinocchio with conda
conda install pinocchio

# Clone and install the package
git clone https://github.com/davidebuoso/g1-primitives.git
cd g1-primitives
pip install -e .
```

### Development Installation

For development with testing and linting tools:

```bash
# First, install pinocchio with conda
conda install pinocchio

# Then install the package with dev dependencies
pip install -e .[dev]
```

### From PyPI (when published)

```bash
# First, install pinocchio with conda
conda install pinocchio

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
conda install pinocchio
```

Or if you need a specific version:

```bash
conda install pinocchio=2.6.0
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
primitives.grab_smooth(hand='left', duration=2.0)
primitives.move_and_tilt_dual_smooth(
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

##### grab_smooth(hand='right', duration=2.0, verbose=False)

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

##### move_and_tilt_dual_smooth(left_hand_pos=None, right_hand_pos=None, left_hand_pos_delta=None, right_hand_pos_delta=None, left_angle_deg=0.0, right_angle_deg=0.0, duration=3.0, verbose=False)

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
primitives.grab_smooth(hand='left', duration=2.0)

# Move to pouring position (single-hand via dual API)
primitives.move_and_tilt_dual_smooth(
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
primitives.move_and_tilt_dual_smooth(
    left_hand_pos=[0.3, 0.15, 0.2],
    right_hand_pos=[0.3, -0.15, 0.2],
    left_angle_deg=45.0,
    right_angle_deg=-45.0,
    duration=4.0
)

# Grasp with both hands
primitives.grab_smooth(hand='left', duration=1.5)
primitives.grab_smooth(hand='right', duration=1.5)
```

### Delta Movements

```python
# Move relative to current position
primitives.move_and_tilt_dual_smooth(
    left_hand_pos_delta=[0.05, 0.0, -0.02],
    right_hand_pos_delta=[0.05, 0.0, -0.02],
    left_angle_deg=0.0,
    right_angle_deg=0.0,
    duration=2.0
)
```

## Integration with Existing Code

This package is designed to work with your existing G1 robot control setup. You'll need to:

1. Keep your existing robot controllers (G1_29_ArmController, Dex3_1_Controller, G1_29_ArmIK)
2. Pass the necessary objects to the G1Primitives constructor
3. Maintain the wrist_positions, hand_state, and other state variables
4. Update these state variables after each primitive execution

## Development

### Running Tests

```bash
pip install -e .[dev]
pytest
```

### Code Formatting

```bash
black g1_primitives/
flake8 g1_primitives/
```

### Running Examples

```bash
# Make sure the package is installed
pip install -e .

# Run the basic usage example
python examples/basic_usage.py
```

### Easy Installation Commands

```bash
# Complete installation in 2 commands:
conda install pinocchio
pip install -e .
```

### Package Structure

```
g1_primitives/
├── g1_primitives/          # Main package
│   ├── __init__.py        # Package initialization
│   ├── primitives.py      # Core motion primitives
│   └── types.py          # Type definitions
├── examples/              # Usage examples
│   ├── __init__.py
│   └── basic_usage.py
├── tests/                 # Test suite
│   ├── __init__.py
│   └── test_primitives.py
├── pyproject.toml        # Modern Python packaging configuration
├── requirements.txt      # Dependencies
├── README.md            # This file
├── INSTALL.md           # Installation guide
└── LICENSE              # MIT License
```

## License

MIT License - see LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Support

For questions and support, please open an issue on GitHub.
