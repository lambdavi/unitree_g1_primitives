# G1 Primitives - Installation Guide

## Quick Installation

### Standard Installation

```bash
# 1. Install pinocchio with conda
conda install pinocchio

# 2. Install the package
pip install -e .
```

### Development Installation (with testing tools)

```bash
# 1. Install pinocchio with conda
conda install pinocchio

# 2. Install with development dependencies
pip install -e .[dev]
```

## Verification

After installation, verify the package works:

```bash
# Test basic import
python -c "import g1_primitives; print('Success! Version:', g1_primitives.__version__)"

# Run the test suite
pytest tests/

# Run examples (with mock controllers)
python examples/basic_usage.py
```

## Usage

Once installed, you can use the primitives in your code:

```python
from g1_primitives import G1Primitives

# Initialize with your robot controllers
primitives = G1Primitives(
    arm_controller=your_arm_controller,
    hand_controller=your_hand_controller,
    ik_solver=your_ik_solver,
    left_hand_array=your_left_hand_array,
    right_hand_array=your_right_hand_array,
    wrist_positions=your_wrist_positions,
    last_hand_sol_tauff=your_last_hand_sol_tauff,
    tilted_status=your_tilted_status,
    hand_state=your_hand_state
)

# Use the primitives
primitives.grab_smooth(hand='left', duration=2.0)
primitives.move_and_tilt_smooth(hand='right', position=[0.3, -0.2, 0.15], angle_deg=-90.0, duration=3.0)
```

## Dependencies

The package requires:
- Python >= 3.8
- numpy >= 1.20.0
- scipy >= 1.7.0

Pinocchio is optional - the package will work with or without it (using mock implementations when needed).

## Troubleshooting

### Import Error: "No module named 'g1_primitives'"

1. Make sure you're in the correct conda environment:
   ```bash
   conda activate venice_clone
   ```

2. Verify the package is installed:
   ```bash
   pip list | grep g1
   ```

3. Try reinstalling:
   ```bash
   pip uninstall g1_primitives -y
   pip install -e .
   ```

### Pinocchio Import Warnings

The package will work without pinocchio, but you'll see import warnings. This is normal and doesn't affect functionality.

## Package Structure

```
g1_primitives/
├── g1_primitives/           # Main package directory
│   ├── __init__.py         # Package initialization
│   ├── primitives.py       # Core primitive implementations
│   └── types.py            # Type definitions
├── examples/               # Usage examples
│   └── basic_usage.py      # Basic usage examples
├── tests/                  # Test suite
│   └── test_primitives.py  # Unit tests
├── README.md               # Documentation
├── setup.py                # Setup script
├── pyproject.toml          # Modern Python packaging
├── requirements.txt        # Dependencies
├── test_simple.py          # Simple test script
└── install.py              # Installation helper
```

## Available Primitives

1. **GRAB_SMOOTH**: Smooth hand grasping motion
2. **HOLD_POSITION**: Maintain current arm and hand positions
3. **MOVE_AND_TILT_SMOOTH**: Move single hand with wrist rotation
4. **MOVE_AND_TILT_DUAL_SMOOTH**: Move both hands with independent wrist rotations

See the README.md for detailed API documentation and examples.
