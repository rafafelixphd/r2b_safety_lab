# Manual Robot Control

Interactive CLI tool for controlling SO-101 robot motors individually.

## Features

- **Menu-driven interface** - Select motors 1-6 or reset all
- **Arrow key controls** - Use ← and → to move motors by 5%
- **Reset function** - Return all motors to zero position
- **Real-time feedback** - See current position after each movement

## Usage

### Basic Usage

```bash
python manual_control.py
```

### With Custom Port

```bash
python manual_control.py --port /dev/tty.usbmodem***
```

### With Custom Calibration

```bash
python manual_control.py --calibration /path/to/calibration.json
```

## Controls

### Main Menu

- `1` - Control Shoulder Pan
- `2` - Control Shoulder Lift
- `3` - Control Elbow Flex
- `4` - Control Wrist Flex
- `5` - Control Wrist Roll
- `6` - Control Gripper
- `0` - Reset all motors to zero
- `q` - Quit

### Motor Control Mode

- `←` (Left Arrow) - Move -5%
- `→` (Right Arrow) - Move +5%
- `b` - Back to main menu

## Requirements

```bash
pip install keyboard
```

**Note:** The `keyboard` library requires sudo/root permissions on Linux. On macOS, it should work without special permissions.

If the keyboard library is not available, the program will fall back to simple text input mode:

- `l` - Move left (-5%)
- `r` - Move right (+5%)
- `b` - Back to menu

## Motor Ranges

- **Gripper**: 0% to 100% (0% = closed, 100% = open)
- **All other motors**: -100% to 100% (0% = center position)
