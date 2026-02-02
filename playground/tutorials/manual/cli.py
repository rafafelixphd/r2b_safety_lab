
import sys
# Hack to import r2b from lerobot/src or similar if needed, matching ping.py
sys.path.insert(0, '/Users/rafaelfelix/Projects/r2b/lerobot/src')

import tty
import termios
import time
import json
from pathlib import Path
import r2b
from lerobot.motors import Motor, MotorNormMode, MotorCalibration
from lerobot.motors.feetech import FeetechMotorsBus

logger = r2b.get_logger()

# --- Key Mapping ---
# User Key -> Motor Name (as per plan: 1=Gripper=ID6, ... 6=Pan=ID1)
KEY_TO_MOTOR = {
    '1': 'gripper',         # ID 6
    '2': 'wrist_roll',      # ID 5
    '3': 'wrist_flex',      # ID 4
    '4': 'elbow_flex',      # ID 3
    '5': 'shoulder_lift',   # ID 2
    '6': 'shoulder_pan'     # ID 1
}

MOTOR_CONFIG = {
    "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
}

def getch():
    """Reads a single character from stdin without requiring enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        if ch == '\x1b':  # Escape sequence processing for arrows
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                if ch3 == 'D': return 'LEFT'
                if ch3 == 'C': return 'RIGHT'
                if ch3 == 'A': return 'UP'
                if ch3 == 'B': return 'DOWN'
        return ch
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

def load_calibration(calibration_path: str) -> dict:
    """Load calibration from JSON file."""
    with open(calibration_path, 'r') as f:
        cal_data = json.load(f)
    
    calibration = {}
    for motor_name, cal in cal_data.items():
        calibration[motor_name] = MotorCalibration(
            id=cal['id'],
            drive_mode=cal['drive_mode'],
            homing_offset=cal['homing_offset'],
            range_min=cal['range_min'],
            range_max=cal['range_max']
        )
    return calibration

def print_interface(selected_motors, multi_select_mode, step_size, current_positions):
    """Prints the CLI interface."""
    # Clear screen and move to top
    print('\033[2J\033[H', end='')
    
    print("-" * 50)
    print(" R2B MANUAL ROBOT CONTROL CLI")
    print("-" * 50)
    print(f" Mode: {'[MULTI-SELECT]' if multi_select_mode else '[SINGLE-SELECT]'}")
    print(f" Step Size: {step_size}% (+/- to adjust)")
    print("-" * 50)
    print(f" Selected Joints: {', '.join(selected_motors) if selected_motors else 'None'}")
    print("-" * 50)
    print(" Motor Status:")
    for key, name in KEY_TO_MOTOR.items():
        selected_marker = "*" if name in selected_motors else " "
        pos = current_positions.get(name, "N/A")
        if isinstance(pos, float):
            pos_str = f"{pos:6.1f}%"
        else:
            pos_str = str(pos)
        print(f" [{key}] {selected_marker} {name:<15} : {pos_str}")
    print("-" * 50)
    print(" CONTROLS:")
    print("  1-6  : Select Joint (Gripper..Pan)")
    print("  j    : Toggle Multi-Select Mode")
    print("  c    : Clear Selection")
    print("  <-   : Move Negative")
    print("  ->   : Move Positive")
    print("  9    : Flex (100%)")
    print("  0    : Center (50% / 0% depending on range)")
    print("  q    : Quit")
    print("-" * 50)

def main(driver_id: str):
    # Load calibration
    # cal_path = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower/v3.json"
    cal_path = Path.home() / ".cache/huggingface/lerobot/calibration/teleoperators/so101_leader/v2.json"
    if not cal_path.exists():
        logger.error(f"Calibration file not found at: {cal_path}")
        return

    logger.info("Loading calibration...")
    calibration = load_calibration(str(cal_path))
    
    # Initialize bus
    bus = FeetechMotorsBus(
        port=driver_id,
        motors=MOTOR_CONFIG,
        calibration=calibration
    )

    try:
        logger.info("Connecting to motors...")
        bus.connect()
        
        # Enable torque for all to hold position
        for name in bus.motors:
            bus.enable_torque(name) # Verify if this is safe/intended? Usually yes for control.

        selected_motors = set()
        multi_select_mode = False
        step_size = 3.0
        
        # Initial read of positions
        current_positions = {}
        for name in bus.motors:
            current_positions[name] = bus.read("Present_Position", name)

        while True:
            print_interface(selected_motors, multi_select_mode, step_size, current_positions)
            
            key = getch()
            
            if key == 'q':
                break
            
            # Selection Logic
            if key in KEY_TO_MOTOR:
                motor_name = KEY_TO_MOTOR[key]
                if multi_select_mode:
                    if motor_name in selected_motors:
                        selected_motors.remove(motor_name)
                    else:
                        selected_motors.add(motor_name)
                else:
                    selected_motors = {motor_name}
            
            # Mode Toggle
            elif key == 'j':
                multi_select_mode = not multi_select_mode
            
            # Clear Selection
            elif key == 'c':
                selected_motors.clear()
            
            # Step Size
            elif key == '+':
                step_size = min(step_size + 1.0, 50.0)
            elif key == '-':
                step_size = max(step_size - 1.0, 0.1)
                
            # Movement Logic
            elif key == 'LEFT' or key == 'RIGHT':
                if not selected_motors:
                    continue
                
                direction = 1 if key == 'RIGHT' else -1
                delta = direction * step_size
                
                for name in selected_motors:
                    current_pos = bus.read("Present_Position", name)
                    if current_pos is not None:
                        new_pos = current_pos + delta
                        new_pos = max(-100, min(100, new_pos))
                        bus.write("Goal_Position", name, new_pos)
                        
            elif key == '9': # Flex / 100%
                targets = list(selected_motors) if selected_motors else list(bus.motors.keys())
                logger.info(f"Moving {len(targets)} motors to 100%...")
                for name in targets:
                     bus.write("Goal_Position", name, 100.0)
                     time.sleep(0.5) # One at a time
                     
            elif key == '0': # Reset / Center
                targets = list(selected_motors) if selected_motors else list(bus.motors.keys())
                logger.info(f"Moving {len(targets)} motors to Center...")
                for name in targets:
                    if name == 'gripper':
                        bus.write("Goal_Position", name, 50.0)
                    else:
                        bus.write("Goal_Position", name, 0.0)
                    time.sleep(0.5) # One at a time
            
            # Update all positions for display
            # To avoid slow loop, maybe read only occasionally or use a separate thread?
            # For this simple CLI, reading all 6 every loop might make UI sluggish if bus is slow.
            # But print_interface clears screen.
            # Let's accept some delay.
            for name in bus.motors:
                val = bus.read("Present_Position", name)
                if val is not None:
                    current_positions[name] = val
                    
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        bus.disconnect()
        print("\nDisconnected.")

if __name__ == "__main__":
    import sys
    main(sys.argv[1])
