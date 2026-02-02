
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

from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
from lerobot.teleoperators.so101_leader.so101_leader import SO101Leader


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

def save_calibration(calibration: dict, calibration_path: str):
    """Save calibration to JSON file."""
    cal_data = {}
    for name, cal in calibration.items():
        cal_data[name] = {
            "id": cal.id,
            "drive_mode": cal.drive_mode,
            "homing_offset": cal.homing_offset,
            "range_min": cal.range_min,
            "range_max": cal.range_max
        }
    
    with open(calibration_path, 'w') as f:
        json.dump(cal_data, f, indent=4)
    logger.info(f"Calibration saved to {calibration_path}")

def print_interface(selected_motors, multi_select_mode, step_size, current_positions, raw_positions, calibration, last_error=None):
    """Prints the CLI interface."""
    # Clear screen and move to top
    print('\033[2J\033[H', end='')
    
    print("-" * 80)
    print(" R2B ROBOT CALIBRATION CLI")
    print("-" * 80)
    print(f" Mode: {'[MULTI-SELECT]' if multi_select_mode else '[SINGLE-SELECT]'}")
    print(f" Step Size: {step_size}% (+/- to adjust)")
    print("-" * 80)
    print(f" Selected Joints: {', '.join(selected_motors) if selected_motors else 'None'}")
    print("-" * 80)
    print(" Motor Status:")
    print(f" {'ID':<4} {'Name':<15} : {'Pos %':<8} | {'Raw':<6} | {'Min':<6} | {'Max':<6}")
    for key, name in KEY_TO_MOTOR.items():
        selected_marker = "*" if name in selected_motors else " "
        pos = current_positions.get(name, "N/A")
        raw = raw_positions.get(name, "N/A")
        
        # Format values
        # Reverting to percentage display
        pos_str = f"{pos:>7.1f}%" if isinstance(pos, (int, float)) else f"{str(pos):>7}"
        raw_str = f"{raw:<6}"
        
        # Get Min/Max from calibration if available
        min_str = "N/A"
        max_str = "N/A"
        if calibration and name in calibration:
            min_str = str(calibration[name].range_min)
            max_str = str(calibration[name].range_max)
            
        print(f" [{key}] {selected_marker} {name:<14} : {pos_str}  | {raw_str} | {min_str:<6} | {max_str:<6}")
    print("-" * 80)
    print(" CONTROLS:")
    print("  1-6  : Select Joint (Gripper..Pan)")
    print("  j    : Toggle Multi-Select Mode")
    print("  c    : Clear Selection")
    print("  <-   : Move Negative")
    print("  ->   : Move Positive")
    print("  o    : Set Origin (Min Limit) to Current Position")
    print("  p    : Set Limit (Max Limit) to Current Position")
    print("  r    : Reset Calibration (Min=0, Max=Full, Offset=0)")
    print("  z    : Zero Motor (Set Current Pos as 0)")
    print("  s    : Save Calibration")
    print("  q    : Quit")
    print("-" * 80)
    if last_error:
        print(f" LAST ERROR: {last_error}")
    print("-" * 80)

def main(driver_id: str, robot_type: str = "follower", calibration_version: str = "v3"):
    '''
    robot_type = follower or leader
    calibration_version = v3 or v2
    '''
    # Load calibration path (just for saving later if needed, though device has its own management)
    # Actually, we should let the device classes handle paths if possible, but our save logic is manual.
    # Let's keep cal_path derivation for the 's' command.
    if robot_type == "follower":
        cal_path = Path.home() / f".cache/huggingface/lerobot/calibration/robots/so101_follower/{calibration_version}.json"
        
        # Instantiate Robot
        config = SO101FollowerConfig(port=driver_id)
        # Force calibration path if needed? Config doesn't usually take version directly easily
        # but the class defaults might check specific paths. 
        # For now, we instantiate the device which sets up the bus.
        device = SO101Follower(config)
        
    elif robot_type == "leader":
        cal_path = Path.home() / f".cache/huggingface/lerobot/calibration/teleoperators/so101_leader/{calibration_version}.json"
        
        # Instantiate Teleoperator
        config = SO101LeaderConfig(port=driver_id)
        device = SO101Leader(config)
        
    else:
        logger.error(f"Invalid robot type: {robot_type}")
        return

    # We still want to load the *specific* calibration file we are targeting ('v3' etc)
    # The device logic loads whatever it finds or defaults.
    # We will overwrite the bus.calibration with what we load from our specific file to map validly.
    
    if not cal_path.exists():
        logger.warning(f"Calibration file not found at: {cal_path}. Starting with empty/default calibration.")
        calibration = {}
    else:
        logger.info(f"Loading calibration from {cal_path}...")
        calibration = load_calibration(str(cal_path))
    
    # Initialize bus via device
    # connect(calibrate=False) avoids the 'check calibration' logic that raises errors.
    logger.info("Connecting to device...")
    try:
        device.connect(calibrate=False)
    except Exception as e:
        logger.error(f"Failed to connect: {e}")
        return

    # Get the bus from the device
    bus = device.bus
    
    # Apply our specifically loaded calibration to the bus (in memory)
    # This ensures we are editing the file we intended (v3) even if the device loaded something else.
    if calibration:
        bus.calibration = calibration

    try:
        # Enable torque for all to hold position
        for name in bus.motors:
            bus.enable_torque(name) 

        selected_motors = set()
        multi_select_mode = False
        step_size = 3.0
        
        # Initial read of positions
        current_positions = {}
        raw_positions = {}
        last_error = None
        
        while True:
            # Read values safely
            try:
                # Use sync_read if available for better performance, or read individually
                # FeetechMotorsBus typically supports sync_read via bus.sync_read("Present_Position", list_of_names)
                # Note: sync_read returns normalized values if bus.calibration is set.
                # To get Raw for display, we might need a separate read or unnormalize?
                # Actually bus.read(..., normalize=False) works.
                # Let's try to read all at once if possible, but fallback to individual safely.
                
                # Check if bus has sync_read method exposed comfortably or just iterate
                # For safety and clarity in this CLI, we will iterate but wrap in try-except
                
                for name in bus.motors:
                    try:
                        # Read Raw
                        raw_val = bus.read("Present_Position", name, normalize=False)
                        raw_positions[name] = raw_val
                        
                        # Read Calibrated (Percent)
                        # We calculate this manually from raw to ensure it uses the *latest* calibration
                        # even before it's written to hardware, because `bus.read(..., normalize=True)`
                        # might rely on `bus.calibration` which we update in memory.
                        if bus.calibration and name in bus.calibration and raw_val is not None:
                            cal = bus.calibration[name]
                            # Manual Normalization Logic to match UI expectations
                            # range_min -> 0 or -100
                            # range_max -> 100
                            # This depends on drive mode etc.
                            # Let's check MotorNormMode
                            min_ = cal.range_min
                            max_ = cal.range_max
                            val = raw_val
                            
                            # Simple clamp
                            bounded_val = min(max(val, min(min_, max_)), max(min_, max_)) # Safe clamp even if inverted ranges?
                            # Actually usually min < max. 
                            
                            # Re-implement simple normalization for display
                            # This ensures that as soon as we press 'o', 
                            # the current raw matches range_min, so (val - min) = 0 -> 0%
                            
                            if max_ == min_:
                                current_positions[name] = 0.0
                            else:
                                if bus.motors[name].norm_mode == MotorNormMode.RANGE_M100_100:
                                    # This mode is typically -100 to 100
                                    # But user asked for 0 at 'o' and 100 at 'p' implies 0-100 range?
                                    # Or maybe -100 to 100 is fine, but 'o' sets the lower bound.
                                    # If 'o' is pressed at current pos, that pos becomes min (-100%).
                                    # If 'p' is pressed, that pos becomes max (100%).
                                    # Let's trust standard normalization.
                                    norm = (((bounded_val - min_) / (max_ - min_)) * 200) - 100
                                    current_positions[name] = norm
                                elif bus.motors[name].norm_mode == MotorNormMode.RANGE_0_100:
                                    norm = ((bounded_val - min_) / (max_ - min_)) * 100
                                    current_positions[name] = norm
                                elif bus.motors[name].norm_mode == MotorNormMode.DEGREES:
                                    # From lerobot motors_bus.py:
                                    # mid = (min_ + max_) / 2
                                    # max_res = self.model_resolution_table[self._id_to_model(id_)] - 1
                                    # normalized_values[id_] = (val - mid) * 360 / max_res
                                    
                                    model = bus.motors[name].model
                                    max_res = bus.model_resolution_table.get(model, 4096) - 1
                                    # Use float division
                                    mid = (min_ + max_) / 2.0
                                    norm = (val - mid) * 360.0 / max_res
                                    current_positions[name] = norm
                                else:
                                    current_positions[name] = "UNK" # Setup doesn't match expected types
                        else:
                             current_positions[name] = "N/A"
                             
                    except Exception as e:
                        current_positions[name] = "ERR"
                        raw_positions[name] = "ERR"
                        last_error = f"{name}: {str(e)}"

            except Exception as e:
                logger.error(f"Bus Error: {e}")
                last_error = f"Bus Error: {str(e)}"
                time.sleep(0.5)
            
            print_interface(selected_motors, multi_select_mode, step_size, current_positions, raw_positions, calibration, last_error)
            
            key = getch()
            
            if key == 'q':
                break
            
            try:
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
                        current_pos = current_positions.get(name)
                        if isinstance(current_pos, (int, float)):
                            new_pos = current_pos + delta
                            # Clamp removed as per user request to allow reaching physical limits
                            # if bus.motors[name].norm_mode == MotorNormMode.RANGE_M100_100:
                            #    new_pos = max(-100, min(100, new_pos))
                            # elif bus.motors[name].norm_mode == MotorNormMode.RANGE_0_100:
                            #    new_pos = max(0, min(100, new_pos))
                            
                            bus.write("Goal_Position", name, new_pos)
                
                # Calibration Logic
                elif key == 'o': # Set Origin / Min
                    if not selected_motors:
                        continue
                    for name in selected_motors:
                        raw = raw_positions.get(name)
                        if isinstance(raw, int):
                            old_cal = calibration[name]
                            new_cal = MotorCalibration(
                                id=old_cal.id,
                                drive_mode=old_cal.drive_mode,
                                homing_offset=old_cal.homing_offset,
                                range_min=raw,
                                range_max=old_cal.range_max
                            )
                            calibration[name] = new_cal
                            # Update bus internal calibration immediately for display
                            bus.calibration[name] = new_cal

                elif key == 'p': # Set Limit / Max
                    if not selected_motors:
                        continue
                    for name in selected_motors:
                        raw = raw_positions.get(name)
                        if isinstance(raw, int):
                            old_cal = calibration[name]
                            new_cal = MotorCalibration(
                                id=old_cal.id,
                                drive_mode=old_cal.drive_mode,
                                homing_offset=old_cal.homing_offset,
                                range_min=old_cal.range_min,
                                range_max=raw
                            )
                            calibration[name] = new_cal
                            bus.calibration[name] = new_cal
                
                elif key == 'r': # Reset
                    if not selected_motors:
                        continue
                    for name in selected_motors:
                        # Get max resolution from bus table
                        model = bus.motors[name].model
                        # Default to 4095 (12-bit) if not found, but it should be in the table
                        max_res = bus.model_resolution_table.get(model, 4096) - 1
                        
                        old_cal = calibration[name]
                        new_cal = MotorCalibration(
                            id=old_cal.id,
                            drive_mode=old_cal.drive_mode,
                            homing_offset=0,
                            range_min=0,
                            range_max=max_res
                        )
                        calibration[name] = new_cal
                        bus.calibration[name] = new_cal
                
                elif key == 'z': # Zero
                    if not selected_motors:
                        continue
                    for name in selected_motors:
                        try:
                            # 1. Get current RAW position (with current offset applied)
                            # Present_Position = Internal_Pos - Homing_Offset
                            # We want Present_Position to be 0.
                            # So NEW_Homing_Offset = Internal_Pos
                            # Internal_Pos = Present_Position + Homing_Offset
                            
                            p_old = raw_positions.get(name)
                            # Handle error string from display loop
                            if p_old is None or isinstance(p_old, str):
                                raise RuntimeError(f"Cannot zero {name}: Read failed ({p_old})")
                            
                            # Read current Homing Offset from hardware to be sure
                            h_old = bus.read("Homing_Offset", name, normalize=False)
                            
                            # New offset target (unbounded)
                            new_offset = p_old + h_old
                            
                            # Wrap offset to be within valid Homing_Offset range for Feetech (Sign-Magnitude 11-bit: [-2047, 2047])
                            # Motor resolution period
                            model = bus.motors[name].model
                            max_res = bus.model_resolution_table.get(model, 4096) - 1
                            period = max_res + 1 # e.g. 4096
                            
                            # Wrap to [-period/2, period/2 - 1] approx
                            # Actually strict limit is +/- 2047.
                            
                            # Normalize into [0, period) first
                            new_offset = new_offset % period
                            
                            # Shift to [-period/2, period/2]
                            half_period = period // 2
                            if new_offset > half_period:
                                new_offset -= period
                                
                            # If for some reason it's still out of bounds (e.g. if period > 4096 which shouldn't happen for STS3215), cap it?
                            # But STS3215 is 12-bit.
                            
                            logger.info(f"Zeroing {name}: Raw={p_old}, OldOffset={h_old} -> NewOffset={new_offset}")
                            
                            # Write NEW offset to hardware (this sets current pos to 0)
                            bus.write("Homing_Offset", name, new_offset)
                            
                            # Reset Limits to Full Range
                            
                            old_cal = calibration[name]
                            new_cal = MotorCalibration(
                                id=old_cal.id,
                                drive_mode=old_cal.drive_mode,
                                homing_offset=new_offset,
                                range_min=0,
                                range_max=max_res
                            )
                            calibration[name] = new_cal
                            bus.calibration[name] = new_cal
                            
                        except Exception as e:
                            logger.error(f"Zero Error: {e}")
                            import traceback
                            traceback.print_exc()
                            # Give visual feedback via last_error if possible, or just log
                            # We can't easily push to last_error here without changing main structure significantly,
                            # but the loop catches standard exceptions. 
                            # We caught it here to prevent breaking the loop.
                            pass
                    logger.info("Zeroed!")
                    time.sleep(1)
                elif key == 's': # Save
                    save_calibration(calibration, str(cal_path))
                    logger.info("Saved!")
                    time.sleep(1)

            except Exception as e:
                # Log error but don't crash
                # Use a small sleep to avoid tight loop log spam if persistent
                logger.error(f"Command Error: {e}")
                time.sleep(0.5)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            bus.disconnect()
        except Exception as e:
            # Often fails if motors are overloaded, but we are quitting anyway.
            logger.error(f"Disconnect Error (Ignored): {e}")
        print("\nDisconnected.")

if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python cli-calibration.py <port> <follower|leader> <version>")
        sys.exit(1)
    
    robot_type = sys.argv[2] if len(sys.argv) > 2 else "follower"
    version = sys.argv[3] if len(sys.argv) > 3 else "v3"
    
    main(sys.argv[1], robot_type, version)
