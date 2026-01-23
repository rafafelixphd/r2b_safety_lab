"""
Ping and control using LeRobot with your existing SO-101 calibration file.

This demonstrates the proper production way to use FeetechMotorsBus with
your actual robot calibration.
"""

import sys
sys.path.insert(0, '/Users/rafaelfelix/Projects/r2b/lerobot/src')

from lerobot.motors import Motor, MotorNormMode, MotorCalibration
from lerobot.motors.feetech import FeetechMotorsBus
import r2b
import time
import json
from pathlib import Path

logger = r2b.get_logger()

def load_calibration(calibration_path: str) -> dict:
    """Load calibration from JSON file."""
    with open(calibration_path, 'r') as f:
        cal_data = json.load(f)
    
    # Convert JSON to MotorCalibration objects
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

def main():
    # Load your existing calibration
    cal_path = Path.home() / ".cache/huggingface/lerobot/calibration/robots/so101_follower/so_101_follower_001.json"
    
    logger.info(f"Loading calibration from: {cal_path}")
    calibration = load_calibration(str(cal_path))
    logger.info(f"Loaded calibration for motors: {list(calibration.keys())}")
    
    # Initialize bus with your SO-101 configuration
    bus = FeetechMotorsBus(
        port="/dev/tty.usbmodem5AAF2631481",
        motors={
            "shoulder_pan": Motor(1, "sts3215", MotorNormMode.RANGE_M100_100),
            "shoulder_lift": Motor(2, "sts3215", MotorNormMode.RANGE_M100_100),
            "elbow_flex": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
            "wrist_flex": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
            "wrist_roll": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
            "gripper": Motor(6, "sts3215", MotorNormMode.RANGE_0_100),
        },
        calibration=calibration
    )
    
    try:
        logger.info("Connecting to motors...")
        bus.connect()
        
        # Ping all motors
        logger.info("\nPinging all motors...")
        for motor_name in bus.motors:
            model = bus.ping(motor_name)
            if model:
                logger.info(f"  ✓ {motor_name}: Model {model}")
            else:
                logger.warning(f"  ✗ {motor_name}: No response")
        
        # Focus on gripper (ID:6)
        logger.info("\n--- Testing Gripper (ID:6) ---")
        logger.info("Enabling torque...")
        bus.enable_torque("gripper")
        
        # Move to center (50%)
        logger.info("Moving to 50% (center)...")
        bus.write("Goal_Position", "gripper", 100.0)
        time.sleep(0.5)
        
        pos = bus.read("Present_Position", "gripper")
        logger.info(f"Current position: {pos:.1f}%")
        
        logger.info("✓ Done! Check your 7.4V/12V power supply if no movement.")
        
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            logger.info("Disconnecting...")
            bus.disconnect()
        except:
            pass

if __name__ == "__main__":
    main()
