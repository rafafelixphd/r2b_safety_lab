#!/usr/bin/env python
import os
import time
import argparse
from r2b.logger import get_logger

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.teleoperators.so_leader import SO101Leader, SO101LeaderConfig

logger = get_logger(namespace="reset-robot")

# Base (home) position. 
# You may adjust these to the safest "parking" position.
HOME_POS = {
    "shoulder_pan.pos": 0.0,
    "shoulder_lift.pos": -100.0,
    "elbow_flex.pos": 100.0,
    "wrist_flex.pos": -100.0,
    "wrist_roll.pos": 0.0, # Adjust if this needs to be 0 or another center
    "gripper.pos": 0.0,  # 100 is typically fully open depending on calibration
}

def interpolate(start, end, progress):
    """Interpolate between start and end values based on progress (0.0 to 1.0)."""
    return start + (end - start) * progress

def read_position(robot):
    """Unified read for both follower and leader."""
    if hasattr(robot, "get_observation"):
        return robot.get_observation()
    else:
        return robot.get_action()

def write_position(robot, target_action):
    """Unified write for both follower and leader."""
    if hasattr(robot, "send_action"):
        robot.send_action(target_action)
    else:
        # Leader arm manually
        goal_pos = {key.removesuffix(".pos"): val for key, val in target_action.items() if key.endswith(".pos")}
        robot.bus.sync_write("Goal_Position", goal_pos)

def reset_single_robot(robot, args):
    """Performs the reset operation on a single robot instance."""
    logger.info(f"Connecting to {robot.__class__.__name__}...")
    try:
        robot.connect()
    except Exception as e:
        logger.error(f"Failed to connect to the robot: {e}")
        return

    try:
        # Leader arm connects with torque disabled by default. Enable it to move.
        if not hasattr(robot, "send_action"):
            robot.bus.enable_torque()
            time.sleep(0.1)

        logger.info("Reading current state...")
        obs = read_position(robot)
        
        # Extract starting positions for motors
        start_pos = {}
        target_pos_dict = {}
        
        # Filter home pos to only include what's present in observation
        for motor, target_val in HOME_POS.items():
            if motor in obs:
                start_pos[motor] = obs[motor]
                target_pos_dict[motor] = target_val
            else:
                logger.warning(f"{motor} not found in observation. Skipping.")

        if not start_pos:
            logger.error("No valid motors found in observation!")
            return

        num_steps = int(args.duration * args.fps)
        sleep_time = 1.0 / args.fps
        
        # Stuck detection parameters
        stuck_threshold = 0.5  # maximum position change to be considered "stuck"
        max_stuck_steps = int(args.fps * 0.5)  # half a second of being stuck
        consecutive_stuck_steps = 0
        previous_obs = None

        logger.info(f"Moving to home position smoothly over {args.duration}s ({num_steps} steps)...")
        for step in range(1, num_steps + 1):
            progress = step / float(num_steps)
            
            target_action = {}
            for motor, target_val in target_pos_dict.items():
                target_action[motor] = interpolate(start_pos[motor], target_val, progress)
            
            write_position(robot, target_action)
            
            # Read current position to ensure we are tracking
            current_obs = read_position(robot)
            current_pos_str = ", ".join([f"{motor}: {current_obs.get(motor, 0):.1f}" for motor in target_pos_dict.keys()])
            logger.info(f"Step {step}/{num_steps} | Pos: {current_pos_str}")
            
            # Stuck detection
            if previous_obs is not None:
                max_delta = 0.0
                for motor in target_pos_dict.keys():
                    delta = abs(current_obs.get(motor, 0) - previous_obs.get(motor, 0))
                    max_delta = max(max_delta, delta)
                
                # Only check if we expect it to be moving (not completely at target yet)
                expected_delta = max([abs(target_action[motor] - current_obs.get(motor, 0)) for motor in target_pos_dict.keys()])
                if expected_delta > 1.0 and max_delta < stuck_threshold:
                    consecutive_stuck_steps += 1
                else:
                    consecutive_stuck_steps = 0
                    
                if consecutive_stuck_steps >= max_stuck_steps:
                    logger.error("Robot appears to be stuck or struggling. Aborting motion to protect hardware!")
                    break
            
            previous_obs = current_obs
            time.sleep(sleep_time)

        logger.info("Finished movement interpolation sequence.")
        
        # Verify final state
        final_obs = read_position(robot)
        logger.info("Final Observation:")
        for motor in target_pos_dict.keys():
            logger.info(f"  {motor}: target={target_pos_dict[motor]:.1f}, actual={final_obs.get(motor, 0):.1f}")
            
        user_input = input("\nDoes this position look correct? (y/n): ")
        if user_input.lower().strip() == 'y':
            logger.info("Position confirmed. It is safe to power off or restart teleop.")
        else:
            logger.warning("Position not confirmed. You may want to adjust HOME_POS in the script.")

    except Exception as e:
        logger.error(f"Error during move operation: {e}")
    finally:
        logger.info("Disconnecting...")
        robot.disconnect()

def get_follower_robot():
    port = os.environ.get("FOLLOWER_PORT")
    rob_id = os.environ.get("FOLLOWER_ID")
    if not port:
        logger.warning("FOLLOWER_PORT is not set. Moving forward if configured by default.")
    config = SO101FollowerConfig(port=port, id=rob_id, cameras={})
    return SO101Follower(config)

def get_leader_robot():
    port = os.environ.get("LEADER_PORT")
    rob_id = os.environ.get("LEADER_ID")
    if not port:
        logger.warning("LEADER_PORT is not set. Moving forward if configured by default.")
    config = SO101LeaderConfig(port=port, id=rob_id)
    return SO101Leader(config)

def main():
    parser = argparse.ArgumentParser(description="Smoothly reset SO101 robots to home position.")
    parser.add_argument("--duration", type=float, default=3.0, help="Duration of the motion in seconds (e.g., 3.0).")
    parser.add_argument("--fps", type=int, default=30, help="Control loop rate in Hz (e.g., 30).")
    parser.add_argument("--type", type=str, default="follower", choices=["follower", "leader", "both"], help="Type of robot to reset.")
    args = parser.parse_args()

    robots_to_reset = []
    
    if args.type in ["follower", "both"]:
        robots_to_reset.append(get_follower_robot())
        
    if args.type in ["leader", "both"]:
        robots_to_reset.append(get_leader_robot())

    for robot in robots_to_reset:
        logger.info(f"--- Resetting {robot.__class__.__name__} ---")
        reset_single_robot(robot, args)

if __name__ == "__main__":
    main()
