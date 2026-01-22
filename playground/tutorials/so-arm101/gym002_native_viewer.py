import argparse
import time
import gymnasium as gym
import gym_aloha
import mujoco
import mujoco.viewer
import r2b
import numpy as np

# Configure logger
logger = r2b.get_logger(__name__)

def get_mujoco_model_data(env):
    """
    Recursively search for the MuJoCo model and data in the environment wrappers.
    """
    current = env
    
    # Traverse unwrapped or internal environments
    candidates = [
        current, 
        getattr(current, "unwrapped", None), 
        getattr(getattr(current, "unwrapped", None), "_env", None)
    ]

    for candidate in candidates:
        if candidate is None:
            continue
        
        # Check for standard Gymnasium MuJoCoEnv attributes
        if hasattr(candidate, "model") and hasattr(candidate, "data"):
            return candidate.model, candidate.data
        
        # Check for DeepMind Control Suite / similar wraps
        if hasattr(candidate, "physics"):
             try:
                 return candidate.physics.model.ptr, candidate.physics.data.ptr
             except AttributeError:
                 pass
                 
    return None, None

def main(args):
    logger.info(f"Creating environment: {args.env}")
    # Render mode 'rgb_array' ensures we don't open an unwanted window, 
    # but still initialize the physics engine.
    env = gym.make(args.env, render_mode="rgb_array")
    
    env.reset()
    
    # Attempt to find the underlying MuJoCo model and data
    model, data = get_mujoco_model_data(env)

    if model is None or data is None:
        logger.critical("Could not access 'model' or 'data' from the environment.")
        logger.critical(f"Env dir: {dir(env.unwrapped)}") 
        if hasattr(env.unwrapped, "_env"):
             logger.critical(f"Inner Env dir: {dir(env.unwrapped._env)}")
        return

    logger.info("Launching native MuJoCo viewer... Press ESC to exit.")

    # Launch the passive viewer.
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # --- Control Loop ---
            action = env.action_space.sample()
            observation, reward, terminated, truncated, info = env.step(action)

            # --- Synchronization ---
            viewer.sync()

            if terminated or truncated:
                env.reset()

            # --- Timing ---
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    logger.info("Viewer closed.")
    env.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run gym_aloha with native MuJoCo viewer.")
    parser.add_argument("--env", type=str, default="gym_aloha/AlohaInsertion-v0", help="Environment ID")
    args = parser.parse_args()
    
    main(args)
