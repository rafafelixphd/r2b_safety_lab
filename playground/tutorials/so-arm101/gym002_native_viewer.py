import argparse
import time
import threading
import gymnasium as gym
import gym_aloha
import mujoco
import mujoco.viewer
import r2b
import numpy as np

# Configure logger
logger = r2b.get_logger(__name__)

# Global flag to control the background thread
running = True

def get_mujoco_model_data(env):
    """
    Recursively search for the MuJoCo model and data in the environment wrappers.
    """
    current = env
    candidates = [
        current, 
        getattr(current, "unwrapped", None), 
        getattr(getattr(current, "unwrapped", None), "_env", None)
    ]

    for candidate in candidates:
        if candidate is None:
            continue
        if hasattr(candidate, "model") and hasattr(candidate, "data"):
            return candidate.model, candidate.data
        if hasattr(candidate, "physics"):
             try:
                 return candidate.physics.model.ptr, candidate.physics.data.ptr
             except AttributeError:
                 pass
    return None, None

def env_loop(env, model):
    global running
    logger.info("Environment loop started in background thread.")
    
    try:
        while running:
            step_start = time.time()

            action = env.action_space.sample()
            observation, reward, terminated, truncated, info = env.step(action)
            if terminated or truncated:
                env.reset()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
    except Exception as e:
        logger.error(f"Exception in background loop: {e}")
    finally:
        logger.info("Environment loop stopping.")
        try:
            env.close()
        except:
            pass

def main(args):
    global running
    logger.info(f"Creating environment: {args.env}")
    
    try:
        env = gym.make(args.env, render_mode=None)
    except Exception:
        logger.warning("Could not create env with render_mode=None, trying default.")
        env = gym.make(args.env)
    
    env.reset()
    
    model, data = get_mujoco_model_data(env)

    if model is None or data is None:
        logger.critical("Could not access 'model' or 'data' from the environment.")
        return

    logger.info("Launching native MuJoCo viewer (blocking main thread)...")
    
    # Start the environment stepping in a background thread
    thread = threading.Thread(target=env_loop, args=(env, model), daemon=True)
    thread.start()

    # Launch the blocking viewer on the main thread.
    # This function will return when the user closes the viewer window.
    try:
        mujoco.viewer.launch(model, data)
    except Exception as e:
        logger.critical(f"Viewer crashed: {e}")
    finally:
        # Signal thread to stop
        running = False
        thread.join(timeout=1.0)
        logger.info("Done.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run gym_aloha with native MuJoCo viewer.")
    parser.add_argument("--env", type=str, default="gym_aloha/AlohaInsertion-v0", help="Environment ID")
    args = parser.parse_args()
    
    main(args)
