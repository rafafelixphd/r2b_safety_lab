import gymnasium as gym
import mujoco
import mujoco.viewer
import numpy as np
import gym_aloha

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

# Initialize the environment
# env = gym.make("Ant-v5", render_mode="rgb_array")
env = gym.make("gym_aloha/AlohaInsertion-v0", render_mode="rgb_array")
env.reset()

model, data = get_mujoco_model_data(env)

print(model, data)
with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()