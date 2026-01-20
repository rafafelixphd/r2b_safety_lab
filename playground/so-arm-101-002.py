# 
import os
import time
import mujoco
import mujoco.viewer
from pathlib import Path
import r2b

logger = r2b.get_logger(__name__)

for var in os.getenv("PROJECT_VARS", "").split(" "):
    logger.info(f"{var}: {os.getenv(var, "")}")


model_path = Path(__file__).parent / "artifacts/scene_001.xml"

if not os.path.exists(model_path):
    logger.error(f"Error: {model_path} not found. Are you in the right directory?")
    logger.critical("Exiting...")
    exit()

# 2. Load the model and data
model = mujoco.MjModel.from_xml_path(str(model_path))
data = mujoco.MjData(model)

# 3. Launch the viewer
# This opens a window where you can drag joints and see the physics
with mujoco.viewer.launch_passive(model, data) as viewer:
    logger.info("MuJoCo Viewer started. You can drag the robot parts with your mouse.")
    
    # Keep the simulation running
    while viewer.is_running():
        step_start = time.time()

        # Step the physics
        mujoco.mj_step(model, data)

        # Sync the viewer with the simulation state
        viewer.sync()

        # Maintain real-time speed
        time_until_next_step = model.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)