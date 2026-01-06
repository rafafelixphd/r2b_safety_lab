import os
import time
import mujoco
import mujoco.viewer
from pathlib import Path
import numpy as np
import r2b

# Initialize Logger
logger = r2b.get_logger(__name__)

# --- [C] Configuration ---
class SimConfig:
    """Centralized Simulation Configuration."""
    DT = 0.002  # 500Hz Physics
    RENDER_DT = 1.0 / 60.0 # 60FPS Rendering
    
    # Bench Dimensions
    BENCH_SIZE = [0.6, 0.4, 0.4] # Half-sizes [lx, ly, lz] -> 1.2m x 0.8m x 0.8m
    BENCH_POS = [0, 0, 0.4]      # Center of bench (z is half-height)
    
    # Robot Placement (On top of bench)
    ROBOT_POS = [0, 0, 0.8]      # z = BENCH_POS[z] + BENCH_SIZE[z]
    
    # Visuals
    SKYBOX_COLOR = "0.1 0.1 0.1"


# --- [E] Environment ---
class WorkbenchEnv:
    """
    The 'Bridge' between Physics Engine and Controller.
    Manages the scene, assets, and robot loading.
    """
    def __init__(self, robot_xml_path: Path):
        self._robot_path = robot_xml_path.resolve()
        self.model = None
        self.data = None
        
        if not self._robot_path.exists():
            logger.critical(f"Robot XML not found at {self._robot_path}")
            raise FileNotFoundError(f"Robot XML not found at {self._robot_path}")
            
        self._build_scene()

    def _build_scene(self):
        """Constructs the MjModel by composing the Robot XML into a Scene XML."""
        logger.info(f"Building Workbench Environment for: {self._robot_path.name}")
        
        # MCE: decoupled environment definition
        # We use <include> wrapped in a body to place the robot.
        # Load the scene XML template
        scene_xml_path = Path(__file__).parent / "artifacts" / "workbench_scene.xml"
        if not scene_xml_path.exists():
             logger.critical(f"Scene XML not found at {scene_xml_path}")
             raise FileNotFoundError(f"Scene XML not found at {scene_xml_path}")
        
        try:
            self.model = mujoco.MjModel.from_xml_string(str(scene_xml_path))
            self.data = mujoco.MjData(self.model)
            self.model.opt.timestep = SimConfig.DT
            logger.info("MuJoCo Model loaded successfully.")
            
        except Exception as e:
            logger.critical(f"Failed to compile MuJoCo model: {e}")
            raise

    def step(self):
        """Physics Step."""
        mujoco.mj_step(self.model, self.data)

    def sync_viewer(self, viewer):
        """Syncs the viewer."""
        viewer.sync()


# --- [M] Controller ---
class PassiveController:
    """
    The 'Brain'.
    Engine-Agnostic logic. For now, it's a stub/passive controller.
    """
    def __init__(self, dt: float):
        self.dt = dt

    def compute(self, state):
        # Placeholder: Return zero torques or hold position
        return np.zeros_like(state)


# --- Main Logic ---
def main():
    # 0. Setup
    # Resolve path (assuming SO101_DIR is set, or fallback logic if needed)
    base_dir = os.getenv("SO101_DIR")
    if not base_dir:
        # Fallback for when current dir acts as project root or similar
        # But per user snippet, we rely on env var or custom path
        logger.warning("SO101_DIR env var not set. Assuming default/current directory structure.")
        base_dir = os.getcwd() # Or some other heuristic
        
    model_path = Path(base_dir) / "Simulation/SO101/so101_new_calib.xml"
    
    # 1. Initialize Environment
    try:
        env = WorkbenchEnv(model_path)
    except Exception as e:
        logger.error(f"Initialization Failed: {e}")
        return

    # 2. Initialize Controller
    ctrl = PassiveController(dt=SimConfig.DT)

    # 3. Simulation Loop
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        logger.info("Simulation Started. Press Ctrl+C to exit.")
        
        while viewer.is_running():
            step_start = time.time()

            # Controller Logic (Placeholder)
            # action = ctrl.compute(...)
            # env.apply_action(action) ...
            
            # Physics Step (Environment)
            env.step()

            # Render
            env.sync_viewer(viewer)

            # Timing
            time_until_next_step = env.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()