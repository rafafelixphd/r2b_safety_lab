import time
import mujoco
import mujoco.viewer
import numpy as np
import gymnasium as gym

def show_cylinder(viewer, position, rotation, radius=0.0245, halfheight=0.05, rgba=[1, 0, 0, 1]):
    # Add a cylinder aligned with z-axis
    mujoco.mjv_initGeom(
        viewer.user_scn.geoms[0],
        type=mujoco.mjtGeom.mjGEOM_CYLINDER,   # cylinder type
        size=[radius, halfheight, 0],          # [radius, half-height, ignored]
        pos=position,                          # center position
        mat=rotation.flatten(),                # orientation matrix (identity = z-up)
        rgba=rgba                              # color
    )
    viewer.user_scn.ngeom = 1
    viewer.sync()

class KinematicsTestEnv(gym.Env):
    def __init__(self, xml_path='model/scene.xml'):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path(xml_path)
        self.data = mujoco.MjData(self.model)
        
        # Action space: 6 joints for the arm
        self.action_space = gym.spaces.Box(low=-np.pi, high=np.pi, shape=(6,), dtype=np.float32)
        
        # Observation space
        self.observation_space = gym.spaces.Dict({
            "qpos": gym.spaces.Box(low=-np.inf, high=np.inf, shape=(self.model.nq,), dtype=np.float32),
            "ee_pos": gym.spaces.Box(low=-np.inf, high=np.inf, shape=(3,), dtype=np.float32),
            "ee_mat": gym.spaces.Box(low=-np.inf, high=np.inf, shape=(9,), dtype=np.float32)
        })

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        
        if options and 'qpos_init' in options:
            # Assumes the first N joints correspond to the configuration
            qpos_init = options['qpos_init']
            n = min(len(qpos_init), self.model.nq)
            self.data.qpos[:n] = qpos_init[:n]
            
        mujoco.mj_forward(self.model, self.data)
        return self._get_obs(), {}

    def step(self, action):
        n = min(len(action), len(self.data.ctrl))
        self.data.ctrl[:n] = action[:n]
        mujoco.mj_step(self.model, self.data)
        return self._get_obs(), 0.0, False, False, {}

    def _get_obs(self):
        # Forward kinematics via site position (assumes 'ee_site' exists)
        ee_pos = np.zeros(3)
        ee_mat = np.eye(3).flatten()
        
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        if site_id >= 0:
            ee_pos = self.data.site(site_id).xpos.copy()
            ee_mat = self.data.site(site_id).xmat.copy()
            
        return {
            "qpos": self.data.qpos.copy(),
            "ee_pos": ee_pos,
            "ee_mat": ee_mat
        }

class PositionController:
    def __init__(self, target_positions):
        self.target = np.array(target_positions, dtype=np.float32)
        
    def get_action(self, obs):
        return self.target

if __name__ == "__main__":
    # Original test configuration mapped to radians/units
    test_configuration = [
        -np.deg2rad(45.0), # shoulder_pan
        np.deg2rad(45.0),  # shoulder_lift
        -np.deg2rad(45.0), # elbow_flex
        np.deg2rad(90.0),  # wrist_flex
        0.0,               # wrist_roll
        10.0               # gripper
    ]

    env = KinematicsTestEnv('model/scene.xml')
    obs, _ = env.reset(options={'qpos_init': test_configuration})
    
    controller = PositionController(test_configuration)
    
    with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
        # Add a cylinder as a site for visualization using initial EE position
        if mujoco.mj_name2id(env.model, mujoco.mjtObj.mjOBJ_SITE, "ee_site") >= 0:
            show_cylinder(viewer, obs['ee_pos'], obs['ee_mat'].reshape(3, 3))
        
        # Simulation loop
        start = time.time()
        while time.time() - start < 20.0:
            step_start = time.time()
            
            action = controller.get_action(obs)
            obs, reward, terminated, truncated, info = env.step(action)
            
            viewer.sync()
            
            # Rate limiting
            time_until_next_step = env.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)
