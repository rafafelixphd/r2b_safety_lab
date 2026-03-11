import argparse
import time
import json
import logging
import torch
from datetime import datetime
from dataclasses import dataclass, field
from lerobot.robots.so_follower import SO101FollowerConfig, SO101Follower
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.cameras.ps4eye.configuration_ps4eye import PS4EyeCameraConfig

from lerobot.configs.policies import PreTrainedConfig
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.utils.control_utils import predict_action
from lerobot.utils.utils import get_safe_torch_device
from lerobot.datasets.utils import build_dataset_frame
from lerobot.utils.constants import OBS_STR
from lerobot.processor import make_default_processors
from lerobot.policies.utils import make_robot_action
from pathlib import Path
from lerobot.datasets.lerobot_dataset import LeRobotDataset

repo_namespace = f'test/{datetime.now().strftime("%Y%m%d_")}{hex(int(time.time()))[3:].upper()}'
@dataclass
class DatasetRecordConfig:
    repo_id: str = repo_namespace
    single_task: str = "task"
    root: str | Path | None = None
    fps: int = 30
    episode_time_s: int | float = 60
    reset_time_s: int | float = 60
    num_episodes: int = 50
    video: bool = True
    push_to_hub: bool = True
    private: bool = False
    tags: list[str] | None = None
    num_image_writer_processes: int = 0
    num_image_writer_threads_per_camera: int = 4
    video_encoding_batch_size: int = 1
    vcodec: str = "libsvtav1"
    streaming_encoding: bool = False
    encoder_queue_maxsize: int = 30
    encoder_threads: int | None = None
    rename_map: dict[str, str] = field(default_factory=dict)


def sprint(value):
    print(f">>>>>>>>>>>>>>>>>>>> {value} >>>>>>>>>>>>>>>>>>>> ")
# We import EnvConfig just to mock the initial features structure if needed
try:
    from lerobot.envs.configs import EnvConfig
except ImportError:
    class EnvConfig:
        pass

# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger("eval")

from r2b import get_logger
logger = get_logger()

def parse_cameras(camera_str):
    if not camera_str:
        return {}
    
    try:
        if "=" in camera_str and camera_str.startswith("{"):
            camera_str = camera_str.replace("=", ":")
        
        import ast
        try:
            cam_dict = ast.literal_eval(camera_str)
        except Exception:
            cam_dict = json.loads(camera_str.replace("'", '"'))
            
        configs = {}
        for cam_name, cam_data in cam_dict.items():
            cam_type = cam_data.get("type", "opencv")
            if cam_type == "opencv":
                configs[cam_name] = OpenCVCameraConfig(**{k: v for k, v in cam_data.items() if k != "type"})
            elif cam_type == "ps4eye":
                configs[cam_name] = PS4EyeCameraConfig(**{k: v for k, v in cam_data.items() if k != "type"})
        return configs
    except Exception as e:
        logger.warning(f"Failed to parse cameras dict: {e}. Defaulting to empty dict.")
        return {}

def fake_env_cfg_from_robot(robot: SO101Follower):
    class MockEnvConfig:
        def __init__(self, rob):
            self.obs_features = rob.observation_features
            self.action_features = rob.action_features
            
            # The env_to_policy_features function expects a specific structure from env_cfg
            self.type = "mock"
            self.features = {**self.obs_features, **self.action_features}

    return MockEnvConfig(robot)

def main():
    parser = argparse.ArgumentParser(description="A highly simplified evaluation script")
    parser.add_argument("--robot.port", dest="port", type=str, required=True, help="Serial port for the follower robot")
    parser.add_argument("--robot.cameras", dest="cameras", type=str, default="{}", help="JSON string for cameras config")
    parser.add_argument("--robot.id", dest="robot_id", type=str, default="v0", help="Robot ID")
    parser.add_argument("--policy.path", dest="policy_path", type=str, required=True, help="Path to compiled policy")
    parser.add_argument("--fps", type=int, default=30, help="Control loop FPS")
    parser.add_argument("--device", type=str, default="mps", help="Torch device")
    
    args, unknown = parser.parse_known_args()
    sprint(f"Policy path: {args.policy_path}")
    logger.info(f"Policy path: {args.policy_path}")
    
    # 1. Initialize Robot Config
    camera_config = parse_cameras(args.cameras)
    robot_config = SO101FollowerConfig(
        port=args.port,
        id=args.robot_id,
        cameras=camera_config
    )
    
    robot = SO101Follower(robot_config)
    
    sprint(f"Robot config: {robot_config}")
    # 2. Connect Robot
    sprint("Connecting to robot...")
    robot.connect()
    sprint(f"Robot connected: {args.robot_id}")
    
    sprint(f"Loading policy from: {args.policy_path}")
    # 3. Load Policy
    sprint(f"Loading policy from {args.policy_path}")
    policy_cfg = PreTrainedConfig.from_pretrained(args.policy_path)
    policy_cfg.pretrained_path = args.policy_path
    sprint("Loading dataset config")
    dataset_config = DatasetRecordConfig()

    from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts
    from lerobot.datasets.pipeline_features import aggregate_pipeline_dataset_features, create_initial_features
    sprint("Making processor")
    _, robot_action_processor, robot_observation_processor = make_default_processors()

    sprint("Making dataset features")
    dataset_features = combine_feature_dicts(
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(
                action=robot.action_features
            ),  # TODO(steven, pepijn): in future this should be come from teleop or policy
            use_videos=False,
        ),
        aggregate_pipeline_dataset_features(
            pipeline=robot_observation_processor,
            initial_features=create_initial_features(observation=robot.observation_features),
            use_videos=dataset_config.video,
        ),
    )

    sprint("Creating dataset")
    dataset = LeRobotDataset.create(
        dataset_config.repo_id,
        dataset_config.fps,
        root=dataset_config.root,
        robot_type=robot.name,
        features=dataset_features,
        use_videos=dataset_config.video,
        image_writer_processes=dataset_config.num_image_writer_processes,
        image_writer_threads=dataset_config.num_image_writer_threads_per_camera * len(robot.cameras),
        batch_encoding_size=dataset_config.video_encoding_batch_size,
        vcodec=dataset_config.vcodec,
        streaming_encoding=dataset_config.streaming_encoding,
        encoder_queue_maxsize=dataset_config.encoder_queue_maxsize,
        encoder_threads=dataset_config.encoder_threads,
    )
    
    sprint("Loading policy")
    # Load pretrained policy
    policy = None if policy_cfg is None else make_policy(policy_cfg, ds_meta=dataset.meta)
    
    sprint(f"{args.device=}")
    device = get_safe_torch_device(args.device)
    sprint(f"{device=}")
    policy.to(device)
    sprint("Policy moved to device")
    policy.eval()
    sprint("Policy set to eval")
    
    # Policy pre/post processors
    sprint("Making pre/post processors")

    sprint(f"{policy_cfg=}")
    sprint(f"{args.policy_path=}")
    sprint(f"{dataset.meta=}")
    preprocessor, postprocessor = make_pre_post_processors(
        policy_cfg=policy_cfg,
        pretrained_path=args.policy_path,
        dataset_stats=None, 
    )
    sprint(f"{preprocessor=}")
    if preprocessor:
        preprocessor.reset()
    sprint(f"{postprocessor=}")
    if postprocessor:
        postprocessor.reset()
    sprint(f"{policy=}")
    policy.reset()


    # 4. Control Loop
    sprint(f"Starting eval loop at {args.fps} FPS. Press Ctrl+C to exit.")
    
    try:
        while True:
            start_t = time.perf_counter()

            sprint(start_t)
            
            # --- Retrieve Robot State ---
            # Returns a dict normally containing 'state' and 'images' (dict with camera arrays)
            obs = robot.get_observation()
            sprint(f"{obs=}")
            obs_processed = robot_observation_processor(obs)
            sprint(f"{obs_processed=}")

            # --- Prepare Input for Model ---
            # Ensures observation matches model feature format
            sprint(f"{policy.config=}, {obs_processed=}, {OBS_STR=}")
            ds_features = {
                "left": {
                    "dtype": "image",  # or map feat.type to dtype string
                    "shape": (3, 800, 1264),
                    # "names": ["left"],
                }, 
                "right": {
                    "dtype": "image",  # or map feat.type to dtype string
                    "shape": (3, 800, 1264),
                }
            }
            print("==========================================================================")
            print("==========================================================================")
            print("==========================================================================")
            print("==========================================================================")
            observation_frame = build_dataset_frame(ds_features, obs_processed, prefix=OBS_STR)
            sprint(f"{observation_frame=}")
            # --- Input to Model ---
            with torch.inference_mode():
                action_values = predict_action(
                    observation=observation_frame,
                    policy=policy,
                    device=device,
                    preprocessor=preprocessor,
                    postprocessor=postprocessor,
                    use_amp=policy.config.use_amp,
                )
            
            # --- Process Response ---
            # Prepare action based on model features config
            act_processed_policy = make_robot_action(action_values, policy.config.features) 
            # Process robot action (e.g. scales config to physical units if needed)
            robot_action_to_send = robot_action_processor((act_processed_policy, obs))
            
            # --- Move the Robot ---
            robot.send_action(robot_action_to_send)
            
            # --- Loop Control ---
            loop_duration = time.perf_counter() - start_t
            sleep_time = max(0.0, (1.0 / args.fps) - loop_duration)
            if sleep_time == 0:
                logger.warning("Control loop lagging behind target FPS!")
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        logger.info("\nEvaluation stopped by user.")
    
    finally:
        logger.info("Disconnecting robot...")
        robot.disconnect()

if __name__ == "__main__":
    logger.info("Hello world")
    print(">>>>>>>>>>>>>>>>>>>> HELLO WORLD >>>>>>>>>>>>>>>>>>>> ")
    main()
