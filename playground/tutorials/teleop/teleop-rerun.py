# teleop-rerun.py
import os
from pathlib import Path
from r2b import get_logger

logger = get_logger(namespace="teleop-rerun")
logger.info("Initializing...")
from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from lerobot.datasets.utils import hw_to_dataset_features
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
from lerobot.teleoperators.so101_leader.so101_leader import SO101Leader

from lerobot.utils.control_utils import init_keyboard_listener
from lerobot.utils.utils import log_say
from lerobot.utils.visualization_utils import init_rerun
from lerobot.scripts.lerobot_record import record_loop
from lerobot.processor import make_default_processors

NUM_EPISODES = 11
FPS = 30
EPISODE_TIME_SEC = 60
RESET_TIME_SEC = 10
TASK_DESCRIPTION = "Picking red cylinder"
cameras= {
    "front": OpenCVCameraConfig(index_or_path=int(os.getenv("FOLLOWER_CAMERA_INDEX")), width=3448, height=808, fps=30)
}
robots = {
    "follower": {
        "port": os.getenv("FOLLOWER_PORT"),
        "id": os.getenv("FOLLOWER_ID"),
        "cameras": cameras,
    },
    "leader": {
        "port": os.getenv("LEADER_PORT"),
        "id": os.getenv("LEADER_ID")
    }
}

logger.info(f"Follower config: {robots['follower']}")
logger.info(f"Leader config: {robots['leader']}")

teleop_config = SO101LeaderConfig(**robots["leader"])
teleop = SO101Leader(teleop_config)
# teleop.connect()
logger.info("Teleoperator connected")

robot_config = SO101FollowerConfig(**robots["follower"])
robot = SO101Follower(robot_config)
# robot.connect()
logger.info("Robot connected")


action_features = hw_to_dataset_features(robot.action_features, "action")
obs_features = hw_to_dataset_features(robot.observation_features, "observation")
dataset_features = {**action_features, **obs_features}

# Create the dataset
dataset_path = Path.home() / ".cache/huggingface/lerobot/"
pid = 1
base_repo_id = "local/data-collection"
repo_id = base_repo_id

while (dataset_path / repo_id ).exists():
    repo_id = f"{base_repo_id}-{pid}"
    pid += 1

logger.info(f"Creating dataset with repo_id: {repo_id}")
dataset = LeRobotDataset.create(
    repo_id=repo_id,
    fps=FPS,
    features=dataset_features,
    robot_type=robot.name,
    use_videos=True,
    image_writer_threads=4,
)

_, events = init_keyboard_listener()
init_rerun(session_name="recording")

# Connect the robot and teleoperator
robot.connect()
teleop.connect()

# Create the required processors
teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

episode_idx = 0
while episode_idx < NUM_EPISODES and not events["stop_recording"]:
    log_say(f"Recording episode {episode_idx + 1} of {NUM_EPISODES}")

    record_loop(
        robot=robot,
        events=events,
        fps=FPS,
        teleop_action_processor=teleop_action_processor,
        robot_action_processor=robot_action_processor,
        robot_observation_processor=robot_observation_processor,
        teleop=teleop,
        dataset=dataset,
        control_time_s=EPISODE_TIME_SEC,
        single_task=TASK_DESCRIPTION,
        display_data=True,
    )

    # Reset the environment if not stopping or re-recording
    if not events["stop_recording"] and (episode_idx < NUM_EPISODES - 1 or events["rerecord_episode"]):
        log_say("Reset the environment")
        record_loop(
            robot=robot,
            events=events,
            fps=FPS,
            teleop_action_processor=teleop_action_processor,
            robot_action_processor=robot_action_processor,
            robot_observation_processor=robot_observation_processor,
            teleop=teleop,
            control_time_s=RESET_TIME_SEC,
            single_task=TASK_DESCRIPTION,
            display_data=True,
        )

    if events["rerecord_episode"]:
        log_say("Re-recording episode")
        events["rerecord_episode"] = False
        events["exit_early"] = False
        dataset.clear_episode_buffer()
        continue

    dataset.save_episode()
    episode_idx += 1

log_say("Stop recording")
robot.disconnect()
teleop.disconnect()
dataset.push_to_hub()