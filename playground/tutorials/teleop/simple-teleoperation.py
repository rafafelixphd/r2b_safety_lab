#!/usr/bin/env python
import os
import time
from r2b.logger import get_logger

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower
from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
from lerobot.teleoperators.so101_leader.so101_leader import SO101Leader

leader_port = os.environ.get("LEADER_PORT")
follower_port = os.environ.get("FOLLOWER_PORT")
leader_id = os.environ.get("LEADER_ID")
follower_id = os.environ.get("FOLLOWER_ID")

logger = get_logger(namespace="teleop")

camera_config = {
    "front": OpenCVCameraConfig(index_or_path=0, width=1920, height=1080, fps=30)
}

robot_config = SO101FollowerConfig(
    port=follower_port,
    id=follower_id,
    cameras=camera_config
)

teleop_config = SO101LeaderConfig(
    port=leader_port,
    id=leader_id,
)

robot = SO101Follower(robot_config)
teleop_device = SO101Leader(teleop_config)
robot.connect()
teleop_device.connect()

while True:
    robot_obs = robot.get_observation()
    logger.info(f"robot_obs: {robot_obs}")
    teleop_action = teleop_device.get_action()
    logger.info(f"teleop_action: {teleop_action}")
    time.sleep(1)
    # robot.send_action(teleop_action)