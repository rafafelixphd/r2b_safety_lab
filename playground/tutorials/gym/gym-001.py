# MuJoCo: 10 lines to first simulation
import os
import r2b
# import time
import argparse
import gymnasium as gym
import numpy as np
from gymnasium.wrappers import RecordVideo, RecordEpisodeStatistics
from pathlib import Path
import traceback

logger = r2b.get_logger(__name__)

ASSET_DIR = Path(__file__).parent.parent / "assets"

for var in os.getenv("PROJECT_VARS", "").split(" "):
    logger.info(f"{var}: {os.getenv(var, "")}")


def record_video(env, args):
    env = RecordVideo(
        env,
        video_folder=args.video_path,
        name_prefix="eval",
        # episode_trigger=lambda x: True
    )
    env = RecordEpisodeStatistics(env, buffer_length=args.steps)
    return env

def main(args):
    try:
        if args.record:
            env = gym.make(args.env, render_mode="rgb_array")
            env = record_video(env, args)
        else:
            env = gym.make(args.env, render_mode="human")

        observation, info = env.reset()
        logger.info(f"Starting observation: {observation}")
        logger.info(f"Starting info: {info}")
        total_reward, total_episodes, episode = 0, 0, 0

        avg_reward = []
        for step in range(args.steps):
            env.reset()
            episode_over = False
            while not episode_over:
                episode += 1
                action = env.action_space.sample()
                observation, reward, terminated, truncated, info = env.step(action)

                total_reward += reward
                avg_reward.append(reward)
                episode_over = terminated or truncated
                
            if step % (max(1, args.steps // 10)) == 0:
                logger.info(f"「{step+1}/{args.steps}」「episodes: {episode}」 | reward: {total_reward} | avg: {np.mean(avg_reward):.2f}")

            total_episodes += episode
            episode_over = False
        
        logger.info(f"Total episodes: {total_episodes}")
        logger.info(f"Total reward: {np.mean(avg_reward):.2f}")
        env.close()

    except KeyboardInterrupt:
        logger.critical("KeyboardInterrupt")
        env.close()
        return
    
    except Exception as e:
        logger.error(f"Error: {e}")
        logger.error(traceback.print_exc())
        env.close()
        return



if __name__ == "__main__":
    arg = argparse.ArgumentParser()
    arg.add_argument("--steps", type=int, default=50)
    arg.add_argument("--env", type=str, default="CartPole-v1")
    arg.add_argument("--record", action="store_true")
    arg.add_argument("--video_path", type=str, default=f"{ASSET_DIR}/videos/example.mp4")
    args = arg.parse_args()


    
    logger.info("Running...")
    logger.info(f"{args=}")

    main(args)
