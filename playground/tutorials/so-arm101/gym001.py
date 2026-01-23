# gym001.py

import os
import r2b
import argparse
import gymnasium as gym
import gym_aloha

import numpy as np
from gymnasium.wrappers import RecordVideo, RecordEpisodeStatistics
from pathlib import Path
import traceback
from stable_baselines3 import PPO

logger = r2b.get_logger(__name__)
ASSET_DIR = Path(__file__).parent.parent / "assets"

def record_video(env, args):
    env = RecordVideo(
        env,
        video_folder=args.video_path,
        name_prefix="eval",
    )
    env = RecordEpisodeStatistics(env, buffer_length=args.steps)
    return env

def main(args):
    env = None
    try:
        if args.record:
            logger.warning("Recording video...")
            env = gym.make(args.env, render_mode="rgb_array")
            env = record_video(env, args)
        else:
            logger.warning("Rendering in human mode...")
            env = gym.make(args.env, render_mode="human")
        

        # R2B Logic: Load existing brain or create new one
        model = PPO("MultiInputPolicy", env, verbose=1)

        model_path = Path(args.model_path)
        if model_path.with_suffix(".zip").exists():
            if args.pretrained:
                logger.info(f"Loading existing model from {model_path}")
                model = PPO.load(model_path, env=env)
            else:
                logger.warning(f"Training from scratch... good luck")

        avg_reward, total_reward, max_actions = [], 0, 0
        for step in range(args.steps):
            episode_over, num_actions = False, 0
    
            env.reset()
            observation, info = env.reset()

            while not episode_over:
                # action, _states = model.predict(observation, deterministic=True)
                action = env.action_space.sample()
                observation, reward, terminated, truncated, info = env.step(action)
                episode_over = terminated or truncated
                avg_reward.append(reward)
                total_reward += reward

                if terminated or truncated:
                    observation, info = env.reset()
                num_actions += 1

            if (num_actions > max_actions) or (step % (max(1, args.steps // 10)) == 0):
                max_actions = max(num_actions, max_actions)
                logger.info(f"「{step+1}/{args.steps}」| rewards: {total_reward:.2f} | avg: {np.mean(avg_reward):.2f} | max_actions: {max_actions}")
                logger.info(f"Saving the model.. {model_path}")
                model.save(model_path)

        env.close()

    except Exception as e:
        logger.critical(f"Error: {e}")
        logger.error(f"{traceback.print_exc()=}")
        if env: env.close()

if __name__ == "__main__":
    arg = argparse.ArgumentParser()
    arg.add_argument("--steps", type=int, default=5000)
    arg.add_argument("--env", type=str, default="gym_aloha/AlohaInsertion-v0")
    arg.add_argument("--train", action="store_true")
    arg.add_argument("--pretrained", action="store_true")
    arg.add_argument("--model_path", type=str, default=f"{ASSET_DIR}/weights/r2b_ppo_model")
    arg.add_argument("--record", action="store_true")
    arg.add_argument("--video_path", type=str, default=str(ASSET_DIR / "videos"))
    args = arg.parse_args()
    print("=="*30)
    logger.info(f"{args=}")
    print("=="*30)
    main(args)