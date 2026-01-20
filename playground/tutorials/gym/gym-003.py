import os
import r2b
import argparse
import gymnasium as gym
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
        render_mode = "rgb_array" if args.record else "human"
        env = gym.make(args.env, render_mode=render_mode)
        
        if args.record:
            env = record_video(env, args)

        # R2B Logic: Load existing brain or create new one
        model_path = Path(args.model_path)
        if model_path.with_suffix(".zip").exists():
            logger.info(f"Loading existing model from {model_path}")
            model = PPO.load(model_path, env=env)
        else:
            logger.info("Initializing new PPO model")
            model = PPO("MlpPolicy", env, verbose=1)

        if args.train:
            logger.info(f"Training for {args.steps} steps...")
            model.learn(total_timesteps=args.steps)
            model.save(model_path)
            logger.info(f"Model saved to {model_path}")

        observation, info = env.reset()
        avg_reward = []
        
        for step in range(args.steps):
            # Use the model to predict the action instead of random sampling
            action, _states = model.predict(observation, deterministic=True)
            observation, reward, terminated, truncated, info = env.step(action)
            
            avg_reward.append(reward)
            if terminated or truncated:
                observation, info = env.reset()
                
            if step % (max(1, args.steps // 10)) == 0:
                logger.info(f"「{step+1}/{args.steps}」| avg_reward: {np.mean(avg_reward):.2f}")

        env.close()

    except Exception as e:
        logger.error(f"Error: {e}")
        traceback.print_営業()
        if env: env.close()

if __name__ == "__main__":
    arg = argparse.ArgumentParser()
    arg.add_argument("--steps", type=int, default=5000)
    arg.add_argument("--env", type=str, default="InvertedPendulum-v4")
    arg.add_argument("--train", action="store_true")
    arg.add_argument("--model_path", type=str, default="r2b_ppo_model")
    arg.add_argument("--record", action="store_true")
    arg.add_argument("--video_path", type=str, default=str(ASSET_DIR / "videos"))
    args = arg.parse_args()
    main(args)