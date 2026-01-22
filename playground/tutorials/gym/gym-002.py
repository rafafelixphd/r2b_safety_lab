# First simulation: Humanoid locomotion
import os
import r2b
import argparse
import numpy as np
import gymnasium as gym


logger = r2b.get_logger(__name__)

for var in os.getenv("PROJECT_VARS", "").split(" "):
    logger.info(f"{var}: {os.getenv(var, "")}")


def save_model(env):
    env.save("assets/humanoid_model.pkl")

def load_model(env, weights: str):
    env.load(weights)


def main(args):
    try:
        # Create environment with visualization
        env = gym.make(args.env, render_mode="human")

        if args.pretrain:
            if args.weights == "latest":
                args.weights = "assets/humanoid_model.pkl"
            if Path(args.weights).exists():
                load_model(env, args.weights)
            else:
                logger.error(f"Error: {args.weights} not found. Are you in the right directory?")
                logger.error("Continuning training without loaded model.")

        obs, info = env.reset()
        logger.info(f"Observation space: {env.observation_space.shape}")
        logger.info(f"Action space: {env.action_space.shape}")
        # Random policy rollout
        for episode in range(args.episodes):
            obs, info = env.reset()
            total_reward = 0
            avg_reward = []
            for step in range(args.steps):
                action = env.action_space.sample()
                obs, reward, terminated, truncated, info = env.step(action)
                total_reward += reward
                avg_reward.append(reward)
                
                if terminated or truncated:
                    # logger.info(f"{terminated=} | {truncated=}")
                    logger.info(f"Episode {episode + 1}: {total_reward:.2f} reward, {step + 1} steps")
                    break
                if step % int(args.steps // 10) == 0:
                    logger.info(f"「Episode {episode + 1}, {step + 1} steps」 | avg: {np.mean(avg_reward):.2f} | {total_reward:.2f} reward |")

        save_model(env)
        env.close()

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt")
    finally:
        env.close()


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("--env", type=str, default="Humanoid-v5")
    args.add_argument("--episodes", type=int, default=10)
    args.add_argument("--steps", type=int, default=1000)
    args.add_argument("--pretrain", action="store_true")
    args.add_argument("--weights", type=str, default="assets/humanoid_model.pkl")
    args = args.parse_args()

    logger.info(f"Initializing environment: {args}")
    main(args)