#!/bin/bash
# RunPod training script for LeRobot ACT policy
# Run this on your RunPod GPU pod after setup is complete

set -e

source ~/.localrc

echo "=== LeRobot RunPod Training ==="

DATASET_NAME:=${DATASET_NAME:-dataset-trial-1}

# Create dataset symlink if it doesn't exist
if [ ! -L "/root/.cache/huggingface/lerobot/$HF_USER/$DATASET_NAME" ]; then
    echo "Creating dataset symlink..."
    mkdir -p /root/.cache/huggingface/lerobot/$HF_USER
    ln -s /workspace/datasets/$DATASET_NAME \
          /root/.cache/huggingface/lerobot/$HF_USER/$DATASET_NAME
    echo "✓ Symlink created"
fi

# Get current timestamp for job name
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo ""
echo "Starting training at $(date)"
echo "Job name: act_so101_runpod_${TIMESTAMP}"
echo ""

OUTPUT_DIR=${OUTPUT_DIR:-/workspace/outputs/train/act_so101_runpod}
RESUME_TRAIN=false
if [ -d "$OUTPUT_DIR" ]; then
  RESUME_TRAIN=true
fi

WANDB_API_KEY=$WANDB_API_KEY wandb login

# Start training
lerobot-train \
  --dataset.repo_id="$HF_USER/$DATASET_NAME" \
  --policy.type=act \
  --output_dir=$OUTPUT_DIR \
  --job_name=act_so101_runpod_${TIMESTAMP} \
  --policy.device=cuda \
  --wandb.enable=true \
  --wandb.project=lerobot-so101 \
  --policy.repo_id="$HF_USER/policy-1" \
  --policy.private=true \
  --policy.push_to_hub=true \
  --batch_size=16 \
  --num_workers=8 \
  --save_freq=5000 \
  --log_freq=100 \
  --steps=100000 ${RESUME_TRAIN:+"--resume"}

echo ""
echo "=== Training Complete at $(date) ==="
echo "Checkpoints saved to: /workspace/outputs/train/act_so101_runpod"
echo ""
echo "Next steps:"
echo "1. Download checkpoints: rsync -avz root@<HOST>:/workspace/outputs/train/act_so101_runpod ~/Downloads/"
echo "2. Stop the pod to avoid charges"
