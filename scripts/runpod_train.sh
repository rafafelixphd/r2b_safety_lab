#!/bin/bash
# RunPod training script for LeRobot ACT policy
# Run this on your RunPod GPU pod after setup is complete

set -e

echo "=== LeRobot RunPod Training ==="

# Create dataset symlink if it doesn't exist
if [ ! -L "/root/.cache/huggingface/lerobot/local/data-collection" ]; then
    echo "Creating dataset symlink..."
    mkdir -p /root/.cache/huggingface/lerobot/local
    ln -s /workspace/datasets/lerobot/local/data-collection \
          /root/.cache/huggingface/lerobot/local/data-collection
    echo "✓ Symlink created"
fi

# Verify dataset is accessible
echo "Verifying dataset..."
python -c "from lerobot.common.datasets.factory import make_dataset; \
           ds = make_dataset('local/data-collection'); \
           print(f'✓ Dataset loaded: {len(ds)} samples, {ds.meta.total_episodes} episodes')"

# Get current timestamp for job name
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

echo ""
echo "Starting training at $(date)"
echo "Job name: act_so101_runpod_${TIMESTAMP}"
echo ""

# Start training
lerobot-train \
  --dataset.repo_id="local/data-collection" \
  --policy.type=act \
  --output_dir=/workspace/outputs/train/act_so101_runpod \
  --job_name=act_so101_runpod_${TIMESTAMP} \
  --policy.device=cuda \
  --wandb.enable=true \
  --wandb.project=lerobot-so101 \
  --policy.repo_id="local/my_policy" \
  --batch_size=16 \
  --num_workers=8 \
  --save_freq=10000 \
  --log_freq=100 \
  --steps=100000

echo ""
echo "=== Training Complete at $(date) ==="
echo "Checkpoints saved to: /workspace/outputs/train/act_so101_runpod"
echo ""
echo "Next steps:"
echo "1. Download checkpoints: rsync -avz root@<HOST>:/workspace/outputs/train/act_so101_runpod ~/Downloads/"
echo "2. Stop the pod to avoid charges"
