# RunPod Training Setup for LeRobot

## Context

You have a LeRobot training setup for the SO-101 robotic arm with:
- **Dataset**: 61 episodes (~3.1GB) of "Grab the red cylinder" demonstrations
- **Current training**: ACT policy on Apple M3 with batch_size=2 (limited by hardware)
- **Goal**: Train on RunPod GPUs to get 8-10x speedup without uploading to HuggingFace Hub

**Key constraints:**
- First-time RunPod user (need flexibility to learn and iterate)
- Want to avoid sending dataset to LeRobot's HuggingFace server
- Will add more datasets in the future
- Concerned about making decisions that are hard to change later

## Recommended Approach

**Start with ad-hoc setup, then migrate to Docker later** for these reasons:
1. Faster iteration while learning RunPod
2. Easy to experiment with different configurations
3. Simpler debugging without Docker build cycles
4. Clear migration path once workflow is stable

**Dataset transfer via RunPod Network Volume** because:
- One-time upload, reusable across all pods
- Only $0.31/month for 3.1GB storage
- Persists when pods terminate
- Scales easily when adding more datasets

**Use RTX 4090 GPU** for best value:
- ~$0.40/hour vs $1.50+ for A100
- Increase batch_size from 2 → 16-24 (8-12x improvement)
- Complete 100k training steps in ~6 hours vs 55 hours locally
- Total cost: ~$2.50 per full training run

## Implementation Steps

### 1. Initial RunPod Setup (One-time, ~1-2 hours)

**1.1 Create RunPod account and setup:**
```bash
# Sign up at runpod.io
# Add $10-20 in credits
# Generate SSH key (if needed): ssh-keygen -t ed25519
# Add SSH public key to RunPod account settings
```

**1.2 Create Network Volume:**
- Go to RunPod UI → Storage → Create Network Volume
- Name: `lerobot-datasets`
- Size: `20GB` (room for future datasets)
- Region: `US-East-1` (or closest to you)

**1.3 Deploy temporary CPU pod for data upload:**
- Deploy → CPU Pods → Ubuntu 22.04
- Volume: Mount `lerobot-datasets` at `/workspace/datasets`
- Start pod, note SSH connection details

**1.4 Transfer your dataset:**
```bash
# From your Mac terminal:
rsync -avz --progress \
  /Users/rafaelfelix/.cache/huggingface/lerobot/local/data-collection \
  root@<RUNPOD_SSH_HOST>:/workspace/datasets/lerobot/local/

# Verify upload (SSH to pod):
ssh root@<RUNPOD_SSH_HOST>
du -sh /workspace/datasets/lerobot/local/data-collection
ls -la /workspace/datasets/lerobot/local/data-collection/meta/info.json
```

Expected transfer time: ~5-10 minutes for 3.1GB

**1.5 Stop CPU pod** (network volume persists!)

### 2. Create Setup Script (One-time, ~10 min)

Create `setup_lerobot.sh` on your local machine:

```bash
#!/bin/bash
set -e

echo "=== LeRobot RunPod Setup ==="

# Install system dependencies
apt-get update && apt-get install -y \
  git wget ffmpeg libsm6 libxext6 libxrender-dev libgomp1

# Upgrade pip
pip install --upgrade pip

# Install PyTorch for CUDA 12.1
pip install torch==2.7.1 torchvision==0.22.1 \
  --index-url https://download.pytorch.org/whl/cu121

# Install LeRobot and dependencies
pip install lerobot==0.4.2 python-dotenv==1.2.1 wandb moviepy==2.2.1 av==15.1.0

# Verify installation
python -c "import torch; print(f'PyTorch {torch.__version__}, CUDA: {torch.cuda.is_available()}, GPUs: {torch.cuda.device_count()}')"
python -c "import lerobot; print(f'LeRobot version: {lerobot.__version__}')"

echo "=== Setup Complete ==="
```

### 3. First Training Run (Repeatable workflow)

**3.1 Deploy GPU pod:**
- Deploy → GPU Pods → RTX 4090 (or RTX A5000)
- Template: RunPod PyTorch 2.7.1
- Volume: Mount `lerobot-datasets` at `/workspace/datasets`
- Container Disk: `30GB`
- Start pod

**3.2 SSH and run setup:**
```bash
ssh root@<RUNPOD_SSH_HOST>

# Upload and run setup script (first time only)
# Upload setup_lerobot.sh via scp or paste contents
bash setup_lerobot.sh

# Configure WandB
wandb login
# Paste your WandB API key
```

**3.3 Link dataset to expected location:**
```bash
# Create symlink so LeRobot finds the dataset
mkdir -p /root/.cache/huggingface/lerobot/local
ln -s /workspace/datasets/lerobot/local/data-collection \
      /root/.cache/huggingface/lerobot/local/data-collection

# Verify dataset loads correctly:
python -c "from lerobot.common.datasets.factory import make_dataset; \
           ds = make_dataset('local/data-collection'); \
           print(f'Dataset loaded: {len(ds)} samples, {ds.meta.total_episodes} episodes')"
```

**3.4 Start training in tmux (for SSH stability):**
```bash
# Start tmux session
tmux new -s training

# Run training with CUDA and larger batch size
lerobot-train \
  --dataset.repo_id="local/data-collection" \
  --policy.type=act \
  --output_dir=/workspace/outputs/train/act_so101_runpod \
  --job_name=act_so101_runpod_$(date +%Y%m%d_%H%M%S) \
  --policy.device=cuda \
  --wandb.enable=true \
  --wandb.project=lerobot-so101 \
  --policy.repo_id="local/my_policy" \
  --batch_size=16 \
  --num_workers=8 \
  --save_freq=10000 \
  --log_freq=100 \
  --steps=100000

# Detach from tmux: Press Ctrl+B, then press D
# Re-attach later: tmux attach -t training
```

**Key changes from your local training:**
- `--policy.device=cuda` (was `mps` on Mac)
- `--batch_size=16` (was `2` on Mac) = **8x larger**
- `--num_workers=8` (better GPU utilization)
- `--save_freq=10000` (checkpoints every 10k steps)

**3.5 Monitor training:**
- Open WandB dashboard in browser
- Check GPU usage: `nvidia-smi` (in separate SSH session or tmux pane)
- Expected: ~4-6 steps/second (vs ~0.5 on Mac)

**3.6 Download checkpoints:**

After training completes (or periodically during training):

```bash
# From your Mac:
rsync -avz --progress \
  root@<RUNPOD_SSH_HOST>:/workspace/outputs/train/act_so101_runpod \
  /Users/rafaelfelix/Projects/r2b/outputs/runpod_training/
```

Alternative: Copy to network volume for safekeeping:
```bash
# On RunPod pod:
cp -r /workspace/outputs/train/act_so101_runpod \
      /workspace/datasets/checkpoints/act_so101_$(date +%Y%m%d)
```

**3.7 Stop the pod** (via RunPod UI or CLI to avoid charges)

### 4. Cost Optimization

**Use spot instances** (50-70% cheaper):
- In RunPod UI, select "Spot" instead of "On-Demand"
- Risk: Pod can be interrupted (rare for RTX 4090)
- Mitigation: Checkpoints save every 10k steps, so max loss is ~30 min of training
- Cost: ~$0.20/hr vs $0.40/hr = **$1.25 per full training run**

**Stop pods immediately after training:**
- Don't leave pods running when idle
- Network volume persists, so you can resume anytime

**Monitor WandB remotely:**
- No need to keep SSH session open
- Check loss curves after first 10k steps to validate config

## Critical Files

| File | Purpose |
|------|---------|
| [~/.cache/huggingface/lerobot/local/data-collection/](file:///Users/rafaelfelix/.cache/huggingface/lerobot/local/data-collection/) | Dataset to upload to RunPod |
| [r2b_safety_lab/requirements.txt](file:///Users/rafaelfelix/Projects/r2b/r2b_safety_lab/requirements.txt) | Dependencies reference |
| [r2b_safety_lab/outputs/train/act_so101_test/wandb/.../config.yaml](file:///Users/rafaelfelix/Projects/r2b/r2b_safety_lab/outputs/train/act_so101_test/wandb/run-20260208_204256-vtgae1vp/files/config.yaml) | Training config reference |
| [r2b_safety_lab/tests/teleop.sh](file:///Users/rafaelfelix/Projects/r2b/r2b_safety_lab/tests/teleop.sh) | Local training command reference |

## Future: Adding More Datasets

When you add new datasets or use others' datasets:

**Option A: Upload your own datasets (same as initial setup):**
```bash
# From Mac:
rsync -avz ~/.cache/huggingface/lerobot/local/task-b-dataset \
  root@<RUNPOD_SSH_HOST>:/workspace/datasets/lerobot/local/
```

**Option B: Download from HuggingFace Hub (when using others' datasets):**
```bash
# On RunPod pod, set cache location to network volume:
export HF_HOME="/workspace/datasets/.cache/huggingface"

# Download dataset (caches to network volume):
python -c "from lerobot.common.datasets.factory import make_dataset; \
           ds = make_dataset('lerobot/aloha_sim_insertion_human')"

# Dataset persists on network volume for future pods!
```

**Multi-dataset training:**
```bash
# Train on multiple datasets:
lerobot-train \
  --dataset.repo_id="local/data-collection,local/task-b-dataset" \
  ...
```

## Future: Migrating to Docker

**When to migrate** (after 3-5 successful runs):
1. You've settled on a stable configuration
2. You want faster pod startup (skip setup script)
3. You want guaranteed reproducibility

**Migration is simple:**
1. Create Dockerfile based on working setup
2. Build and push to Docker Hub
3. Deploy RunPod pod with custom Docker image
4. Same workflow, just faster startup

I'll help you create the Dockerfile when you're ready.

## Verification

After completing setup, verify:

**✓ Dataset uploaded correctly:**
```bash
# On RunPod pod:
ls -lh /workspace/datasets/lerobot/local/data-collection/meta/info.json
du -sh /workspace/datasets/lerobot/local/data-collection
```

**✓ Dataset loads in LeRobot:**
```python
from lerobot.common.datasets.factory import make_dataset
ds = make_dataset('local/data-collection')
assert len(ds) > 0, "Dataset empty!"
assert ds.meta.total_episodes == 61, "Wrong episode count!"
print("✓ Dataset verified")
```

**✓ Training starts without errors:**
```bash
# Check first 100 steps complete
# Monitor WandB for loss curves
# Check GPU utilization: nvidia-smi shows >80% usage
```

**✓ Checkpoints save correctly:**
```bash
ls -lh /workspace/outputs/train/act_so101_runpod/checkpoints/
# Should show folders: 010000/, 020000/, etc.
```

**✓ Download checkpoints successfully:**
```bash
# On Mac:
ls -lh /Users/rafaelfelix/Projects/r2b/outputs/runpod_training/
# Should contain downloaded checkpoint folders
```

## Expected Results

- **Training time**: 100k steps in ~6 hours (vs 55 hours locally) = **9x faster**
- **Cost per run**: ~$2.50 on-demand, ~$1.25 on spot instances
- **Batch size**: 16-24 (vs 2 locally) = **8-12x larger batches**
- **GPU utilization**: >80% (vs limited on M3)
- **Flexibility**: Easy to try different hyperparameters, multiple runs per day

## Quick Reference

**Start training on RunPod:**
1. Deploy RTX 4090 pod with `lerobot-datasets` volume
2. SSH in, run `bash setup_lerobot.sh`
3. Create dataset symlink
4. `wandb login`
5. Start training in tmux
6. Monitor on WandB, check `nvidia-smi`
7. Download checkpoints when done
8. Stop pod

**Costs:**
- Network volume: $2/month (20GB)
- GPU pod: $0.40/hr (on-demand) or $0.20/hr (spot)
- Per training run: ~$2.50 (or ~$1.25 spot)

**Troubleshooting:**
- Dataset not found → Check symlink exists
- OOM error → Reduce `--batch_size=8` or `--num_workers=4`
- Slow training → Increase `--num_workers=12`
- Pod interrupted → Resume from latest checkpoint in `checkpoints/` folder
