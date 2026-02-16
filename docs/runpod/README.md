# RunPod Training Guide for LeRobot

Quick reference for training your SO-101 LeRobot policies on RunPod.

## Overview

- **Dataset**: 61 episodes (~3.1GB) stored on RunPod Network Volume
- **Policy**: ACT (Action Chunking Transformers)
- **Recommended GPU**: RTX 4090 (~$0.40/hr)
- **Training time**: ~6 hours for 100k steps
- **Cost per run**: ~$2.50 on-demand, ~$1.25 spot

## Quick Start

### 1. Initial Setup (One-time)

**Create RunPod Account:**
- Sign up at [runpod.io](https://runpod.io)
- Add $10-20 credits
- Add your SSH public key to account settings

**Create Network Volume:**
1. Go to RunPod → Storage → Create Network Volume
2. Name: `lerobot-datasets`
3. Size: `20GB`
4. Region: `US-East-1`

**Upload Dataset:**
```bash
# Deploy temporary CPU pod with network volume mounted at /workspace/datasets
# Then from your Mac:
rsync -avz --progress \
  ~/.cache/huggingface/lerobot/local/data-collection \
  root@<RUNPOD_SSH_HOST>:/workspace/datasets/lerobot/local/

# Verify (on RunPod pod):
du -sh /workspace/datasets/lerobot/local/data-collection
# Should show ~3.1G
```

### 2. Training Workflow (Repeatable)

**Deploy GPU Pod:**
- Template: RunPod PyTorch 2.7.1
- GPU: RTX 4090 (or RTX A5000)
- Volume: Mount `lerobot-datasets` at `/workspace/datasets`
- Container Disk: 30GB
- (Optional) Use Spot instance for 50% savings

**Run Setup Script (first time only):**
```bash
ssh root@<RUNPOD_SSH_HOST>

# Upload setup script
# From Mac: scp r2b_safety_lab/scripts/setup_lerobot_runpod.sh root@<HOST>:/workspace/

# Run setup
bash /workspace/setup_lerobot_runpod.sh

# Configure WandB
wandb login
# Paste your API key
```

**Start Training:**
```bash
# Option A: Run in tmux (recommended)
tmux new -s training
bash /workspace/runpod_train.sh
# Detach: Ctrl+B, then D

# Option B: Run directly
bash /workspace/runpod_train.sh
```

**Monitor Training:**
- Open WandB dashboard in browser
- Check GPU usage: `nvidia-smi`
- Expected: 4-6 steps/sec (vs 0.5 on Mac)

**Download Checkpoints:**
```bash
# From your Mac (while training or after):
rsync -avz --progress \
  root@<RUNPOD_SSH_HOST>:/workspace/outputs/train/act_so101_runpod \
  ~/Projects/r2b/outputs/runpod_training/
```

**Stop Pod** when done to avoid charges!

## Files

| File | Purpose |
|------|---------|
| `scripts/setup_lerobot_runpod.sh` | Install dependencies on RunPod |
| `scripts/runpod_train.sh` | Training script for RunPod |
| `docs/runpod/README.md` | This guide |
| `docs/runpod/scratchpad.md` | Planning notes |

## Training Command Breakdown

The training command differs from local training:

| Parameter | Local (Mac) | RunPod | Notes |
|-----------|-------------|--------|-------|
| `--policy.device` | `mps` | `cuda` | Apple Silicon → NVIDIA GPU |
| `--batch_size` | `2` | `16` | 8x larger batches |
| `--num_workers` | `4` | `8` | Better GPU utilization |
| `--save_freq` | `20000` | `10000` | More frequent checkpoints |

**Performance gains:**
- **Batch size**: 2 → 16 (8x larger)
- **Speed**: 0.5 → 4-6 steps/sec (8-12x faster)
- **Time**: 55 hours → 6 hours (9x faster)

## Cost Optimization

**Use Spot Instances:**
- 50-70% cheaper (~$0.20/hr vs $0.40/hr)
- Low interruption risk for RTX 4090
- Checkpoints save every 10k steps (max 30 min loss)

**Stop Pods Immediately:**
- Network volume persists
- Only pay for active GPU time
- Monitor via WandB, no need to keep SSH open

**Validate Early:**
- Check loss curves after 10k steps
- Stop if not learning before full 100k run

## Troubleshooting

**Dataset not found:**
```bash
# Verify network volume is mounted
ls /workspace/datasets/lerobot/local/data-collection

# Create symlink manually
mkdir -p /root/.cache/huggingface/lerobot/local
ln -s /workspace/datasets/lerobot/local/data-collection \
      /root/.cache/huggingface/lerobot/local/data-collection
```

**Out of memory (OOM):**
```bash
# Reduce batch size in runpod_train.sh
--batch_size=8  # Instead of 16
```

**Slow training:**
```bash
# Increase workers in runpod_train.sh
--num_workers=12  # Instead of 8
```

**Pod interrupted (spot):**
```bash
# Find latest checkpoint
ls /workspace/outputs/train/act_so101_runpod/checkpoints/

# Resume from checkpoint (add to lerobot-train command)
--checkpoint_path=/workspace/outputs/.../checkpoints/040000/pretrained_model.safetensors \
--resume=true
```

## Future: Adding More Datasets

**Upload your own datasets:**
```bash
# Same process as initial dataset
rsync -avz ~/.cache/huggingface/lerobot/local/new-dataset \
  root@<HOST>:/workspace/datasets/lerobot/local/
```

**Use HuggingFace datasets:**
```bash
# On RunPod pod, cache to network volume:
export HF_HOME="/workspace/datasets/.cache/huggingface"

# Download dataset (persists on network volume)
python -c "from lerobot.common.datasets.factory import make_dataset; \
           ds = make_dataset('lerobot/aloha_sim_insertion_human')"
```

**Multi-dataset training:**
```bash
# Modify runpod_train.sh:
--dataset.repo_id="local/data-collection,local/new-dataset"
```

## Reference

Full implementation plan: `~/.claude/plans/eager-crafting-toucan.md`

**Costs:**
- Network volume: $2/month (20GB)
- RTX 4090: $0.40/hr (on-demand), $0.20/hr (spot)
- Per training run: $2.50 (on-demand), $1.25 (spot)

**Performance:**
- Local (M3): 55 hours, batch_size=2
- RunPod (4090): 6 hours, batch_size=16
- **Speedup: 9x faster**
