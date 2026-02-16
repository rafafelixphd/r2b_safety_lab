#!/bin/bash
set -e

echo "=== LeRobot RunPod Setup ==="
echo "Starting setup at $(date)"

# Install system dependencies
echo "Installing system dependencies..."
apt-get update && apt-get install -y \
  git \
  wget \
  ffmpeg \
  libsm6 \
  libxext6 \
  libxrender-dev \
  libgomp1

# Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# Install PyTorch for CUDA 12.1
echo "Installing PyTorch with CUDA 12.1 support..."
pip install torch==2.7.1 torchvision==0.22.1 \
  --index-url https://download.pytorch.org/whl/cu121

# Install LeRobot and dependencies
echo "Installing LeRobot and dependencies..."
pip install lerobot==0.4.2 \
  python-dotenv==1.2.1 \
  wandb \
  moviepy==2.2.1 \
  av==15.1.0

# Verify installations
echo ""
echo "=== Verifying Installation ==="
python -c "import torch; print(f'✓ PyTorch {torch.__version__}')"
python -c "import torch; print(f'✓ CUDA available: {torch.cuda.is_available()}')"
python -c "import torch; print(f'✓ CUDA devices: {torch.cuda.device_count()}')"
python -c "import torch; print(f'✓ CUDA version: {torch.version.cuda}')" || true
python -c "import lerobot; print(f'✓ LeRobot {lerobot.__version__}')"
python -c "import wandb; print(f'✓ WandB {wandb.__version__}')"

echo ""
echo "=== Setup Complete at $(date) ==="
echo ""
echo "Next steps:"
echo "1. Run: wandb login"
echo "2. Create dataset symlink (see plan)"
echo "3. Start training!"
