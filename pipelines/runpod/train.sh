#!/bin/bash
# RunPod training script for LeRobot ACT policy

set -e

# Source configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/config.env" ]; then
    source "${SCRIPT_DIR}/config.env"
else
    echo "Error: config.env not found in ${SCRIPT_DIR}"
    exit 1
fi

log_info "=== LeRobot RunPod Training ==="

# Create dataset symlink if it doesn't exist (Idempotent check)
CACHE_DIR="/root/.cache/huggingface/lerobot/${HF_USER}"
TARGET_LINK="${CACHE_DIR}/${DATASET_NAME}"

if [ ! -L "${TARGET_LINK}" ]; then
    log_info "Creating dataset symlink..."
    mkdir -p "${CACHE_DIR}"
    ln -s "${DATASET_DIR}" "${TARGET_LINK}"
    log_success "Symlink created"
else
    log_info "Dataset symlink valid."
fi

# Get current timestamp for job name
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
JOB_NAME="act_so101_runpod_${TIMESTAMP}"
OUTPUT_PATH="${OUTPUT_ROOT}/act_so101_runpod"

log_info "Job name: ${JOB_NAME}"
log_info "Output dir: ${OUTPUT_PATH}"

log_info "Starting training..."

# Start training
lerobot-train \
  --dataset.repo_id="${HF_USER}/${DATASET_NAME}" \
  --policy.type=act \
  --output_dir="${OUTPUT_PATH}" \
  --job_name="${JOB_NAME}" \
  --policy.device=cuda \
  --wandb.enable=true \
  --wandb.project="${WANDB_PROJECT}" \
  --policy.repo_id="${HF_USER}/${POLICY_NAME}" \
  --policy.private=true \
  --policy.push_to_hub=true \
  --batch_size=16 \
  --num_workers=8 \
  --save_freq=5000 \
  --log_freq=100 \
  --steps=100000

log_success "=== Training Complete at $(date) ==="
log_info "Checkpoints saved to: ${OUTPUT_PATH}"
log_info "Next steps:"
log_info "1. Download checkpoints: rsync -avz root@<HOST>:${OUTPUT_PATH} ~/Downloads/"
log_info "2. Stop the pod to avoid charges"
