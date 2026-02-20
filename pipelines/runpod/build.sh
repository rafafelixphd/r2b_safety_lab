#!/bin/bash
#
# bash /workspace/bench/r2b_safety_lab/scripts/build-training.sh

set -e

# Source configuration
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [ -f "${SCRIPT_DIR}/config.env" ]; then
    source "${SCRIPT_DIR}/config.env"
else
    echo "Error: config.env not found in ${SCRIPT_DIR}"
    exit 1
fi



# Validate Repo Dir
if [ ! -d "${REPO_DIR}" ]; then
    log_error "Repository not found at ${REPO_DIR}"
    exit 1
fi

deactivate
log_info "Checking system dependencies..."
if ! command -v ffmpeg &> /dev/null; then
    log_info "Installing ffmpeg..."
    # Ensure add-apt-repository exists
    if ! command -v add-apt-repository &> /dev/null; then
        apt-get update && apt-get install -y software-properties-common
    fi
    /usr/bin/add-apt-repository -y ppa:ubuntuhandbook1/ffmpeg7
    apt-get update
    apt-get install -y ffmpeg
    log_success "ffmpeg installed."
fi

# Ensure ~/.localrc is sourced if it exists (for interactive checks)
[ -f ~/.localrc ] && source ~/.localrc

log_info "Installing Python dependencies..."
cd "${REPO_DIR}"
pip install -e .[train]
log_success "Dependencies installed."

log_info "Logging in to Hugging Face..."
if [ -n "$HF_TOKEN" ]; then
    hf auth login --token "$HF_TOKEN" --add-to-git-credential
else
    log_warn "HF_TOKEN not set. Skipping automated login."
fi

log_info "Logging in to Weights & Biases..."
if [ -n "$WANDB_API_KEY" ]; then
    WANDB_API_KEY=$WANDB_API_KEY wandb login
else
    log_warn "WANDB_API_KEY not set. Skipping automated login."
fi

log_info "Checking dataset: ${DATASET_NAME}..."
if [[ ! -d "${DATASET_DIR}" ]]; then
    log_info "Downloading ${DATASET_NAME}..."
    hf download "${HF_USER}/${DATASET_NAME}" --repo-type dataset --local-dir "${DATASET_DIR}"
    log_success "Dataset downloaded."
else
    log_info "Dataset already exists at ${DATASET_DIR}"
fi

log_info "Linking dataset to Hugging Face cache..."
CACHE_DIR="/root/.cache/huggingface/lerobot/${HF_USER}"
mkdir -p "${CACHE_DIR}"
# Remove existing link/dir if it conflicts to ensure clean state
rm -rf "${CACHE_DIR}/${DATASET_NAME}"
ln -s "${DATASET_DIR}" "${CACHE_DIR}/${DATASET_NAME}"
log_success "Symlink created."

log_success "Build complete! Ready to train."
log_info "Next: bash ${SCRIPT_DIR}/train.sh"