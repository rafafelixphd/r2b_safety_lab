#!/bin/bash
#
# bash /workspace/bench/r2b_safety_lab/scripts/init.sh
# bash /workspace/bench/r2b_safety_lab/scripts/build-training.sh
#

source ~/.localrc

export REPO_DIR:="${REPO_DIR:-/workspace/bench/r2b_safety_lab}"

if [ ! -d "${REPO_DIR}" ]; then
    echo "Error: ${REPO_DIR} not found"
    exit 1
fi

echo "Installing dependencies..."
{
    cd ${REPO_DIR} &&
    pip install -e .[train]
} || {
    echo "[Error] Error installing dependencies"
    exit 1
}


echo "Logging in to Hugging Face..."
{
    hf auth login --token $HF_TOKEN
} || {
    echo "[Error] Error logging in to Hugging Face"
    exit 1
}
    
echo "Checking dataset"
export DATASET_DIR:="${DATASET_DIR:-/workspace/dataset/}"
export DATASET_NAME:="${DATASET_NAME:-dataset-trial-1}"

if [[ ! -d "$DATASET_DIR/$DATASET_NAME" ]]; then
    echo "Downloading $DATASET_NAME..."
    hf download $HF_USER/$DATASET_NAME --repo-type dataset --local-dir $DATASET_DIR
else
    echo "$DATASET_NAME already exists at $DATASET_DIR/$DATASET_NAME"
fi

echo "Linking dataset to Hugging Face cache..."
mkdir --parents /root/.cache/huggingface/lerobot/$HF_USER/
ln -s /workspace/dataset/* /root/.cache/huggingface/lerobot/$HF_USER/
echo "✓ Symlink created"

echo "Ready to train!"