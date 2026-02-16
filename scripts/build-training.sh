#/bin/bash
#
# bash /workspace/bench/r2b_safety_lab/scripts/init.sh
# bash /workspace/bench/r2b_safety_lab/scripts/build-training.sh
#

if [ ! -d "/workspace/bench/r2b_safety_lab" ]; then
    echo "Error: /workspace/bench/r2b_safety_lab not found"
    exit 1
fi

source /workspace/bench/r2b_safety_lab/scripts/.localrc


echo "Installing dependencies..."
{
    cd /workspace/bench/r2b_safety_lab &&
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
DATASET_DIR:="${DATASET_DIR:-/workspace/dataset/}"
DATASET_NAME:="${DATASET_NAME:-dataset-trial-1}"

if [[ ! -d "$DATASET_DIR/$DATASET_NAME" ]]; then
    echo "Downloading $DATASET_NAME..."
    hf download $HF_USER/$DATASET_NAME --repo-type dataset --local-dir $DATASET_DIR
else
    echo "$DATASET_NAME already exists at $DATASET_DIR/$DATASET_NAME"
fi


echo "Ready to train!"