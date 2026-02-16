#/bin/bash
source /workspace/bench/r2b_safety_lab/scripts/.localrc

pip install -e .[train]
hf auth login --token $HF_TOKEN

DATASET_DIR:="${DATASET_DIR:-/workspace/dataset/}"
DATASET_NAME:="${DATASET_NAME:-dataset-trial-1}"

if [[ ! -d "$DATASET_DIR/$DATASET_NAME" ]]; then
    echo "Downloading $DATASET_NAME..."
    hf download $HF_USER/$DATASET_NAME --repo-type dataset --local-dir $DATASET_DIR
else
    echo "$DATASET_NAME already exists at $DATASET_DIR/$DATASET_NAME"
fi
