#!/bin/bash
#
# bash /workspace/bench/r2b_safety_lab/scripts/init.sh
# bash /workspace/bench/r2b_safety_lab/scripts/build-training.sh
#
# Current docker: runpod/pytorch:2.2.0-py3.10-cuda12.1.1-devel-ubuntu22.04

apt-get update

if ! command -v htop &> /dev/null; then
    apt-get install -y htop
fi

echo "Copying SSH keys..."
{
    cp /workspace/.ssh/id_ed25519 /root/.ssh/id_ed25519
    chmod 600 /root/.ssh/id_ed25519
    if [ ! -f "/root/.ssh/id_ed25519" ]; then
        echo "[Error] SSH key not found" && exit 1
    fi
} || {
    echo "[Error] Error copying SSH keys"
    exit 1
}

echo "Cloning r2b_safety_lab..."

REPO_GIT_REF=${REPO_GIT_REF:-git@github.com:rafafelixphd/r2b_safety_lab.git}
REPO_DIR=${REPO_DIR:-/workspace/bench/r2b_safety_lab}

if [ ! -d "${REPO_DIR}" ]; then
    git clone ${REPO_GIT_REF} ${REPO_DIR} --branch dev
else
    echo "r2b_safety_lab already exists at ${REPO_DIR}"
fi

echo "Updating r2b_safety_lab..."
{
    source ${REPO_DIR}/scripts/.localrc &&
    cd ${REPO_DIR} &&
    git pull
} || {
    echo "Error updating r2b_safety_lab"
}

if [ ! -d "/workspace/envs/.env" ]; then
    echo "Creating .env file..."
    python -m venv /workspace/envs/.env
    source /workspace/envs/.env/bin/activate
    pip install --upgrade pip
fi

cp ${REPO_DIR}/scripts/.localrc ~/.localrc
echo "Ready to work!"