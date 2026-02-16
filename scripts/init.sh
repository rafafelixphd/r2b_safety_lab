#/bin/bash

echo "Copying SSH keys..."
cp /workspace/.ssh/* ~/.ssh/
chmod 600 ~/.ssh/*

echo "Cloning r2b_safety_lab..."
if [ ! -d "/workspace/bench/r2b_safety_lab" ]; then
    git clone git@github.com:rafafelixphd/r2b_safety_lab.git /workspace/bench/r2b_safety_lab --branch dev
else
    echo "r2b_safety_lab already exists at /workspace/bench/r2b_safety_lab"
fi

echo "Updating r2b_safety_lab..."
source /workspace/bench/r2b_safety_lab/scripts/.localrc
cd /workspace/bench/r2b_safety_lab
git pull

echo "Logging in to Hugging Face..."
hf auth login --token $HF_TOKEN

echo "Ready to work!"