#/bin/bash

cp /workspace/.ssh/* ~/.ssh/
chmod 600 ~/.ssh/*

if [ ! -d "/workspace/bench/r2b_safety_lab" ]; then
    git clone git@github.com:rafafelixphd/r2b_safety_lab.git /workspace/bench/r2b_safety_lab --branch dev
fi

source /workspace/bench/r2b_safety_lab/scripts/.localrc
cd /workspace/bench/r2b_safety_lab

git pull

hf auth login --token $HF_TOKEN