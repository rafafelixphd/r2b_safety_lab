#/bin/bash

source /workspace/bench/r2b_safety_lab/scripts/.localrc

cp /workspace/.ssh/* ~/.ssh/
chmod 600 ~/.ssh/*

cd /workspace/bench/r2b_safety_lab

git pull

hf auth login --token $HF_TOKEN