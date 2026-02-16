#/bin/bash

#ssh-keygen -t ed25519 -C "rafafelix.subscribe@gmail.com"
cp /workspace/.ssh/* ~/.ssh/
chmod 600 ~/.ssh/*

git clone git@github.com:rafafelixphd/r2b_safety_lab.git /workspace/bench/ --branch dev

hf auth login --token $HF_TOKEN

cd /workspace/bench/r2b_safety_lab

source /workspace/bench/r2b_safety_lab/scripts/.localrc

pip install -e .[train]