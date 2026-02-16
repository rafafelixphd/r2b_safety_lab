#/bin/bash

#ssh-keygen -t ed25519 -C "rafafelix.subscribe@gmail.com"
cp /workspace/.ssh/* ~/.ssh/
chmod 600 ~/.ssh/*

if [ ! -d "/workspace/bench/r2b_safety_lab" ]; then
    git clone git@github.com:rafafelixphd/r2b_safety_lab.git /workspace/bench/r2b_safety_lab --branch dev
fi


cd /workspace/bench/r2b_safety_lab
git pull

source scripts/.localrc

pip install -e .[train]

hf auth login --token $HF_TOKEN
