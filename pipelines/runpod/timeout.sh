#!/bin/bash


export RUNPOD_POD_ID="$(runpodctl get pod | awk 'NR==2 {print $1}')"
sleep 1; runpodctl stop pod $RUNPOD_POD_ID &
