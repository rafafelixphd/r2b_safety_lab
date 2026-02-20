#!/bin/bash

# Define your base paths
BASE_DIR="/Users/rafaelfelix/.cache/huggingface/lerobot/rafafelixphd/dataset-trial-1"
IN_DIR="$BASE_DIR/videos/observation.images.front/chunk-000"

# Create target directories
mkdir -p "$BASE_DIR/videos/observation.camera.stereo/chunk-000"
mkdir -p "$BASE_DIR/videos/observation.camera.left/chunk-000"
mkdir -p "$BASE_DIR/videos/observation.camera.right/chunk-000"

for f in "$IN_DIR"/*.mp4; do
    FILENAME=$(basename "$f")
    
    ffmpeg -i "$f" \
    -filter_complex \
    "[0:v]crop=1275:798:50:0,scale=1280:720[stereo_l]; \
     [0:v]crop=1275:798:1330:0,scale=1280:720[stereo_r]; \
     [0:v]crop=1275:798:50:0,scale=224:224[sd_l]; \
     [0:v]crop=1275:798:1330:0,scale=224:224[sd_r]" \
    -map "[stereo_l]" -c:v libx264 -crf 18 "$BASE_DIR/videos/observation.camera.stereo/chunk-000/left-$FILENAME" \
    -map "[stereo_r]" -c:v libx264 -crf 18 "$BASE_DIR/videos/observation.camera.stereo/chunk-000/right-$FILENAME" \
    -map "[sd_l]" -c:v libx264 -crf 18 "$BASE_DIR/videos/observation.camera.left/chunk-000/$FILENAME" \
    -map "[sd_r]" -c:v libx264 -crf 18 "$BASE_DIR/videos/observation.camera.right/chunk-000/$FILENAME"
done