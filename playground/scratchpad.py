# for filename in $(ls -l /Users/rafaelfelix/.cache/huggingface/lerobot//rafafelixphd/dataset-trial-1/videos/observation.images.front/chunk-000/); do
#     ffmpeg -i $filename -filter:v "crop=1724:808:0:0, scale=224:224" -c:v libx264 -crf 18 observation.images.front_left.mp4
# done



# ffmpeg -i input.mp4 -filter:v "crop=1275:798:50:0, scale=224:224" -c:v libx264 -crf 18 left.mp4
# ffmpeg -i input.mp4 -filter:v "crop=1275:798:1330:0, scale=224:224" -c:v libx264 -crf 18 right.mp4

# /Users/rafaelfelix/.cache/huggingface/lerobot/rafafelixphd/dataset-trial-2/videos/observation.images.front.left/chunk-000/
# /Users/rafaelfelix/.cache/huggingface/lerobot/rafafelixphd/dataset-trial-2/videos/observation.images.front.right/chunk-000/



# # Define your base paths
# BASE_DIR="/Users/rafaelfelix/.cache/huggingface/lerobot/rafafelixphd/dataset-trial-1"
# IN_DIR="$BASE_DIR/videos/observation.images.front/chunk-000"

# # Create target directories
# mkdir -p "$BASE_DIR/videos/observation.camera.stereo/chunk-000"
# mkdir -p "$BASE_DIR/videos/observation.camera.left/chunk-000"
# mkdir -p "$BASE_DIR/videos/observation.camera.right/chunk-000"

# for f in "$IN_DIR"/*.mp4; do
#     FILENAME=$(basename "$f")
    
#     ffmpeg -i "$f" \
#     -filter_complex \
#     "[0:v]crop=2605:798:50:0,scale=2560:720[stereo]; \
#      [0:v]crop=1275:798:50:0,scale=224:224[sd_l]; \
#      [0:v]crop=1275:798:1330:0,scale=224:224[sd_r]" \
#     -map "[stereo]" -c:v libx264 -crf 18 "$BASE_DIR/videos/observation.camera.stereo/chunk-000/$FILENAME" \
#     -map "[sd_l]" -c:v libx264 -crf 18 "$BASE_DIR/videos/observation.camera.left/chunk-000/$FILENAME" \
#     -map "[sd_r]" -c:v libx264 -crf 18 "$BASE_DIR/videos/observation.camera.right/chunk-000/$FILENAME"
# done




# lerobot-train \
#   --dataset.repo_id="rafafelixphd/dataset-trial-1" \
#   --output_dir=outputs/train/act_so101_test \
#   --job_name=act_so101_test \
#   --policy.type=act \
#   --policy.device=cuda \
#   --policy.repo_id="rafafelixphd/my_policy" \
#   --wandb.enable=true \
#   --batch_size=1