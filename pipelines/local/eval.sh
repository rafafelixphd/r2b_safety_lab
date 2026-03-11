TIMESTAMP=$(date +%Y%m%d_%H%M%S)
# lerobot-record \
#   --robot.type=so101_follower \
#   --robot.port="$FOLLOWER_PORT" \
#   --robot.cameras="$PS4EYE_CAMERA" \
#   --robot.id=v0 \
#   --display_data=true \
#    --dataset.encoder_threads=8 \
#   --dataset.repo_id="${HF_USER}/eval_dataset-trial-${TIMESTAMP}" \
#   --dataset.single_task="Put red cylinder into the toy box" \
#   --policy.path="/Users/rafaelfelix/.cache/huggingface/lerobot/rafafelixphd/act-stereo-1/last/pretrained_model"

echo "$FOLLOWER_PORT";

python /Users/rafaelfelix/Projects/r2b/r2b_safety_lab/playground/scripts/lerobot-eval.py \
  --robot.port="$FOLLOWER_PORT" \
  --robot.cameras="$PS4EYE_CAMERA" \
  --robot.id=v0 \
  --display_data=true \
  --dataset.repo_id="${HF_USER}/eval_dataset-trial-${TIMESTAMP}" \
  --dataset.single_task="Put red cylinder into the toy box" \
  --policy.path="/Users/rafaelfelix/.cache/huggingface/lerobot/rafafelixphd/act-stereo-1/last/pretrained_model" \
  --device mps
