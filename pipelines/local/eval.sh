TIMESTAMP=$(date +%Y%m%d_%H%M%S)
lerobot-record \
  --robot.type=so101_follower \
  --robot.port="$FOLLOWER_PORT" \
  --robot.cameras="$PS4EYE_CAMERA" \
  --robot.id=v0 \
  --display_data=true \
  --dataset.repo_id="${HF_USER}/eval_dataset-trial-${TIMESTAMP}" \
  --dataset.single_task="Put red cylinder into the toy box" \
  --policy.path="/Users/rafaelfelix/.cache/huggingface/lerobot/rafafelixphd/act-stereo-1/last/pretrained_model"

