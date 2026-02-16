TIMESTAMP=$(date +%Y%m%d_%H%M%S)
lerobot-record \
  --robot.type=so101_follower \
  --robot.port=$FOLLOWER_PORT \
  --robot.cameras=$FRONT_CAMERA \
  --robot.id=v0 \
  --display_data=true \
  --dataset.repo_id=${HF_USER}/eval_dataset-trial-${TIMESTAMP} \
  --dataset.single_task="Put red cylinder into the toy box" \
  --policy.path=/Users/rafaelfelix/.cache/huggingface/hub/models--rafafelixphd--act_so101_runpod/snapshots/11259a347611049c25ee98a3c141a9ab236ef476/checkpoints/015000/pretrained_model

