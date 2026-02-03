lerobot-record \
  --robot.type=so101_follower \
  --robot.port="${FOLLOWER_PORT}" \
  --robot.id="${FOLLOWER_ID}" \
  --robot.cameras='{ front: {type: opencv, index_or_path: 1, width: 3448, height: 808, fps: 30}}' \
  --teleop.type=so101_leader \
  --teleop.port="${LEADER_PORT}" \
  --teleop.id="${LEADER_ID}" \
  --dataset.repo_id="${HF_USER}/local-test" \
  --dataset.num_episodes=5 \
  --dataset.push_to_hub=false \
  --dataset.single_task="Grab the red cylinder" \
  --dataset.video_encoding_batch_size=5 \
  --display_data=true

lerobot-record \
    --robot.type=so101_follower \
    --robot.port=$FOLLOWER_PORT \
    --robot.id=$FOLLOWER_ID \
    --teleop.type=so101_leader \
    --teleop.port=$LEADER_PORT \
    --teleop.id=$LEADER_ID \
    --robot.cameras='{ front: {type: opencv, index_or_path: 1, width: 3448, height: 808, fps: 30}}' \
    --display_data=true \
    --dataset.push_to_hub=false \
    --dataset.single_task="Grab the red cylinder" \
    --dataset.repo_id="local/data-collection" \
    --dataset.num_episodes=10 \
    --dataset.video_encoding_batch_size=5




