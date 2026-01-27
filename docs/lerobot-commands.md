# LeRobot commands

---

**1. Finding ports**

```bash
    lerobot-find-port
```

**2. Setting environment variables**

```bash
    export FOLLOWER_ID="/dev/tty.usbmodem5AAF2631481"
    export LEADER_ID="/dev/tty.usbmodem5AA90240081"
```

**3. Calibrate**

```bash
lerobot-calibrate \
    --robot.type="so101_follower" \
    --robot.port="$FOLLOWER_ID" \
    --robot.id="so-arm-101-002"
# To be created here: ~/.cache/huggingface/lerobot/calibration/robots/so101_follower

lerobot-calibrate \
    --teleop.type="so101_leader" \
    --teleop.port="$LEADER_ID" \
    --teleop.id="so-arm-101-002"

# To be created here: ~/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader
```

**4. Teleoperate**

```bash
lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=so_101_follower_001 \
    --robot.id=$FOLLOWER_NAME \
    --teleop.type=so101_leader \
    --teleop.port=$LEADER_ID \
    --teleop.id=so-arm-101-002 \

```
