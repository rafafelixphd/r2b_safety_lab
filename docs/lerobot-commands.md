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

3. Teleoperate

```bash
lerobot-teleoperate \
    --robot.type=so101_follower \
    --robot.port=$FOLLOWER_ID \
    --robot.id=$FOLLOWER_NAME \
    --teleop.type=so101_leader \
    --teleop.port=$LEADER_ID \
    --teleop.id=$LEADER_NAME
```
