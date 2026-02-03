## Logging issues

---

> Issues when trying to run lerobot-record command

```log

rafaelfelix@Mac r2b % lerobot-record \
    --robot.type=so101_follower \
    --robot.port=$FOLLOWER_PORT \
    --robot.id=$FOLLOWER_ID \
    --teleop.type=so101_leader \
    --teleop.port=$LEADER_PORT \
    --teleop.id=$LEADER_ID \
    --robot.cameras='{ front: {type: opencv, index_or_path: 0, width: 3448, height: 808, fps: 30}}' \
    --display_data=true \
    --dataset.push_to_hub=false \
    --dataset.single_task="Grab the red cylinder" \
    --dataset.repo_id="local/data-collection" \
    --dataset.num_episodes=2 \
    --dataset.video_encoding_batch_size=4
objc[40009]: Class AVFFrameReceiver is implemented in both /Users/rafaelfelix/.pyenv/versions/3.10.16/envs/r2b-3.10/lib/python3.10/site-packages/cv2/.dylibs/libavdevice.61.3.100.dylib (0x1064943a8) and /Users/rafaelfelix/.pyenv/versions/3.10.16/envs/r2b-3.10/lib/python3.10/site-packages/av/.dylibs/libavdevice.61.3.100.dylib (0x1444c43a8). This may cause spurious casting failures and mysterious crashes. One of the duplicates must be removed or renamed.
objc[40009]: Class AVFAudioReceiver is implemented in both /Users/rafaelfelix/.pyenv/versions/3.10.16/envs/r2b-3.10/lib/python3.10/site-packages/cv2/.dylibs/libavdevice.61.3.100.dylib (0x1064943f8) and /Users/rafaelfelix/.pyenv/versions/3.10.16/envs/r2b-3.10/lib/python3.10/site-packages/av/.dylibs/libavdevice.61.3.100.dylib (0x1444c43f8). This may cause spurious casting failures and mysterious crashes. One of the duplicates must be removed or renamed.
INFO 2026-02-03 18:59:06 t_record.py:375 {'dataset': {'episode_time_s': 60,
             'fps': 30,
             'num_episodes': 2,
             'num_image_writer_processes': 0,
             'num_image_writer_threads_per_camera': 4,
             'private': False,
             'push_to_hub': False,
             'rename_map': {},
             'repo_id': 'local/data-collection',
             'reset_time_s': 60,
             'root': None,
             'single_task': 'Grab the red cylinder',
             'tags': None,
             'video': True,
             'video_encoding_batch_size': 4},
 'display_data': True,
 'play_sounds': True,
 'policy': None,
 'resume': False,
 'robot': {'calibration_dir': None,
           'cameras': {'front': {'color_mode': <ColorMode.RGB: 'rgb'>,
                                 'fourcc': None,
                                 'fps': 30,
                                 'height': 808,
                                 'index_or_path': 0,
                                 'rotation': <Cv2Rotation.NO_ROTATION: 0>,
                                 'warmup_s': 1,
                                 'width': 3448}},
           'disable_torque_on_disconnect': True,
           'id': 'v0',
           'max_relative_target': None,
           'port': '/dev/tty.usbmodem5AAF2631481',
           'use_degrees': False},
 'teleop': {'calibration_dir': None,
            'id': 'v0',
            'port': '/dev/tty.usbmodem5AA90240081',
            'use_degrees': False}}
2026-02-03 18:59:07.163 python[40009:1099605] WARNING: AVCaptureDeviceTypeExternal is deprecated for Continuity Cameras. Please use AVCaptureDeviceTypeContinuityCamera and add NSCameraUseContinuityCameraDeviceType to your Info.plist.
INFO 2026-02-03 18:59:08 a_opencv.py:180 OpenCVCamera(0) connected.
INFO 2026-02-03 18:59:08 follower.py:104 v0 SO101Follower connected.
INFO 2026-02-03 18:59:08 01_leader.py:82 v0 SO101Leader connected.
INFO 2026-02-03 18:59:08 ls/utils.py:227 Recording episode 0
WARNING 2026-02-03 18:59:08 l/darwin.py:211 This process is not trusted! Input event monitoring will not be possible until it is added to accessibility clients.
Right arrow key pressed. Exiting loop...
INFO 2026-02-03 18:59:26 ls/utils.py:227 Reset the environment
Map: 100%|█████████████████████████████████████████████████████| 527/527 [00:00<00:00, 2065.23 examples/s]
INFO 2026-02-03 19:00:31 ls/utils.py:227 Recording episode 1
Right arrow key pressed. Exiting loop...
Map: 100%|█████████████████████████████████████████████████████| 391/391 [00:00<00:00, 2063.88 examples/s]
INFO 2026-02-03 19:00:50 eo_utils.py:640 Recording stopped. Encoding remaining episodes...
INFO 2026-02-03 19:00:50 eo_utils.py:644 Encoding remaining 2 episodes, from episode 0 to 1
INFO 2026-02-03 19:00:50 dataset.py:1264 Batch encoding 4 videos for episodes 0 to 1
Traceback (most recent call last):
  File "/Users/rafaelfelix/.pyenv/versions/r2b-3.10/bin/lerobot-record", line 6, in <module>
    sys.exit(main())
  File "/Users/rafaelfelix/Projects/r2b/lerobot/src/lerobot/scripts/lerobot_record.py", line 516, in main
    record()
  File "/Users/rafaelfelix/Projects/r2b/lerobot/src/lerobot/configs/parser.py", line 233, in wrapper_inner
    response = fn(cfg, *args, **kwargs)
  File "/Users/rafaelfelix/Projects/r2b/lerobot/src/lerobot/scripts/lerobot_record.py", line 448, in record
    with VideoEncodingManager(dataset):
  File "/Users/rafaelfelix/Projects/r2b/lerobot/src/lerobot/datasets/video_utils.py", line 648, in __exit__
    self.dataset._batch_save_episode_video(start_ep, end_ep)
  File "/Users/rafaelfelix/Projects/r2b/lerobot/src/lerobot/datasets/lerobot_dataset.py", line 1268, in _batch_save_episode_video
    chunk_idx = self.meta.episodes[start_episode]["data/chunk_index"]
TypeError: 'NoneType' object is not subscriptable
```
