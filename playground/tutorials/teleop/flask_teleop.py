#!/usr/bin/env python
import os
import time
import threading
import cv2
import json
import logging
from flask import Flask, render_template, Response, jsonify, request

from lerobot.cameras.opencv.configuration_opencv import OpenCVCameraConfig
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.robots.so101_follower.so101_follower import SO101Follower

from lerobot.teleoperators.so101_leader.config_so101_leader import SO101LeaderConfig
from lerobot.teleoperators.so101_leader.so101_leader import SO101Leader

import mujoco
import numpy as np

app = Flask(__name__)

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("flask_teleop")

# Robot Configuration
follower_port = os.environ.get("FOLLOWER_PORT")
follower_id = os.environ.get("FOLLOWER_ID")
leader_port = os.environ.get("LEADER_PORT")
leader_id = os.environ.get("LEADER_ID")

# MuJoCo Setup
# Using a relative path that should be valid if running from project root or playground
# Ideally this should be more robust, but hardcoding for the known path for now
MODEL_PATH = "lerobot/SO101/so101_new_calib.xml" 
if not os.path.exists(MODEL_PATH):
    # Fallback to absolute path search or error
    logger.warning(f"MuJoCo model not found at {MODEL_PATH}. Visualization disabled.")
    mj_model = None
    mj_data = None
    renderer = None
else:
    mj_model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    mj_data = mujoco.MjData(mj_model)
    renderer = mujoco.Renderer(mj_model, height=480, width=640)


if not follower_port:
    logger.warning("FOLLOWER_PORT not set. Robot might not connect.")
if not leader_port:
    logger.warning("LEADER_PORT not set. Teleoperation might not work.")

camera_config = {
    "front": OpenCVCameraConfig(index_or_path=1, width=3448, height=808, fps=30)
}

robot_config = SO101FollowerConfig(
    port=follower_port,
    id=follower_id,
    cameras=camera_config
)

teleop_config = SO101LeaderConfig(
    port=leader_port,
    id=leader_id,
)

robot = SO101Follower(robot_config)
leader_robot = SO101Leader(teleop_config)

# Global state for camera frame and robot data
current_frame = None
current_sim_frame = None
current_state = {}
leader_state = {}
state_lock = threading.Lock()
frame_lock = threading.Lock()
sim_lock = threading.Lock()

def robot_loop():
    """Background thread to update robot state, fetch camera frames, and run teleop."""
    global current_frame, current_state, leader_state, current_sim_frame
    
    logger.info("Connecting to follower robot...")
    try:
        robot.connect()
        logger.info("Follower robot connected!")
    except Exception as e:
        logger.error(f"Failed to connect to follower robot: {e}")
    
    logger.info("Connecting to leader robot...")
    try:
        leader_robot.connect()
        logger.info("Leader robot connected!")
    except Exception as e:
        logger.error(f"Failed to connect to leader robot: {e}")

    # For mapping motor names to qpos indices, we might need a mapping.
    # The SO101 model likely uses joint names that match the config.
    # If not, we iterate joints.
    joint_names = [f"joint_{i}" for i in range(1, 7)] # Assumption based on usual naming 
    # Or inspecting model:
    if mj_model:
        # Construct map from robot motor key to qpos address
        # Robot keys: "shoulder_pan.pos", etc.
        # Check standard names. Typically SO101Follower uses descriptive names.
        # Let's try to map by order or known names if possible.
        # For now, simplistic mapping:
        pass

    while True:
        try:
            # Teleoperation Logic
            if leader_robot.is_connected and robot.is_connected:
                leader_action = leader_robot.get_action()
                # robot.send_action(leader_action)
                
                with state_lock:
                    leader_state = {k: v for k, v in leader_action.items()}
            
            # Follower Observation (includes camera frames)
            if robot.is_connected:
                obs = robot.get_observation()
                
                with state_lock:
                    # Extract motor positions/velocities if available
                    state_data = {k: v for k, v in obs.items() if not k in ["front", "top", "wrist"]}
                    current_state = state_data

                # Extract camera frame
                if "front" in obs:
                    with frame_lock:
                        frame_rgb = obs["front"]
                        current_frame = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)
                
                # Update Simulation
                if mj_model and mj_data and renderer:
                    # Map state_data to qpos
                    # Available keys in state example: 'shoulder_pan.pos', 'shoulder_lift.pos', ...
                    # Model joints: usually named similarly or indexed.
                    # Let's look up joint ids by name.
                    # Common mapping for SO-100/101:
                    # 'shoulder_pan' -> 'prop_shoulder_pan' or just 'shoulder_pan'
                    
                    # NOTE: To simple this, we rely on the fact that `state_data` has the values.
                    # We need to know the order or names in XML.
                    # Assuming names match for now.
                    for name, value in state_data.items():
                        if name.endswith(".pos"):
                            joint_name = name.replace(".pos", "")
                            # Try to find joint in model
                            try:
                                # MuJoCo names might differ slightly, trying direct match
                                j_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
                                if j_id != -1:
                                    # qpos address
                                    qpos_adr = mj_model.jnt_qposadr[j_id]
                                    mj_data.qpos[qpos_adr] = value
                            except Exception:
                                pass # Joint not found
                    
                    mujoco.mj_forward(mj_model, mj_data)
                    renderer.update_scene(mj_data)
                    img = renderer.render()
                    # Img is RGB. Convert to BGR for standard OpenCV usage if needed, 
                    # but here we just encode to jpg.
                    # cv2.imencode expects BGR usually if input is numpy.
                    # renderer.render returns (H,W,3) RGB.
                    img_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    
                    with sim_lock:
                        current_sim_frame = img_bgr

            time.sleep(0.01) # Small sleep to prevent busy loop
        except Exception as e:
            logger.error(f"Error in robot loop: {e}")
            time.sleep(1)

def generate_frames():
    """Generator for Real Camera MJPEG stream."""
    global current_frame
    while True:
        with frame_lock:
            if current_frame is None:
                frame_bytes = None
            else:
                ret, buffer = cv2.imencode('.jpg', current_frame)
                frame_bytes = buffer.tobytes()
        
        if frame_bytes:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.03) # Limit FPS for streaming

def generate_sim_frames():
    """Generator for Simulation MJPEG stream."""
    global current_sim_frame
    while True:
        with sim_lock:
            if current_sim_frame is None:
                frame_bytes = None
            else:
                ret, buffer = cv2.imencode('.jpg', current_sim_frame)
                frame_bytes = buffer.tobytes()
        
        if frame_bytes:
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
        
        time.sleep(0.03)

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/simulation_feed')
def simulation_feed():
    return Response(generate_sim_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/state')
def get_state():
    with state_lock:
        return jsonify({
            "status": "connected" if robot.is_connected else "disconnected",
            "leader_status": "connected" if leader_robot.is_connected else "disconnected",
            "data": current_state,
            "leader_data": leader_state
        })

if __name__ == '__main__':
    # Start robot thread
    t = threading.Thread(target=robot_loop, daemon=True)
    t.start()
    
    app.run(host='0.0.0.0', port=5050, debug=False)
