
import argparse
import sys
import time
import os
import cv2
import mediapipe as mp
import numpy as np
from pathlib import Path

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

# Ensure src is in pythonpath
current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(current_dir, "../../../src")
sys.path.append(src_path)

from r2b.depth.stereo_engine import StereoPerceptionEngine
from utils import visualize # Assuming utils.py exists in the dir

def decode(frame):
    if frame is None: return None, None
    left = frame[0:800, 64:1328]
    right = frame[0:800, 1328:2592]
    return left, right

def create_alpha_depth_view(filt_disp, min_disp=0, max_disp_vis=96):
    """
    Creates a new window/image where alpha (transparency) or intensity represents distance.
    """
    disp_float = filt_disp.astype(np.float32) / 16.0
    disp_norm = (disp_float - min_disp) / max_disp_vis
    disp_norm = np.clip(disp_norm, 0, 1)
    
    # Red = Close (High Disp), Blue = Far (Low Disp)
    disp_u8 = (disp_norm * 255).astype(np.uint8)
    colored_depth = cv2.applyColorMap(disp_u8, cv2.COLORMAP_JET)
    
    # Alpha Map: Fade to black for far/background
    alpha_map = disp_norm
    alpha_3c = cv2.merge([alpha_map, alpha_map, alpha_map])
    
    fade_view = (colored_depth.astype(np.float32) * alpha_3c).astype(np.uint8)
    return fade_view

def run(model: str, camera_id: int, width: int, height: int) -> None:
    # Variables to calculate FPS
    counter, fps = 0, 0
    start_time = time.time()
    
    # Visualization parameters
    fps_avg_frame_count = 10
    
    detection_results = {"left": [], "right": []}
    
    def visualize_callback_left(result: vision.ObjectDetectorResult, output_image: mp.Image, timestamp_ms: int):
        detection_results["left"] = [result]
        
    def visualize_callback_right(result: vision.ObjectDetectorResult, output_image: mp.Image, timestamp_ms: int):
        detection_results["right"] = [result]

    # Initialize Object Detector
    base_options = python.BaseOptions(model_asset_path=model)
    options_left = vision.ObjectDetectorOptions(base_options=base_options,
                                           running_mode=vision.RunningMode.LIVE_STREAM,
                                           score_threshold=0.5,
                                           result_callback=visualize_callback_left)
    detector_left = vision.ObjectDetector.create_from_options(options_left)
    
    options_right = vision.ObjectDetectorOptions(base_options=base_options,
                                           running_mode=vision.RunningMode.LIVE_STREAM,
                                           score_threshold=0.5,
                                           result_callback=visualize_callback_right)
    detector_right = vision.ObjectDetector.create_from_options(options_right)

    # Initialize Camera
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)
    
    # Initialize Engine
    # Width/Height here refer to the single eye resolution after decode
    engine = StereoPerceptionEngine(width=1264, height=800)

    mode = 'a' # a: alpha/depth view (default requested), d: detection, s: disparity

    print("Commands:")
    print(" 'a': Alpha Depth View (Red=Close)")
    print(" 'd': Detection View")
    print(" 's': Disparity View")
    print(" 'q' / ESC: Quit")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret: break
        
        counter += 1
        
        # 1. Decode & Preprocess
        left, right = decode(frame)
        if left is None: continue

        # 2. Object Detection (Async)
        # Note: MediaPipe expects RGB
        rgb_left = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
        mp_left = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_left)
        
        rgb_right = cv2.cvtColor(right, cv2.COLOR_BGR2RGB)
        mp_right = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_right)

        # Timestamps must be monotonically increasing
        ts_ms = int(time.time() * 1000)
        detector_left.detect_async(mp_left, ts_ms)
        detector_right.detect_async(mp_right, ts_ms)
        
        # 3. Stereo Compute
        depth_data = engine.compute(left, right)
        
        # Calculate FPS
        if counter % fps_avg_frame_count == 0:
            end_time = time.time()
            fps = fps_avg_frame_count / (end_time - start_time)
            start_time = time.time()
            
        fps_text = 'FPS = {:.1f}'.format(fps)
        
        # 4. Visualization Selection
        final_display = None
        
        if mode == 'd':
            # Detection Overlay
            vis_left = left.copy()
            vis_right = right.copy()
            if detection_results["left"]:
                vis_left = visualize(vis_left, detection_results["left"][0])
            if detection_results["right"]:
                vis_right = visualize(vis_right, detection_results["right"][0])
            final_display = np.hstack((vis_left, vis_right))
            # Downscale for viewing
            final_display = cv2.resize(final_display, (0,0), fx=0.5, fy=0.5)

        elif mode == 's':
            # Raw Disparity (Jet)
            disp_vis = cv2.normalize(depth_data.filtered_disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            final_display = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
            # Add text
            cv2.putText(final_display, "Red=Close, Blue=Far", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        elif mode == 'a':
            # Alpha Channel View (Requested Feature)
            # Using typical SGBM range for min/max norm
            final_display = create_alpha_depth_view(depth_data.filtered_disparity, min_disp=0, max_disp_vis=64*16)
            cv2.putText(final_display, "Alpha Distance View", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255,255,255), 2)

        # Overlay FPS
        if final_display is not None:
             cv2.putText(final_display, fps_text, (24, 20), cv2.FONT_HERSHEY_PLAIN, 1, (255, 255, 255), 2)
             cv2.imshow('Object & Depth Demo', final_display)
        
        key = cv2.waitKey(1)
        if key in (27, ord('q')):
            break
        elif key == ord('d'): mode = 'd'
        elif key == ord('s'): mode = 's'
        elif key == ord('a'): mode = 'a'

    detector_left.close()
    detector_right.close()
    cap.release()
    cv2.destroyAllWindows()

def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      default=str(Path.home() / '.cache/mediapipe/weights/efficientdet.tflite')) # Simplified default
  parser.add_argument(
      '--cameraId', help='Id of camera.', required=False, type=int, default=0)
  parser.add_argument(
      '--frameWidth',
      help='Width of frame to capture from camera.',
      required=False,
      type=int,
      default=1280)
  parser.add_argument(
      '--frameHeight',
      help='Height of frame to capture from camera.',
      required=False,
      type=int,
      default=720)
  args = parser.parse_args()

  run(args.model, int(args.cameraId), args.frameWidth, args.frameHeight)

if __name__ == '__main__':
  main()
