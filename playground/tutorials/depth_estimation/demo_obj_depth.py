import argparse
import sys
import time

import cv2
import mediapipe as mp

from pathlib import Path

from mediapipe.tasks import python
from mediapipe.tasks.python import vision

from utils import visualize


def run(model: str, camera_id: int, width: int, height: int) -> None:
  """Continuously run inference on images acquired from the camera.

  Args:
    model: Name of the TFLite object detection model.
    camera_id: The camera id to be passed to OpenCV.
    width: The width of the frame captured from the camera.
    height: The height of the frame captured from the camera.
  """

  # Variables to calculate FPS
  counter, fps = 0, 0
  start_time = time.time()

  # Visualization parameters
  row_size = 20  # pixels
  left_margin = 24  # pixels
  text_color = (0, 0, 255)  # red
  font_size = 1
  font_thickness = 1
  fps_avg_frame_count = 10

  detection_results = {"left": [], "right": []}

  def visualize_callback_left(result: vision.ObjectDetectorResult, output_image: mp.Image, timestamp_ms: int):
      detection_results["left"] = [result]

  def visualize_callback_right(result: vision.ObjectDetectorResult, output_image: mp.Image, timestamp_ms: int):
      detection_results["right"] = [result]

  # Initialize the object detection models (Left and Right)
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

  from r2b.video import PS4EyeStereoCamera
  from r2b.depth import StereoPerceptionEngine
  import numpy as np

  camera = PS4EyeStereoCamera(video_id=camera_id)
  stereo_engine = StereoPerceptionEngine(width=camera.width // 2, height=camera.height)

  mode = 'd' # d: detection, s: disparity, z: depth (point cloud Z)

  print("Commands:")
  print(" 'd': Detection View (Left/Right)")
  print(" 's': Disparity View")
  print(" 'z': Depth/Z-Map View")
  print(" 'q' / ESC: Quit")

  # Continuously capture images from the camera and run inference
  for idx, left, right in camera:
    counter += 1
    
    # 1. Inference (Left & Right)
    # Flip for visualization consistency if needed, but usually kept raw for stereo matching
    # Note: If we flip, stereo matching might break if not flipped consistently or if calibration expects unflipped.
    # PS4 camera images are usually correct. demo_obj_depth flipped manually. 
    # Let's flip for display but keep raw for stereo? Or flip both?
    # Existing demo flipped image. Let's flip both for consistency.
    left_display = cv2.flip(left, 1)
    right_display = cv2.flip(right, 1)
    
    # But wait, Stereo Engine expects specific alignment. 
    # If we flip horizontally, we swap left/right perspective effectively? No, just mirroring.
    # Standard stereo matching on mirrored images might need swapping left/right images?
    # Let's NOT flip for strictly correct stereo processing, OR flip the result for display only.
    # Modifying the input for detection is fine.
    
    # To be safe with Stereo Engine which likely assumes raw camera feed orientation:
    # We will pass raw left/right to stereo engine.
    # We will pass flippped images to detector?
    # Actually, if we flip the image, the bounding boxes flip too.
    # Let's stick to consistent orientation. If the user wants a "mirror" selfie view, we flip everything at the end?
    # But bounding boxes need to be drawn on the flipped image.
    # Using raw images for now to avoid breaking stereo correspondence.
    
    # Convert for MediaPipe
    rgb_left = cv2.cvtColor(left, cv2.COLOR_BGR2RGB)
    mp_left = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_left)
    
    rgb_right = cv2.cvtColor(right, cv2.COLOR_BGR2RGB)
    mp_right = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_right)

    detector_left.detect_async(mp_left, counter)
    detector_right.detect_async(mp_right, counter)

    # 2. Stereo Compute
    depth_data = stereo_engine.compute(left, right)

    # Calculate FPS
    if counter % fps_avg_frame_count == 0:
        end_time = time.time()
        fps = fps_avg_frame_count / (end_time - start_time)
        start_time = time.time()
    
    # Visualization
    fps_text = 'FPS = {:.1f}'.format(fps)
    
    final_display = None

    if mode == 'd':
        # Detection View
        vis_left = left.copy()
        vis_right = right.copy()
        
        if detection_results["left"]:
            vis_left = visualize(vis_left, detection_results["left"][0])
        if detection_results["right"]:
            vis_right = visualize(vis_right, detection_results["right"][0])
            
        final_display = np.hstack((vis_left, vis_right))
        # Resize if too wide
        if final_display.shape[1] > 1920:
             final_display = cv2.resize(final_display, (0, 0), fx=0.5, fy=0.5)

    elif mode == 's':
        # Disparity View
        # Normalize disparity for display
        disp_vis = cv2.normalize(depth_data.filtered_disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        final_display = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

    elif mode == 'z':
        # Depth/Z View
        # Z is in meters. Range ~0.5m to 5m usually relevant.
        z_map = depth_data.point_cloud_z
        # Clip for visualization (e.g., 0 to 3 meters)
        z_vis = np.clip(z_map, 0, 3.0)
        # Normalize to 0-255
        z_vis = cv2.normalize(z_vis, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        final_display = cv2.applyColorMap(z_vis, cv2.COLORMAP_TURBO) # Turbo is good for depth

    # Overlay FPS
    if final_display is not None:
        cv2.putText(final_display, fps_text, (24, 20), cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255), 1)
        cv2.imshow('Stereo Demo', final_display)

    # Stop the program if the ESC key is pressed.
    keypressed = cv2.waitKey(1)
    if keypressed in (27, ord('q')):
      break
    elif keypressed == ord('d'):
        mode = 'd'
        print("Switched to Detection Mode")
    elif keypressed == ord('s'):
        mode = 's'
        print("Switched to Disparity Mode")
    elif keypressed == ord('z'):
        mode = 'z'
        print("Switched to Depth Mode")

  detector_left.close()
  detector_right.close()
  if hasattr(camera, 'release'):
      camera.release()
  elif hasattr(camera, 'stop'):
      camera.stop()
  cv2.destroyAllWindows()


def main():
  parser = argparse.ArgumentParser(
      formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument(
      '--model',
      help='Path of the object detection model.',
      required=False,
      default=str(Path('~/.cache/mediapipe/weights/efficientdet.tflite').expanduser()))
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
