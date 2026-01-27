"""
Interactive Demo for StereoPerceptionEngine (Pro).
Allows real-time tuning of stereo parameters using OpenCV Trackbars.
Updates:
- Integrates calibrated StereoPerceptionEngine.
- Red=Close, Blue=Far visualization.
- Alpha blending channel for distance.
"""

import cv2
import numpy as np
import time
import sys
import os

# Ensure src is in pythonpath
current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(current_dir, "../../../src")
sys.path.append(src_path)

from r2b.depth.stereo_engine import StereoPerceptionEngine

def empty(x):
    pass

def decode(frame):
    # Same decoding as established
    if frame is None: return None, None
    left = frame[0:800, 64:1328]
    right = frame[0:800, 1328:2592]
    return left, right

def create_alpha_depth_view(filt_disp, min_disp=0, max_disp_vis=96):
    """
    Creates a new window/image where alpha (transparency) or intensity represents distance.
    Actually, standard images don't support Alpha in cv2.imshow (ignores alpha).
    We will simulate this by creating a synthetic view:
    - Foreground (Close) is opaque/bright.
    - Background (Far) is transparent/dark (faded to black).
    """
    # Normalize disparity to 0-1
    # raw filtered range is dynamic, but we can assume [0, num_disp * 16] roughly
    # Convert to float
    disp_float = filt_disp.astype(np.float32) / 16.0
    
    # Clip to visible range
    disp_norm = (disp_float - min_disp) / max_disp_vis
    disp_norm = np.clip(disp_norm, 0, 1)
    
    # Alpha Map: Close (High Disp) -> 1.0, Far (Low Disp) -> 0.0
    alpha_map = disp_norm
    
    # Create a nice colored base (e.g., Heatmap)
    # We want Red=Close (High Disp), Blue=Far (Low Disp)
    # OpenCV's COLORMAP_JET does Blue(low) -> Red(high).
    # So applying JET to disp_norm (scaled 0-255) gives exactly Red=Close, Blue=Far.
    
    disp_u8 = (disp_norm * 255).astype(np.uint8)
    colored_depth = cv2.applyColorMap(disp_u8, cv2.COLORMAP_JET)
    
    # Now apply the alpha effect (Fade to black for far/background)
    # This emphasizes objects close to the camera.
    # alpha_map needs 3 channels
    alpha_3c = cv2.merge([alpha_map, alpha_map, alpha_map])
    
    # Result = Color * Alpha
    fade_view = (colored_depth.astype(np.float32) * alpha_3c).astype(np.uint8)
    
    return fade_view

def run_interactive_demo():
    camera_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    print(f"Initializing Camera {camera_id}...")
    
    cap = cv2.VideoCapture(camera_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)
    
    if not cap.isOpened():
        print("Error: Camera not found")
        return

    print("Initializing StereoPerceptionEngine...")
    # Initialize with default calibrated settings
    engine = StereoPerceptionEngine(width=1264, height=800)    
    
    # --- GUI SETUP ---
    window_name = "Stereo Tuner"
    cv2.namedWindow(window_name)
    
    # Trackbars
    cv2.createTrackbar("Min Disp", window_name, 0, 100, empty)
    cv2.createTrackbar("Num Disp * 16", window_name, 3, 10, empty) # Default 3*16 = 48
    cv2.createTrackbar("WLS Lambda", window_name, 8000, 20000, empty)
    cv2.createTrackbar("WLS Sigma x10", window_name, 15, 50, empty)
    
    print("\n[INFO] Controls:")
    print(" - Adjust trackbars to tune stereo matching.")
    print(" - Press 'q' to exit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame drop")
            break
            
        left, right = decode(frame)
        if left is None: continue

        # 1. Read Trackbars
        min_disp = cv2.getTrackbarPos("Min Disp", window_name)
        n_disp_val = cv2.getTrackbarPos("Num Disp * 16", window_name)
        num_disp = max(1, n_disp_val) * 16
        w_lambda = cv2.getTrackbarPos("WLS Lambda", window_name)
        w_sigma_val = cv2.getTrackbarPos("WLS Sigma x10", window_name)
        w_sigma = w_sigma_val / 10.0
        
        # 2. Update Engine
        engine.set_tuning_params(
            min_disp=min_disp,
            num_disp=num_disp,
            wls_lambda=w_lambda,
            wls_sigma=w_sigma
        )
        
        # 3. Compute
        start_t = time.time()
        result = engine.compute(left, right)
        dt = time.time() - start_t
        
        # 4. Visualization
        # (A) Standard Disparity Map (normalized min-max)
        disp_raw = result.filtered_disparity
        disp_vis = cv2.normalize(disp_raw, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
        
        # (B) Alpha/Distance View (Red=Close, Blue=Far + Fade)
        alpha_view = create_alpha_depth_view(disp_raw, min_disp=min_disp*16, max_disp_vis=num_disp*16)
        
        # Overlay Metrics on Alpha View
        metrics = result.risk_metadata
        fps_txt = f"FPS: {1.0/dt:.1f}"
        
        cv2.putText(alpha_view, fps_txt, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(alpha_view, "RED = CLOSE (Risk)", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(alpha_view, "BLUE = FAR (Safe)", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        
        if metrics:
            cv2.putText(alpha_view, f"Min Dist: {metrics['min_dist']:.2f}m", (10, 130), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        
        # Stack for display
        # Resize input for compactness
        input_small = cv2.resize(left, (632, 400))
        disp_small = cv2.resize(disp_color, (632, 400))
        alpha_small = cv2.resize(alpha_view, (632, 400))
        
        # Top row: Input | Disparity
        row1 = np.hstack((input_small, disp_small))
        # Bot row: Alpha View | Black (or Right input)
        # Let's put Alpha View large separate or just stacking
        
        cv2.imshow("Tuner (Input | Raw Disparity)", row1)
        cv2.imshow("Alpha Distance View (Red=Close)", alpha_small)
        
        key = cv2.waitKey(1)
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_interactive_demo()
