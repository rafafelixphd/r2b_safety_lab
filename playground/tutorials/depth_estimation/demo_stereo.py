"""
Interactive Demo for StereoPerceptionEngine.
Allows real-time tuning of stereo parameters using OpenCV Trackbars.
"""

import cv2
import numpy as np
import time
from r2b.depth import StereoPerceptionEngine

def empty(x):
    pass

def run_interactive_demo():
    print("Initializing StereoPerceptionEngine...")
    engine = StereoPerceptionEngine(width=1080, height=720)    
    # --- GUI SETUP ---
    window_name = "Stereo Tuner"
    cv2.namedWindow(window_name)
    
    # Trackbars
    # Min Disparity: 0 to 100
    cv2.createTrackbar("Min Disp", window_name, 0, 100, empty)
    # Num Disparities: 1 to 10 (multiplied by 16)
    cv2.createTrackbar("Num Disp * 16", window_name, 4, 10, empty)
    # WLS Lambda: 100 to 10000
    cv2.createTrackbar("WLS Lambda", window_name, 8000, 20000, empty)
    # WLS Sigma (x10): 0 to 50 -> 0.0 to 5.0
    cv2.createTrackbar("WLS Sigma x10", window_name, 15, 50, empty)

    print("\n[INFO] Controls:")
    print(" - Adjust trackbars to tune stereo matching.")
    print(" - Press 'q' to exit.")
    
    import r2b.video

    import sys
    camera_id = int(sys.argv[1])

    camera = r2b.video.PS4EyeStereoCamera(video_id=camera_id)
    for idx, left, right in camera:
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
        
        disp_vis = cv2.normalize(result.filtered_disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        
        # Overlay Metrics
        info_img = np.zeros((150, 400, 3), dtype=np.uint8)
        cv2.putText(info_img, f"FPS: {1.0/dt:.1f}", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)
        cv2.putText(info_img, f"Min Dist: {result.risk_metadata['min_dist']:.2f} m", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
        cv2.putText(info_img, f"Avg Dist: {result.risk_metadata['avg_dist']:.2f} m", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        cv2.putText(info_img, f"Max Dist: {result.risk_metadata['max_dist']:.2f} m", (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
        
        cv2.imshow("Disparity Map", disp_vis)
        cv2.imshow("Metrics", info_img)
        # Also show input
        cv2.imshow("Left Input", left)
        cv2.imshow("Right Input", right)
        
        key = cv2.waitKey(30)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    run_interactive_demo()
