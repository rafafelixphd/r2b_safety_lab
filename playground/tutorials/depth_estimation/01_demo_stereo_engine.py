
import cv2
import numpy as np
import sys
import os
import time

# Ensure src is in pythonpath
current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(current_dir, "../../../src")
sys.path.append(src_path)

from r2b.depth.stereo_engine import StereoPerceptionEngine

def decode(frame):
    """
    Decodes the side-by-side PS4 camera frame.
    Based on established slicing for 3448x808 input.
    """
    # Initialize buffers (800x1264)
    # Note: The original generic code used slightly different slicing, 
    # but we will stick to the one found in View_Depth.py
    # left[i] = frame[i, 64: 1280 + 48] -> width 1264
    # right[i] = frame[i, 1280 + 48: 1280 + 48 + 1264] -> width 1264
    
    # Pre-allocate for speed
    h, w, c = 800, 1264, 3
    left = np.zeros((h, w, c), np.uint8)
    right = np.zeros((h, w, c), np.uint8)
    
    # Vectorized slicing is faster than loop
    # Frame shape is expected to be (808, 3448, 3) or similar
    # The loop in View_Depth was: range(800) -> so raw frame might be 808 high but we only take 800 ?
    # Let's assume the user wants the loop logic but vectorized:
    # frame[0:800, 64:1328]
    
    if frame is None:
        return None, None

    # Using the exact offsets from View_Depth.py
    # 64 : 1280+48  => 64 : 1328 (Length 1264)
    # 1328 : 1328+1264 => 1328 : 2592 (Length 1264)
    
    # We take the first 800 rows
    left = frame[0:800, 64:1328]
    right = frame[0:800, 1328:2592]
    
    return left, right

def main():
    video_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    
    print(f"Initializing Camera {video_id}...")
    cap = cv2.VideoCapture(video_id)
    
    # PS4 Camera specific resolution for 60fps mode typically
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)
    
    if not cap.isOpened():
        print("Failed to open camera.")
        return

    # Initialize Engine (Advanced/Safety-Critical Version)
    # Note: The engine now automatically loads calibration from assets
    engine = StereoPerceptionEngine(width=1264, height=800)
    
    print("Starting Loop. Press 'q' to exit.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame drop.")
            break
            
        start_time = time.time()
        
        # 1. Decode generic side-by-side
        left_img, right_img = decode(frame)
        if left_img is None:
            continue
            
        # 2. Compute Depth & Safety Metrics
        depth_data = engine.compute(left_img, right_img)
        
        # 3. Visualization
        # Normalize disparity for display
        disp_vis = cv2.normalize(depth_data.filtered_disparity, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)
        disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)
        
        # Overlay Metrics
        metrics = depth_data.risk_metadata
        fps = 1.0 / (time.time() - start_time)
        
        info_txt = [
            f"FPS: {fps:.1f}",
            f"Min Dist: {metrics['min_dist']:.2f} m",
            f"Avg Dist: {metrics['avg_dist']:.2f} m",
            f"Max Dist: {metrics['max_dist']:.2f} m"
        ]
        
        for i, line in enumerate(info_txt):
            cv2.putText(disp_color, line, (10, 30 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Stack images (Left Input | Depth Output)
        # Resize left to match disparity height if needed, but they should match
        combined = np.hstack((left_img, disp_color))
        
        cv2.imshow("Stereo Perception Engine (Pro)", combined)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
