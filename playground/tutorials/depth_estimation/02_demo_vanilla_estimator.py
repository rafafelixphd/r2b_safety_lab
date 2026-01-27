
import cv2
import numpy as np
import sys
import os
import time

# Ensure src is in pythonpath
current_dir = os.path.dirname(os.path.abspath(__file__))
src_path = os.path.join(current_dir, "../../../src")
sys.path.append(src_path)

from r2b.depth.vanilla.estimator import StereoProcessor

def decode(frame):
    """
    Decodes the side-by-side PS4 camera frame.
    """
    if frame is None:
        return None, None

    # Using the exact offsets from View_Depth.py
    # Frame expected: 3448 x 808
    # Left:  0:800, 64:1328
    # Right: 0:800, 1328:2592
    
    left = frame[0:800, 64:1328]
    right = frame[0:800, 1328:2592]
    
    return left, right

def main():
    video_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    
    print(f"Initializing Camera {video_id}...")
    cap = cv2.VideoCapture(video_id)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3448)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 808)
    
    if not cap.isOpened():
        print("Failed to open camera.")
        return

    # Initialize Processor (Vanilla/Lightweight Version)
    processor = StereoProcessor(width=1264, height=800)
    
    print("Starting Loop. Press 'q' to exit.")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Frame drop.")
            break
            
        start_time = time.time()
        
        # 1. Decode
        left_img, right_img = decode(frame)
        if left_img is None:
            continue
            
        # 2. Compute (Returns primitives, not object)
        min_d, avg_d, max_d, disp_vis = processor.compute_3d_metrics(left_img, right_img)
        
        # 3. Visualization
        # disp_vis from Vanilla is already normalized/color-ready or normalized float?
        # Looking at code: disp_vis = (disp_norm - disp_norm.min()) * 255 -> uint8, then morphological close
        # So it is grayscale uint8. We can apply colormap.
        
        if disp_vis is not None and isinstance(disp_vis, np.ndarray):
            disp_color = cv2.applyColorMap(disp_vis, cv2.COLORMAP_OCEAN)
        else:
            disp_color = np.zeros_like(left_img)
        
        fps = 1.0 / (time.time() - start_time)
        
        # Overlay Metrics
        info_txt = [
            f"FPS: {fps:.1f}",
            f"Min: {min_d:.2f} m",
            f"Avg: {avg_d:.2f} m",
            f"Max: {max_d:.2f} m"
        ]
        
        for i, line in enumerate(info_txt):
            cv2.putText(disp_color, line, (10, 30 + i*30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
        # Stack images
        # Resize left to match disparity if needed (Vanilla does downsampling internally, but returns disp of what size?)
        # Vanilla returns `denoised` which is derived from `disp_norm` from `disp`.
        # `disp` comes from `matcher.compute(l_gray, r_gray)` where l_gray is pyrDown (so half size).
        # So output disp is 632x400.
        # We need to resize left image to match for vstack/hstack or resize disp up.
        
        h, w = disp_color.shape[:2]
        left_resized = cv2.resize(left_img, (w, h))
        
        combined = np.hstack((left_resized, disp_color))
        
        cv2.imshow("Stereo Processor (Vanilla)", combined)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
