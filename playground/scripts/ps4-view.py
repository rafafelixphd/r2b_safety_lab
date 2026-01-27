import cv2
import sys
import time
import numpy as np
from r2b.video.ps4 import PS4EyeStereoCamera
from r2b.logger import get_logger

logger = get_logger(namespace="ps4-view")

def draw_overlay(frame, fps, frame_count, width, height):
    # Colors
    bg_color = (20, 20, 20)      # Dark Gray
    text_color = (0, 255, 255)   # Cyan
    accent_color = (0, 0, 255)   # Red
    
    # Bottom bar
    bar_height = 40
    cv2.rectangle(frame, (0, height - bar_height), (width, height), bg_color, -1)
    
    # Text info
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.6
    thickness = 1
    
    # FPS
    fps_text = f"FPS: {fps:.1f}"
    cv2.putText(frame, fps_text, (20, height - 12), font, font_scale, text_color, thickness, cv2.LINE_AA)

    # Frame Count
    frame_text = f"Frame: {frame_count}"
    cv2.putText(frame, frame_text, (150, height - 12), font, font_scale, text_color, thickness, cv2.LINE_AA)

    # Title
    title_text = "PS4 Eye Stereo View"
    text_size = cv2.getTextSize(title_text, font, font_scale, thickness)[0]
    text_x = (width - text_size[0]) // 2
    cv2.putText(frame, title_text, (text_x, height - 12), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)
    
    # Blinking REC dot (visual only)
    if (time.time() % 1) > 0.5:
        cv2.circle(frame, (width - 30, height - 20), 6, accent_color, -1)
    
    cv2.putText(frame, "LIVE", (width - 70, height - 12), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)


if __name__ == "__main__":

    video_id = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    camera = PS4EyeStereoCamera(video_id=video_id, target_size=(1080,720))
    
    start_time = time.time()
    frames_processed = 0
    current_fps = 0.0
    last_fps_time = time.time()

    print("Starting PS4 Eye View... Press 'q' to quit.")

    try:
        for (idx, left, right) in camera:
            # Stack images horizontally
            combined = np.hstack((left, right))
            
            h, w = combined.shape[:2]
            
            # FPS Calculation
            frames_processed += 1
            now = time.time()
            if now - last_fps_time >= 1.0:
                current_fps = frames_processed / (now - last_fps_time)
                frames_processed = 0
                last_fps_time = now
            
            # Draw visual elements
            draw_overlay(combined, current_fps, idx, w, h)
            
            # Draw separator line
            cv2.line(combined, (w//2, 0), (w//2, h), (50, 50, 50), 2)

            cv2.imshow("PS4 Stereo Viewer", combined)
            if camera._save():
                logger.info("Frame saved")
            if camera._quit():
                break

    except KeyboardInterrupt:
        pass
    finally:
        camera.release()
        cv2.destroyAllWindows()
