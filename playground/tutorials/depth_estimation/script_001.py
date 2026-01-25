import cv2
import r2b.video
import r2b.depth

camera = r2b.video.PS4EyeStereoCamera(video_id=0)
processor = r2b.depth.vanilla.StereoProcessor()

print("R2B Research Stream Active...")

for idx, left, right in camera:
    # 1. Run your adjusted algorithm
    min_d, avg_d, max_d, depth_vis = processor.compute_3d_metrics(left, right)
    
    # 2. High-Visibility Visualization
    depth_color = cv2.applyColorMap(depth_vis, cv2.COLORMAP_OCEAN)
    
    # Overlay metrics (R2B Safety Style)
    display = cv2.resize(left, (640, 400))
    cv2.putText(display, f"MIN: {min_d:.2f}m", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)
    cv2.putText(display, f"AVG: {avg_d:.2f}m", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
    
    cv2.imshow("R2B Safe Distance Monitor", display)
    cv2.imshow("Filtered Disparity", depth_vis)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()