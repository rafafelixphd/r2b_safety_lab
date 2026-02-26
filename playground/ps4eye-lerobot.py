from lerobot.cameras.ps4eye import PS4EyeCamera, PS4EyeCameraConfig

# ---------------------------------------------------------------------------
# Two-camera pattern: left + right eye as independent cameras
#   - Both share the same physical VideoCapture (opened only once)
#   - Each returns its own cropped eye slice on .read()
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import sys
    import cv2

    index = int(sys.argv[1]) if len(sys.argv) > 1 else 1

    cameras = {
        "left": PS4EyeCamera(
            PS4EyeCameraConfig(index_or_path=index, fps=30, width=3448, height=808, eye="left")
        ),
        "right": PS4EyeCamera(
            PS4EyeCameraConfig(index_or_path=index, fps=30, width=3448, height=808, eye="right")
        ),
    }

    for name, cam in cameras.items():
        cam.connect(warmup=False)
        print(f"{name} connected")

    try:
        while True:
            left_img  = cameras["left"].read()
            right_img = cameras["right"].read()

            # Stack side-by-side for preview
            # combined = cv2.hconcat([
                # cv2.cvtColor(left_img,  cv2.COLOR_RGB2BGR),
                # cv2.cvtColor(right_img, cv2.COLOR_RGB2BGR),
            # ])
            cv2.imshow("PS4 Eye — Right", right_img)
            cv2.imshow("PS4 Eye — Left", left_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        for cam in cameras.values():
            cam.disconnect()
        cv2.destroyAllWindows()