from ai2thor.controller import Controller

# Standard controller (no start_xserver call needed)
controller = Controller(scene="FloorPlan1", width=640, height=480)
event = controller.step(action="MoveAhead")

# Save a frame to check if it worked
import cv2
cv2.imwrite("test.png", event.cv2img)