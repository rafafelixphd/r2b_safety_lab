# constants.py
import cv2

from enum import Enum, auto

class ColorSpace(Enum):
    RGB = cv2.COLOR_BGR2RGB
    BGR = None
    GRAY = cv2.COLOR_BGR2GRAY
    HSV = cv2.COLOR_BGR2HSV
    YUV = cv2.COLOR_BGR2YUV

class OutputFormat(Enum):
    NUMPY = auto()
    PIL = auto()
    JPEG = auto()