import cv2
import numpy as np

from abc import ABC, abstractmethod
from ..logger import get_logger
from .contants import ColorSpace, OutputFormat

logger = get_logger(namespace="video")

class CameraInterface(ABC):
    def __init__(
        self, 
        video_id: int, 
        width: int | None = None, 
        height: int | None = None, 
        *args,
        **kwargs
    ) -> None:
            self.video_id = video_id
            self.cap = cv2.VideoCapture(video_id)
            
            self.width, self.height = (width, height)

            if width:
                self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
            if height:
                self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))

            self.frame_idx = 0
            self._running = True 

            self.known_arguments(kwargs)
            self.__dict__.update(kwargs)

    def known_arguments(self, kwargs):
        self.output_fmt = OutputFormat.NUMPY
        self.color_mode = ColorSpace.BGR
        self.target_size = None
        if "output_fmt" in kwargs:
            if isinstance(kwargs["output_fmt"], str):
                self.output_fmt = OutputFormat[kwargs["output_fmt"].upper()]
                kwargs.pop("output_fmt")
            else:
                self.output_fmt = kwargs["output_fmt"]
                kwargs.pop("output_fmt")
        
        if "color_mode" in kwargs:
            if isinstance(kwargs["color_mode"], str):
                self.color_mode = ColorSpace[kwargs["color_mode"].upper()]
                kwargs.pop("color_mode")
            else:
                self.color_mode = kwargs["color_mode"]
                kwargs.pop("color_mode")

        if "target_size" in kwargs:
            self.target_size = tuple(kwargs["target_size"])
            kwargs.pop("target_size")

        logger.debug(f"Output format: {self.output_fmt}")
        logger.debug(f"Color mode: {self.color_mode}")
        logger.debug(f"Target size: {self.target_size}")


    def _transform(self, frame: np.ndarray):
        """Standardizes frame before yielding to the user."""
        if self.color_mode.value is not None:
            frame = cv2.cvtColor(frame, self.color_mode.value)
        
        if self.target_size:
            frame = cv2.resize(frame, self.target_size, interpolation=cv2.INTER_LINEAR)
        
        print(self.output_fmt == OutputFormat.PIL, self.output_fmt, OutputFormat.PIL)
        if self.output_fmt == OutputFormat.PIL:
            logger.info(f"Turning into an Image: {self.output_fmt}")
            return Image.fromarray(frame)
        
        elif self.output_fmt.value == OutputFormat.JPEG.value:
            logger.info(f"Turning into a JPEG: {self.output_fmt}")
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
            return buffer
        return frame

    def __next__(self):
        if not self._running:
            self.release()
            raise StopIteration

        ret, frame = self.cap.read()

        if self.frame_idx == 0:
            self.width = frame.shape[1]
            self.height = frame.shape[0]

        if not ret:
            self.release()
            raise StopIteration
        
        # Get raw fragments from child (e.g., Left/Right or just Main)
        raw_data = self._process_frame(self.frame_idx, frame)
        self.frame_idx += 1
        
        if isinstance(raw_data, (list, tuple)):
            logger.info("Multiple images")
            idx, *frames = raw_data
            return (idx, *(self._transform(f) for f in frames))
        
        logger.info("Single image")
        return self._transform(raw_data)

    def stop(self) -> None:
        self._running = False

    def __iter__(self) -> 'CameraInterface':
        return self

    def __len__(self) -> int:
        return self.frame_idx

    def __del__(self) -> None:
        self.release()

    def shape(self, orientation: str = "wh") -> tuple[int, int]:
        '''
        Returns the shape of the camera frame.

        Returns:
            tuple[int, int]: (width, height)
        '''
        if orientation.lower() == "wh":
            return (self.width, self.height)
        if orientation.lower() == "hw":
            return (self.height, self.width)
        if orientation.lower() == "whc":
            return (self.width, self.height, self.color_mode)
        if orientation.lower() == "hwc":
            return (self.height, self.width, self.color_mode)
        

    @property
    def current_frame(self) -> tuple[int, np.ndarray, np.ndarray]:
        return next(self)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()

    @abstractmethod
    def _process_frame(self, *args, **kwargs):
        pass

    def release(self):
        if self.cap.isOpened():
            self.cap.release()
            logger.info(f"Camera {self.video_id} released.")