from ..interface import CameraInterface

class PS4EyeStereoCamera(CameraInterface):
    l_slice = [64, 1328]
    r_slice = [1328, 2592]
    h = 800
    def __init__(
        self, 
        video_id: int = 0, 
        width: int = 3448, 
        height: int = 808, 
        *args, 
        **kwargs
    ) -> None:
        super().__init__(video_id, width, height, *args, **kwargs)

    def _process_frame(self, idx, frame):
        left = frame[0:self.h, slice(*self.l_slice)]
        right = frame[0:self.h, slice(*self.r_slice)]
        return idx, left.copy(), right.copy()