import pyzed.sl as sl
import cv2
import numpy as np

class ZedCameraHandler:
    def __init__(self):
        self.camera = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA
        self.open()

    def open(self):
        if self.camera.open(self.init_params) != sl.ERROR_CODE.SUCCESS:
            print("Failed to open ZED camera")
            exit(1)

    def capture_frame(self):
        image = sl.Mat()
        runtime_parameters = sl.RuntimeParameters()
        if self.camera.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.camera.retrieve_image(image, sl.VIEW.LEFT)
            frame = image.get_data()
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
            return True, frame
        return False, None
    
    def get_depth(self):
        depth_map = sl.Mat()
        if self.camera.retrieve_measure(depth_map, sl.MEASURE.DEPTH) == sl.ERROR_CODE.SUCCESS:
            return depth_map
        else:
            return None
    
    def close(self):
        self.camera.close()
