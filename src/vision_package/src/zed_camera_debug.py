import pyzed.sl as sl
import cv2
import numpy as np

class ZedCameraHandler:
    def __init__(self):
        self.camera = sl.Camera()
        self.init_params = sl.InitParameters()
        self.init_params.camera_resolution = sl.RESOLUTION.HD720  # Res
        self.init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # Depth
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

            # Display frame | Debugging
            cv2.imshow("ZED Camera Frame", frame)
            cv2.waitKey(1)  

            return True, frame
        return False, None

    #Close window
    def close(self):
        self.camera.close()
        cv2.destroyAllWindows() 

if __name__ == "__main__":
    zed_handler = ZedCameraHandler()
    try:
        while True:
            ret, frame = zed_handler.capture_frame()
            if not ret:
                break
    except KeyboardInterrupt:
        print("Stopped")
    finally:
        zed_handler.close()
