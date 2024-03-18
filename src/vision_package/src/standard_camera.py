import cv2

class StandardCamera:
    def __init__(self, camera_id=0, width=640, height=480):
        # Init. camera
        self.camera_id = camera_id
        self.camera = cv2.VideoCapture(camera_id)
        self.is_opened = self.camera.isOpened()

        if self.is_opened:
            # Resolution
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        else:
            print(f"Failed to open camera with ID {camera_id}")

    def capture_frame(self):
        # Frame
        if not self.is_opened:
            print("Camera is not opened.")
            return False, None

        ret, frame = self.camera.read()
        if ret:
            return True, frame
        else:
            return False, None

    def release(self):

        if self.is_opened:
            self.camera.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":
    camera = StandardCamera(camera_id=0, width=640, height=480)
    try:
        while True:
            ret, frame = camera.capture_frame()
            if ret:
                cv2.imshow("Frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
            else:
                print("Failed to capture frame")
    finally:
        camera.release()
