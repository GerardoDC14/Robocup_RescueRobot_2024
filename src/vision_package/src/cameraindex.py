import cv2

def test_camera(camera_index):
    # Video Stream
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        print(f"Failed to open camera with index {camera_index}")
        return

    print(f"Successfully opened camera with index {camera_index}. Press 'q' to quit.")

    while True:
        # f-b-f
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab a frame")
            break

        # Display
        cv2.imshow(f'Camera {camera_index}', frame)

        # Close
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Test
test_camera(6)
