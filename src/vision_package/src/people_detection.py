import cv2
import numpy as np

class PeopleDetector:
    def __init__(self, zed_camera_handler):
        self.prototxt_path = 'src/vision_package/src/Trained_Models/MobileNetSSD_deploy.prototxt'
        self.model_path = 'src/vision_package/src/Trained_Models/MobileNetSSD_deploy.caffemodel'
        self.net = cv2.dnn.readNetFromCaffe(self.prototxt_path, self.model_path)
        self.zed_camera_handler = zed_camera_handler 
        
        self.classes = ["background", "aeroplane", "bicycle", "bird", "boat",
                        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                        "sofa", "train", "tvmonitor"]

    def detect_and_measure_depth(self, frame):
        # Frame| Blob
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)
        
        # Blob -> Network
        self.net.setInput(blob)
        detections = self.net.forward()
        
        results = []

        # Init Math | Info
        depth_map = self.zed_camera_handler.get_depth()

        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5:
                idx = int(detections[0, 0, i, 1])
                if self.classes[idx] != "person":
                    continue
                
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # Centroid
                centroidX = int((startX + endX) / 2)
                centroidY = int((startY + endY) / 2)

                # GDepth | Centroid
                depth = depth_map.get_value(centroidX, centroidY)[1]
                if depth:
                    depth_str = f"{depth:.2f}m"
                else:
                    depth_str = "N/A"

                # Append 
                results.append(((startX, startY, endX, endY), confidence, depth_str))

        return results

    def draw_detections(self, frame, detections_with_depth):
        # Bounding box 
        for ((startX, startY, endX, endY), confidence, depth_str) in detections_with_depth:
            # Bounding box | Probability
            text = f"{confidence:.2f}% | Depth: {depth_str}"
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
            cv2.putText(frame, text, (startX, startY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

