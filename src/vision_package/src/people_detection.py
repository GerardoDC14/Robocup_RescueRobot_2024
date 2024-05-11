#!/usr/bin/env python3
import cv2
import numpy as np

class PeopleDetector:
    def __init__(self):
        # Paths to the model's configuration and weights files
        self.prototxt_path = '/home/robotec/rescue_ws/Robocup_RescueRobot_2024/src/vision_package/src/Trained_Models/MobileNetSSD_deploy.prototxt'
        self.model_path = '/home/robotec/rescue_ws/Robocup_RescueRobot_2024/src/vision_package/src/Trained_Models/MobileNetSSD_deploy.caffemodel'
        # Load the pre-trained model using OpenCV
        self.net = cv2.dnn.readNetFromCaffe(self.prototxt_path, self.model_path)
        
        # Define the list of class labels MobileNet SSD was trained to detect
        self.classes = ["background", "aeroplane", "bicycle", "bird", "boat",
                        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                        "sofa", "train", "tvmonitor"]

    def detect_people(self, frame):
        # Prepare the frame for detection
        (h, w) = frame.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(frame, (300, 300)), 0.007843, (300, 300), 127.5)

        # Perform detection
        self.net.setInput(blob)
        detections = self.net.forward()

        # List to store results
        results = []

        for i in np.arange(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            if confidence > 0.5:  # Filter out weak detections
                idx = int(detections[0, 0, i, 1])
                
                # Compute the (x, y)-coordinates of the bounding box
                box = detections[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                # Add the bounding box, class label, and confidence to the results list
                results.append(((startX, startY, endX, endY), self.classes[idx], confidence))

        return results

    def draw_detections(self, frame, detections):
        # Draw the bounding box, label, and confidence score on the frame
        for ((startX, startY, endX, endY), label, confidence) in detections:
            # Format the label and confidence for display
            text = f"{label}: {confidence:.2f}%"
            # Draw the bounding box and label the frame
            cv2.rectangle(frame, (startX, startY), (endX, endY), (0, 255, 0), 2)
            cv2.putText(frame, text, (startX, startY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
