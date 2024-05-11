#!/usr/bin/env python3
import tkinter
import cv2
import PIL.Image, PIL.ImageTk
import time
import tk_tools
import rospy
from std_msgs.msg import String
from tkinter import Frame

from people_detection import PeopleDetector
from qr_code import  QRCodeDetector

class tkCamera(Frame):
    def __init__(self, window, video_source=0, enable_people_detection=False, enable_qr_detection=False):
        super().__init__(window)
        self.window = window
        self.video_source = video_source
        self.vid = MyVideoCapture(self.video_source)

        self.enable_people_detection = enable_people_detection
        self.enable_qr_detection = enable_qr_detection

        if self.enable_people_detection:
            self.people_detector = PeopleDetector()
        if self.enable_qr_detection:
            self.qr_detector = QRCodeDetector()

        self.canvas = tkinter.Canvas(window, width=self.vid.width, height=self.vid.height)
        self.canvas.pack()

        self.btn_snapshot = tkinter.Button(window, text="Snapshot", command=self.snapshot)
        self.btn_snapshot.pack(anchor=tkinter.CENTER, expand=True)

        self.delay = 15
        self.update_widget()

    def snapshot(self):
        ret, frame = self.vid.get_frame()
        if ret:
            cv2.imwrite("frame-" + time.strftime("%d-%m-%Y-%H-%M-%S") + ".jpg", frame)  # Save original frame

    def update_widget(self):
        ret, frame = self.vid.get_frame()
        if ret:
            if self.enable_people_detection:
                detections = self.people_detector.detect_people(frame)
                self.people_detector.draw_detections(frame, detections)
            if self.enable_qr_detection:
                frame = self.qr_detector.detect_and_decode(frame)
            
            self.image = PIL.Image.fromarray(frame)
            self.photo = PIL.ImageTk.PhotoImage(image=self.image)
            self.canvas.create_image(0, 0, image=self.photo, anchor=tkinter.NW)
        self.window.after(self.delay, self.update_widget)

class App:
    def __init__(self, window, window_title, video_source1=0, video_source2=2):
        self.window = window
        self.window.title(window_title)
        self.initialize_ros()

        # Configure the grid layout
        self.configure_grid()

        # Setup cameras in their own frames
        self.camera_frame1 = tkinter.Frame(self.window)
        self.camera_frame1.grid(row=0, column=0, sticky='nsew')
        self.camera_frame2 = tkinter.Frame(self.window)
        self.camera_frame2.grid(row=0, column=1, sticky='nsew')

        self.vid1 = tkCamera(self.camera_frame1, video_source1, enable_people_detection=True,enable_qr_detection=True)
        self.vid1.pack(expand=True, fill='both')  # Changed to pack

        self.vid2 = tkCamera(self.camera_frame2, video_source2, enable_people_detection=False,enable_qr_detection=False)
        self.vid2.pack(expand=True, fill='both')  # Changed to pack

        # Setup gauges
        self.setup_gauges()

        self.window.mainloop()

    def configure_grid(self):
        # Configure rows and columns
        self.window.grid_columnconfigure(0, weight=1)
        self.window.grid_columnconfigure(1, weight=1)
        self.window.grid_rowconfigure(0, weight=1)
        self.window.grid_rowconfigure(1, weight=0)  # Adjust if necessary
        self.window.grid_rowconfigure(2, weight=0)
        self.window.grid_rowconfigure(3, weight=0)

    def initialize_ros(self):
        rospy.init_node('tkinter_gui', anonymous=True)
        rospy.Subscriber("serial_data", String, self.ros_callback)

    def setup_gauges(self):
        # Set up gauges in the specified grid locations
        self.rpm_gauge = tk_tools.RotaryScale(self.window, max_value=100.0, size=100, unit='RPM')
        self.rpm_gauge.grid(row=1, column=1, padx=10, pady=10, sticky='nsew')
        
        self.voltage_gauge = tk_tools.RotaryScale(self.window, max_value=240.0, size=100, unit='Voltage')
        self.voltage_gauge.grid(row=2, column=1, padx=10, pady=10, sticky='nsew')
        
        self.co2_gauge = tk_tools.RotaryScale(self.window, max_value=5000.0, size=100, unit='ppm')
        self.co2_gauge.grid(row=3, column=1, padx=10, pady=10, sticky='nsew')

    def ros_callback(self, msg):
        data = msg.data.split(',')
        if len(data) == 1:  # Assuming the data is formatted as "RPM,100,Voltage,220,CO2,400"
            self.rpm_gauge.set_value(float(data[0]))
            self.voltage_gauge.set_value(float(data[0]))
            self.co2_gauge.set_value(float(data[0]))


class MyVideoCapture:
    def __init__(self, video_source=0):
        self.vid = cv2.VideoCapture(video_source)
        if not self.vid.isOpened():
            raise ValueError("Unable to open video source", video_source)

        self.width = 400
        self.height = 300

    def get_frame(self):
        if self.vid.isOpened():
            ret, frame = self.vid.read()
            if ret:
                frame = cv2.resize(frame, (self.width, self.height))
                return (ret, cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            else:
                return (ret, None)
        else:
            return (ret, None)

    def __del__(self):
        if self.vid.isOpened():
            self.vid.release()


App(tkinter.Tk(), "Tkinter and OpenCV", 0,2)
