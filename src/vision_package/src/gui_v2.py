#!/usr/bin/env python3
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import threading
import cv2
import tk_tools
import time
import queue
import rospy
from std_msgs.msg import String

from standard_camera import StandardCamera
from people_detection import PeopleDetector

class RescueRobotHMI:
    def __init__(self, window):
        self.window = window
        self.window.title("Carbabot | RHI | V2")
        self.window.geometry("1920x1080")

        # Attributes
        self.serial_data_queue = queue.Queue()
        self.stopEvent = threading.Event()


        # Init. cameras
        self.standard_camera0 = StandardCamera(camera_id=0)
        self.standard_camera1 = StandardCamera(camera_id=2)  
        

        # Init. people detection
        self.people_detector = PeopleDetector()

        # Setup GUI
        self.setup_gui_components()

        # ROS
        rospy.init_node('gui_subscriber', anonymous=True)
        rospy.Subscriber('serial_data', String, self.ros_callback)

        # ROS thread
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        # Gauges
        self.setup_gauges()

        self.thread = threading.Thread(target=self.video_loop, args=())
        self.thread.start()

        self.window.after(100, self.update_gauges_from_ros)

        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)

    def ros_callback(self, msg):
        self.serial_data_queue.put(msg.data)

    def ros_spin(self):
        rospy.spin()

    def setup_gui_components(self):
        label_font = ("Helvetica", 12, "bold")
        label_fg = "#FFFFFF"
        label_bg = "#333333"
        label_relief = "groove"
        label_borderwidth = 2

        self.window.grid_rowconfigure(0, weight=0)
        self.window.grid_rowconfigure(1, weight=0)
        self.window.grid_rowconfigure(2, weight=0)
        self.window.grid_rowconfigure(3, weight=0)
        self.window.grid_rowconfigure(4, weight=0)
        self.window.grid_columnconfigure(0, weight=1)
        self.window.grid_columnconfigure(1, weight=1)
        self.window.grid_columnconfigure(2, weight=1)

        # Front camera
        self.standard_frame_label0 = ttk.Label(self.window)
        self.standard_frame_label0.grid(row=1, column=0, padx=5, pady=5, sticky='wens')
        self.standard_camera_label0 = tk.Label(self.window, text="Front Camera", font=label_font,
                                            foreground=label_fg, background=label_bg,
                                            relief=label_relief, borderwidth=label_borderwidth)
        self.standard_camera_label0.grid(row=0, column=1, padx=5, pady=0, sticky='n')

        # Front camera
        self.standard_frame_label1 = ttk.Label(self.window)
        self.standard_frame_label1.grid(row=1, column=1, padx=5, pady=5, sticky='wens')
        self.standard_camera_label1 = tk.Label(self.window, text="Front Camera", font=label_font,
                                            foreground=label_fg, background=label_bg,
                                            relief=label_relief, borderwidth=label_borderwidth)
        self.standard_camera_label1.grid(row=0, column=0, padx=5, pady=0, sticky='n')

    def setup_gauges(self):
        self.rpm_gauge = tk_tools.RotaryScale(self.window, max_value=1023.0, size=100, unit='RPM')
        self.rpm_gauge.grid(row=2, column=1, padx=20, pady=10)
        
        self.voltage_gauge = tk_tools.RotaryScale(self.window, max_value=1023.0, size=100, unit='Voltage')
        self.voltage_gauge.grid(row=3, column=1, padx=20, pady=10)
        
        self.co2_gauge = tk_tools.RotaryScale(self.window, max_value=1023.0, size=100, unit='CO2')
        self.co2_gauge.grid(row=4, column=1, padx=20, pady=10)
        
        # Update | Gauges
        threading.Thread(target=self.update_gauges_from_ros, daemon=True).start()

        pass

    def update_gauges_from_ros(self):
        try:
            # Process | queue
            while not self.serial_data_queue.empty():
                line = self.serial_data_queue.get_nowait()
                data = line.split()
                rpm = float(data[1])
                voltage = float(data[3])
                co2 = float(data[5])

                # Update | Gauges
                self.rpm_gauge.set_value(rpm)
                self.voltage_gauge.set_value(voltage)
                self.co2_gauge.set_value(co2)
        except queue.Empty:
            pass
        finally:
            if not self.stopEvent.is_set():
                self.window.after(100, self.update_gauges_from_ros)


    def video_loop(self):
        while not self.stopEvent.is_set():
            # Handle camera 0
            ret, frame = self.standard_camera1.capture_frame()
            if ret:
                # Optionally perform detection and display for camera 0
                self.display_frame(frame, self.standard_frame_label1)
            
            # Handle camera 1
            ret, frame = self.standard_camera0.capture_frame()
            if ret:
                detections = self.people_detector.detect_people(frame)
                self.people_detector.draw_detections(frame, detections)
                self.display_frame(frame, self.standard_frame_label0)
            

    def display_camera_feed(self, camera, label):
        ret, frame = camera.capture_frame()
        if ret:
            self.display_frame(frame, label)

    def display_frame(self, frame, label, width=640, height=480):
        frame = cv2.resize(frame, (width, height))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        im = Image.fromarray(frame)
        img = ImageTk.PhotoImage(image=im)
        label.imgtk = img
        label.configure(image=img)
        label.grid_configure(sticky='wens')

    def on_closing(self):
        print("Closing...")
        self.stopEvent.set()
        self.standard_camera0.release()
        self.standard_camera1.release()
        self.window.destroy()

def main():
    root = tk.Tk()
    app = RescueRobotHMI(root)
    root.mainloop()

if __name__ == "__main__":
    main()