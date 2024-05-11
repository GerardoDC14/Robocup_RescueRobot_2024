#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import serial
import time

rospy.init_node('serial_node', anonymous=True)

pub = rospy.Publisher('serial_data', String, queue_size=10)

rate = rospy.Rate(100) 

ser = serial.Serial('/dev/ttyACM0', 115200)
time.sleep(2)

while not rospy.is_shutdown():
    if ser.in_waiting > 0:
        line = ser.readline().rstrip().decode('utf-8', errors='replace')
        rospy.loginfo(line) 
        pub.publish(line) 
        rate.sleep()  
