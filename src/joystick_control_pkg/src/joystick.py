#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64MultiArray
import pygame
import sys

def map_joystick_value(joy_value, min_joy, max_joy, min_out, max_out):
    """
    Maps a joystick input value from its range to a specified output range.
    """
    return ((joy_value - min_joy) / (max_joy - min_joy)) * (max_out - min_out) + min_out

def initialize_joystick():
    pygame.init()
    pygame.joystick.init()
    joystick = None
    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        joystick.init()
        rospy.loginfo(f"Joystick {joystick.get_name()} initialized")
    else:
        rospy.loginfo("No joystick detected")
        pygame.quit()
        sys.exit()
    return joystick

def joystick_publisher():
    pub = rospy.Publisher('joystick_angles', Float64MultiArray, queue_size=10)
    rospy.init_node('joystick_publisher_node', anonymous=True)
    rate = rospy.Rate(1)  # 10hz
    joystick = initialize_joystick()

    while not rospy.is_shutdown():
        pygame.event.pump()  # Process event queue.
        data = Float64MultiArray()

        # Assuming the joystick axes map directly to the robot's joint angles
        data.data = [
            map_joystick_value(joystick.get_axis(0), -1, 1, -1, 1),  # q1
            map_joystick_value(joystick.get_axis(1), 0, 1, 0, 1),      # q2
            map_joystick_value(joystick.get_axis(2), 0, 1, 0, 1)       # q3
        ]

        pub.publish(data)
        rate.sleep()

if __name__ == '__main__':
    try:
        joystick_publisher()
    except rospy.ROSInterruptException:
        pass
    finally:
        pygame.quit()
