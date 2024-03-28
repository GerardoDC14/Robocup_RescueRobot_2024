#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Point

# Define link lengths
l1 = 0.50
l2 = 1.10
l3 = 4.00
l4 = 3.90
l5 = 1.769
l6 = 2.00

def publish_target_position():
    rospy.init_node('target_position_publisher', anonymous=True)
    pub = rospy.Publisher('/target_position', Point, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    target_position = Point()
    target_position.x = l2 + l4 + l5 + l6
    target_position.y = 0
    target_position.z = l1 + l3

    while not rospy.is_shutdown():
        rospy.loginfo(target_position)
        pub.publish(target_position)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_target_position()
    except rospy.ROSInterruptException:
        pass

