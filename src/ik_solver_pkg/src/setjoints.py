#!/usr/bin/env python3

import rospy
import moveit_commander
import sys
from std_msgs.msg import Float64MultiArray

def joystick_callback(data):
    # data.data is a list of joint values [q1, q2, q3]
    joint_values = data.data
    joint_names = ['q1', 'q2', 'q3']  # Replace with your actual joint names
    
    # Creating a dictionary of joint names and their target values
    joint_target = {name: value for name, value in zip(joint_names, joint_values)}
    
    # Set the target joint state
    move_group.set_joint_value_target(joint_target)
    
    # Plan and execute
    move_group.go(wait=True)

def initialize_moveit():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_joystick_control')

    # Initialize the robot and the move group
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    move_group = moveit_commander.MoveGroupCommander("manipulator")  # Use your actual move group name

    return move_group

if __name__ == '__main__':
    move_group = initialize_moveit()
    
    # Subscribe to the topic where joystick angles are published
    rospy.Subscriber("joystick_angles", Float64MultiArray, joystick_callback)
    
    rospy.spin()
