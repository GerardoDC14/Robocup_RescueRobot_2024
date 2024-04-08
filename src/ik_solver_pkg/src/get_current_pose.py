#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('get_current_pose', anonymous=True)

    # Move group | Manipulator
    group_name = "manipulator" 
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Get | Current pose 
    current_pose = move_group.get_current_pose()
    print("Current Pose of the end effector: ", current_pose.pose)

    # Shut down 
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
