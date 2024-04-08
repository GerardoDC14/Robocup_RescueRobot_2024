#!/usr/bin/env python3
import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('go_to_pose_goal', anonymous=True)

    # Move group | Manipulator
    group_name = "manipulator" 
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Define | Target pose
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = 0
    pose_goal.position.y = 0
    pose_goal.position.z = 0
    pose_goal.orientation.x = 0
    pose_goal.orientation.y = 0
    pose_goal.orientation.z = 0
    pose_goal.orientation.w = 1

    # Set | Target pose 
    move_group.set_pose_target(pose_goal)

    # Call | Planner
    success = move_group.go(wait=True)
    # Stop | No residual movement
    move_group.stop()
    # Clear | Targets
    move_group.clear_pose_targets()

    if success:
        print("Successfully moved to the pose goal")
    else:
        print("Failed to move to the pose goal")

    # Shut down 
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
