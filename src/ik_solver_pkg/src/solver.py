#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import moveit_msgs.msg

class RobotMover:
    def __init__(self, group_name, reference_frame):
        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)
        
        # Initialize the rospy node if it hasn't been initialized yet
        if not rospy.get_node_uri():
            rospy.init_node('robot_mover', anonymous=True)

        # Instantiate a RobotCommander object
        self.robot = moveit_commander.RobotCommander()

        # Instantiate a PlanningSceneInterface object
        self.scene = moveit_commander.PlanningSceneInterface()

        # Instantiate a MoveGroupCommander object for the specified group
        self.move_group = moveit_commander.MoveGroupCommander(group_name)

        # Set the reference frame for pose targets
        self.move_group.set_pose_reference_frame(reference_frame)

        # Set the planner
        self.move_group.set_planner_id("RRTConnectkConfigDefault")
    
    def get_current_joint_values(self):
        return self.move_group.get_current_joint_values()


    def move_to_pose(self, pose_name):
        print(f"Moving to {pose_name}")
        
        # Set the pose target
        self.move_group.set_named_target(pose_name)

        # Plan to the pose target
        plan = self.move_group.plan()

        # Execute the plan
        self.move_group.execute(plan[1], wait=True)

        # Ensure there are no residual movements
        self.move_group.stop()
        self.move_group.clear_pose_targets()

        print(f"Arrived at {pose_name}")

if __name__ == '__main__':
    try:
        robot_mover = RobotMover(group_name="manipulator", reference_frame="Spacecraft")

        robot_mover.move_to_pose("zero_pose")
        rospy.sleep(5)  # Wait for 5 seconds
        
        # Get and print the current joint values
        joint_values = robot_mover.get_current_joint_values()
        print("Current joint values:", joint_values)

        robot_mover.move_to_pose("max_x")
        rospy.sleep(5)  # Wait for 5 seconds

        # Get and print the joint values again
        joint_values = robot_mover.get_current_joint_values()
        print("Joint values at max_x:", joint_values)

        print("Movement to max_x completed")

        robot_mover.move_to_pose("max_z")
        rospy.sleep(5)  # Wait for 5 seconds

        # Get and print the joint values again
        joint_values = robot_mover.get_current_joint_values()
        print("Joint values at max_z:", joint_values)

        print("Movement to max_z completed")

    except rospy.ROSInterruptException:
        pass
    finally:
        # Ensure clean shutdown
        moveit_commander.roscpp_shutdown()

