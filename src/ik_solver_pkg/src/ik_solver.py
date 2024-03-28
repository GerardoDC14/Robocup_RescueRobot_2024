#!/usr/bin/env python3
import rospy
import sys  # Add this if you're using sys.argv in moveit_commander.roscpp_initialize
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState  # Add this line

class IKSolver:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('ik_solver', anonymous=True)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group_name = "manipulator"  # Update this with your actual group name
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)

        # Publisher for the robot's joint states
        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

        # Subscriber to the target position
        rospy.Subscriber("/target_position", Point, self.target_position_callback)

    def target_position_callback(self, msg):
        # Prepare the pose target
        pose_target = geometry_msgs.msg.Pose()
        pose_target.position.x = msg.x
        pose_target.position.y = msg.y
        pose_target.position.z = msg.z
        pose_target.orientation.w = 1.0  # Assuming no specific orientation

        # Set the pose target
        self.move_group.set_pose_target(pose_target)

        # Plan to the new pose
        plan_success, plan, planning_time, error_code = self.move_group.plan()
        if plan_success:
            # Execute the plan
            self.move_group.execute(plan, wait=True)
        else:
            rospy.logerr("Planning failed with error code: %s", error_code)

        # Clear the targets after planning
        self.move_group.clear_pose_targets()

if __name__ == '__main__':
    ik_solver = IKSolver()
    rospy.spin()
