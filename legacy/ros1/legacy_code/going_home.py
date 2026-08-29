### This code is working as expected. It takes the robot to the home position as expected.

#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped

def move_to_named_pose(group, pose_name):
    group.set_named_target(pose_name)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    return plan

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('dual_arm_move_group_python', anonymous=True)

    # Initialize the MoveIt Commander for both arms
    left_arm_group = moveit_commander.MoveGroupCommander("left_panda_arm")
    right_arm_group = moveit_commander.MoveGroupCommander("right_panda_arm")

    # Go to the 'left_arm_home' pose
    rospy.loginfo("Moving left arm to 'left_arm_home' pose...")
    move_to_named_pose(left_arm_group, "left_arm_home")
    rospy.loginfo("Left arm is in 'left_arm_home' pose.")

    # Go to the 'right_arm_home' pose
    rospy.loginfo("Moving right arm to 'right_arm_home' pose...")
    move_to_named_pose(right_arm_group, "right_arm_home")
    rospy.loginfo("Right arm is in 'right_arm_home' pose.")

    rospy.loginfo("Both arms are in their home positions.")

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
