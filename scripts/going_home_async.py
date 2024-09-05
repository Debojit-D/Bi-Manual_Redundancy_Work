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

    # Initialize the MoveIt Commander for both arms as a combined group
    both_arms_group = moveit_commander.MoveGroupCommander("both_arms")

    # Go to the 'home' pose for both arms
    rospy.loginfo("Moving both arms to 'home' pose...")
    move_to_named_pose(both_arms_group, "both_arms_home")
    rospy.loginfo("Both arms are in the 'home' pose.")

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
