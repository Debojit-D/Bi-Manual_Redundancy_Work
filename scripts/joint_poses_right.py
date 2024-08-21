#!/usr/bin/env python

import sys
import rospy
import moveit_commander

def move_to_joint_pose(group, joint_goal):
    rospy.loginfo("Moving right arm to the specified joint positions...")
    group.go(joint_goal, wait=True)
    group.stop()  # Ensure no residual movement
    current_joints = group.get_current_joint_values()

    # Check if the target was reached within a small tolerance
    if all(abs(current_joints[i] - joint_goal[i]) < 0.01 for i in range(len(joint_goal))):
        rospy.loginfo("Right arm has reached the target joint positions.")
        return True
    else:
        rospy.logerr("Right arm failed to reach the target joint positions.")
        return False

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('right_arm_joint_space_move', anonymous=True)

    # Initialize the MoveIt Commander for the right arm
    right_arm_group = moveit_commander.MoveGroupCommander("right_panda_arm")

    # Define the joint space target for the right arm
    joint_goal = right_arm_group.get_current_joint_values()
    joint_goal[0] = 0.0   # Right Arm Joint 1
    joint_goal[1] = -1.5  # Right Arm Joint 2
    joint_goal[2] = 0.0   # Right Arm Joint 3
    joint_goal[3] = -1.5  # Right Arm Joint 4
    joint_goal[4] = 1.0   # Right Arm Joint 5
    joint_goal[5] = 1.0   # Right Arm Joint 6
    joint_goal[6] = 1.5   # Right Arm Joint 7

    # Move the right arm to the joint space target positions
    rospy.loginfo("Starting joint space movement for the right arm...")

    move_to_joint_pose(right_arm_group, joint_goal)

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
