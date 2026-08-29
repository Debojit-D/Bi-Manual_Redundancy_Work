#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def move_to_cartesian_pose(group, pose_target, arm_name):
    rospy.loginfo(f"Moving {arm_name} to the specified Cartesian pose...")
    group.set_pose_target(pose_target)

    # Plan and execute the motion
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    if plan:
        rospy.loginfo(f"{arm_name} has reached the target Cartesian pose.")
    else:
        rospy.logerr(f"{arm_name} failed to reach the target Cartesian pose.")
    return plan

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('dual_arm_cartesian_move', anonymous=True)

    # Initialize the MoveIt Commander for both arms
    left_arm_group = moveit_commander.MoveGroupCommander("left_panda_arm")
    right_arm_group = moveit_commander.MoveGroupCommander("right_panda_arm")

    right_arm_group.set_planning_time(10)

    # Define the Cartesian target pose for the left arm
    left_pose_target = geometry_msgs.msg.Pose()
    left_pose_target.position.x = 0.499387
    left_pose_target.position.y = 0.200195
    left_pose_target.position.z = 1.2998
    left_pose_target.orientation.x = 0.0
    left_pose_target.orientation.y = 0
    left_pose_target.orientation.z = 0
    left_pose_target.orientation.w = 0.999

    # Define the Cartesian target pose for the right arm
    right_pose_target = geometry_msgs.msg.Pose()
    right_pose_target.position.x = 0.30
    right_pose_target.position.y = 0.38
    right_pose_target.position.z = 1.497
    right_pose_target.orientation.x = -0.92
    right_pose_target.orientation.y = 0.38
    right_pose_target.orientation.z = 4.07
    right_pose_target.orientation.w = 6.416

    # Move both arms to the Cartesian target poses simultaneously
    rospy.loginfo("Starting simultaneous Cartesian movement...")

    left_arm_success = move_to_cartesian_pose(left_arm_group, left_pose_target, "Left Arm")
    right_arm_success = move_to_cartesian_pose(right_arm_group, right_pose_target, "Right Arm")
    
    if left_arm_success and right_arm_success:
        rospy.loginfo("Both arms have reached their Cartesian target poses.")
    else:
        rospy.logerr("One or both arms failed to reach the Cartesian target poses.")

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
