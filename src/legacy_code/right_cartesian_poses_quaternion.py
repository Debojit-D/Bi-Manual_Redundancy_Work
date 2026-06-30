#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import numpy as np

def normalize_quaternion(quaternion):
    """ Normalize the quaternion to ensure it's valid for use in MoveIt. """
    norm = np.linalg.norm([quaternion.x, quaternion.y, quaternion.z, quaternion.w])
    quaternion.x /= norm
    quaternion.y /= norm
    quaternion.z /= norm
    quaternion.w /= norm
    return quaternion

def move_to_cartesian_pose(group, pose_target):
    rospy.loginfo("Moving right arm to the specified Cartesian pose...")
    
    # Debugging: Print out the target pose
    rospy.loginfo(f"Target position: x={pose_target.position.x}, y={pose_target.position.y}, z={pose_target.position.z}")
    rospy.loginfo(f"Target orientation: x={pose_target.orientation.x}, y={pose_target.orientation.y}, z={pose_target.orientation.z}, w={pose_target.orientation.w}")
    
    # Normalize the quaternion for safety
    pose_target.orientation = normalize_quaternion(pose_target.orientation)

    # Set the target pose for the group
    group.set_pose_target(pose_target)
    
    # Increase planning time and attempts
    group.set_planning_time(20)  # Increased planning time
    group.set_num_planning_attempts(10)  # More attempts to find a valid plan

    # Optional: Set a specific planner to use
    group.set_planner_id("RRTConnectkConfigDefault")
    
    # Plan and execute the motion
    plan = group.go(wait=True)
    group.stop()  # Ensure no residual movement
    group.clear_pose_targets()

    if plan:
        rospy.loginfo("Right arm has reached the target Cartesian pose.")
    else:
        rospy.logerr("Right arm failed to reach the target Cartesian pose.")
    return plan

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('right_arm_cartesian_move', anonymous=True)

    # Initialize the MoveIt Commander for the right arm
    right_arm_group = moveit_commander.MoveGroupCommander("right_panda_arm")

    # Define the Cartesian target pose for the right arm
    right_pose_target = geometry_msgs.msg.Pose()
    right_pose_target.position.x = 0.2
    right_pose_target.position.y = -0.913
    right_pose_target.position.z = 1.431764
    right_pose_target.orientation.x = 0.0692
    right_pose_target.orientation.y = -0.7677
    right_pose_target.orientation.z = -0.4086
    right_pose_target.orientation.w = -0.4886

    # Move the right arm to the Cartesian target pose
    rospy.loginfo("Starting Cartesian movement for the right arm...")

    move_to_cartesian_pose(right_arm_group, right_pose_target)

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
