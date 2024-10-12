#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf_trans
import numpy as np

def move_to_cartesian_pose(group, pose_target):
    rospy.loginfo("Moving right arm to the specified Cartesian pose...")
    
    # Debugging: Print out the target pose
    rospy.loginfo("Target position: x={}, y={}, z={}".format(pose_target.position.x, pose_target.position.y, pose_target.position.z))
    rospy.loginfo("Target orientation: x={}, y={}, z={}, w={}".format(pose_target.orientation.x, pose_target.orientation.y, pose_target.orientation.z, pose_target.orientation.w))
    
    # Set the target pose for the group
    group.set_pose_target(pose_target)
    
    # Increase planning time
    group.set_planning_time(10)
    
    # Plan and execute the motion
    plan = group.go(wait=True)
    group.stop()  # Ensure that there is no residual movement
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

    # Define the Cartesian target pose for the right arm using Euler angles
    right_pose_target = geometry_msgs.msg.Pose()
    right_pose_target.position.x = 2.3
    right_pose_target.position.y = -2.2
    right_pose_target.position.z = 2.0

    # Euler angles
    roll = -1.54
    pitch = 0.0
    yaw = 0.0

    # Convert Euler angles to quaternion
    quaternion = tf_trans.quaternion_from_euler(roll, pitch, yaw)
    # Normalize quaternion
    norm = np.linalg.norm([quaternion[0], quaternion[1], quaternion[2], quaternion[3]])
    quaternion = [x / norm for x in quaternion]
    right_pose_target.orientation.x = quaternion[0]
    right_pose_target.orientation.y = quaternion[1]
    right_pose_target.orientation.z = quaternion[2]
    right_pose_target.orientation.w = quaternion[3]

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
