#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

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
    left_pose_target.position.x = 0.2999
    left_pose_target.position.y = 0.1000
    left_pose_target.position.z = 1.0998

    # Define Euler angles for the left arm orientation
    left_roll = 1.54 # Example value
    left_pitch = 0.0   # Example value
    left_yaw = 0.0     # Example value

    # Convert Euler angles to quaternion
    left_quat = euler_to_quaternion(left_roll, left_pitch, left_yaw)
    left_pose_target.orientation.x = left_quat[0]
    left_pose_target.orientation.y = left_quat[1]
    left_pose_target.orientation.z = left_quat[2]
    left_pose_target.orientation.w = left_quat[3]

    # Define the Cartesian target pose for the right arm
    right_pose_target = geometry_msgs.msg.Pose()
    right_pose_target.position.x = 0.2999
    right_pose_target.position.y = -0.1000
    right_pose_target.position.z = 1.2998

    # Define Euler angles for the right arm orientation
    right_roll = 0.0  # Example value
    right_pitch = 0.0 # Example value
    right_yaw = 0.0      # Example value

    # Convert Euler angles to quaternion
    right_quat = euler_to_quaternion(right_roll, right_pitch, right_yaw)
    right_pose_target.orientation.x = right_quat[0]
    right_pose_target.orientation.y = right_quat[1]
    right_pose_target.orientation.z = right_quat[2]
    right_pose_target.orientation.w = right_quat[3]

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
