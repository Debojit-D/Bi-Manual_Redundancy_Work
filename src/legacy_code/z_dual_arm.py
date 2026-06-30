#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    quaternion = tf.quaternion_from_euler(roll, pitch, yaw)
    return quaternion

def move_to_cartesian_pose(group, pose_target_left, pose_target_right):
    rospy.loginfo("Moving both arms to the specified Cartesian poses...")

    # Set the pose targets for both arms
    group.set_pose_target(pose_target_left, "left_panda_arm")
    group.set_pose_target(pose_target_right, "right_panda_arm")

    # Disable collision checking temporarily
    group.set_planning_pipeline_id("ompl")  # Example using OMPL planner
    group.set_planner_id("RRTConnectkConfigDefault")

    # Plan and execute the motion
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    if plan:
        rospy.loginfo("Both arms have reached the target Cartesian poses.")
    else:
        rospy.logerr("Both arms failed to reach the target Cartesian poses.")
    return plan

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('dual_arm_cartesian_move', anonymous=True)

    # Initialize the MoveIt Commander for both arms as one group
    both_arms_group = moveit_commander.MoveGroupCommander("both_arms")

    both_arms_group.set_planning_time(10)

    # Define the Cartesian target pose for the right arm
    right_pose_target = geometry_msgs.msg.Pose()
    right_pose_target.position.x = 0.2999
    right_pose_target.position.y = -0.2500
    right_pose_target.position.z = 1.0998

    right_roll = -1.54   # Example value
    right_pitch = 0.0  # Example value
    right_yaw = 0.0    # Example value

    right_quat = euler_to_quaternion(right_roll, right_pitch, right_yaw)
    right_pose_target.orientation.x = right_quat[0]
    right_pose_target.orientation.y = right_quat[1]
    right_pose_target.orientation.z = right_quat[2]
    right_pose_target.orientation.w = right_quat[3]

    # Define the Cartesian target pose for the left arm
    left_pose_target = geometry_msgs.msg.Pose()
    left_pose_target.position.x = 0.2999
    left_pose_target.position.y = 0.2500
    left_pose_target.position.z = 1.0998

    left_roll = 1.54  # Example value
    left_pitch = 0.0  # Example value
    left_yaw = 0.0    # Example value

    left_quat = euler_to_quaternion(left_roll, left_pitch, left_yaw)
    left_pose_target.orientation.x = left_quat[0]
    left_pose_target.orientation.y = left_quat[1]
    left_pose_target.orientation.z = left_quat[2]
    left_pose_target.orientation.w = left_quat[3]

    rospy.loginfo("Starting Cartesian movement for both arms...")
    both_arms_success = move_to_cartesian_pose(both_arms_group, left_pose_target, right_pose_target)

    if both_arms_success:
        rospy.loginfo("Both arms have reached their Cartesian target poses.")
    else:
        rospy.logerr("Both arms failed to reach their Cartesian target poses.")

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
