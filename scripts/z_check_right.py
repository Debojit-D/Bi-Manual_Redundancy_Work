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

def move_to_cartesian_pose(group, pose_target, arm_name, collision_check=True):
    rospy.loginfo(f"Moving {arm_name} to the specified Cartesian pose...")

    if not collision_check:
        group.set_planner_id("RRTConnectkConfigDefault")  # Example planner, you can customize this
        group.set_planning_pipeline_id("ompl")  # Using OMPL pipeline

        # Disable collision checking for this planning group
        # group.set_path_constraints(None)
        # group.allow_replanning(False)
        # rospy.loginfo(f"Collision checking disabled for {arm_name}.")

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
    rospy.init_node('right_arm_cartesian_move', anonymous=True)

    # Initialize the MoveIt Commander for the right arm
    right_arm_group = moveit_commander.MoveGroupCommander("right_panda_arm")

    right_arm_group.set_planning_time(10)

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

    rospy.loginfo("Starting Cartesian movement for the right arm...")

    # Disable collision checking during planning
    right_arm_success = move_to_cartesian_pose(right_arm_group, right_pose_target, "Right Arm", collision_check=False)
    
    if right_arm_success:
        rospy.loginfo("Right arm has reached its Cartesian target pose.")
    else:
        rospy.logerr("Right arm failed to reach the Cartesian target pose.")

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
