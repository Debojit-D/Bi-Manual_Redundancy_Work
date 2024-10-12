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

def move_robot_base(group, angle_in_degrees):
    """Rotate the robot's base joint by a specific angle."""
    # Get the current joint values
    joint_values = group.get_current_joint_values()

    # Assuming the base joint is the first joint, set the base rotation
    base_joint_index = 0  # Adjust if necessary based on your robot configuration
    joint_values[base_joint_index] += angle_in_degrees * (3.14159 / 180.0)  # Convert degrees to radians

    # Plan and execute the motion for the base
    group.set_joint_value_target(joint_values)
    success = group.go(wait=True)
    group.stop()

    if success:
        rospy.loginfo(f"Robot base rotated by {angle_in_degrees} degrees.")
    else:
        rospy.logerr("Failed to rotate the robot base.")

def move_to_cartesian_pose(group, pose_target, arm_name, collision_check=True):
    rospy.loginfo(f"Moving {arm_name} to the specified Cartesian pose...")

    if not collision_check:
        group.set_planner_id("RRTConnectkConfigDefault")
        group.set_planning_pipeline_id("ompl")

    group.set_pose_target(pose_target)

    # Set velocity and acceleration scaling for faster motion
    group.set_max_velocity_scaling_factor(0.8)  # Increase speed
    group.set_max_acceleration_scaling_factor(0.8)  # Increase acceleration


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

    # Define the base group to move the robot's base joint
    robot_group = moveit_commander.MoveGroupCommander("right_panda_arm")  # Adjust if necessary

    right_arm_group.set_planning_time(10)

    # Step 1: Rotate the base by -30 degrees
    base_rotation_angle = -60  # -30 degrees
    rospy.loginfo("Rotating the robot base by -30 degrees...")
    move_robot_base(robot_group, base_rotation_angle)

    # Define the Cartesian target pose for the right arm (first step)
    right_pose_target = geometry_msgs.msg.Pose()
    right_pose_target.position.x = 0.2999
    right_pose_target.position.y = -0.2210  # First step target
    right_pose_target.position.z = 1.0998

    right_roll = -1.54  # Example value
    right_pitch = 0.0   # Example value
    right_yaw = 0.0     # Example value

    right_quat = euler_to_quaternion(right_roll, right_pitch, right_yaw)
    right_pose_target.orientation.x = right_quat[0]
    right_pose_target.orientation.y = right_quat[1]
    right_pose_target.orientation.z = right_quat[2]
    right_pose_target.orientation.w = right_quat[3]

    rospy.loginfo("Starting Cartesian movement for the right arm to the first position...")

    # Step 2: Move the right arm to the first target position (y = -0.2210)
    move_to_cartesian_pose(right_arm_group, right_pose_target, "Right Arm", collision_check=False)

    # Update the position for the second step (y = -0.1510)
    right_pose_target.position.y = -0.1510

    rospy.loginfo("Moving to the final target position...")

    # Step 3: Move the right arm to the final target position (y = -0.1510)
    right_arm_success = move_to_cartesian_pose(right_arm_group, right_pose_target, "Right Arm", collision_check=False)

    if right_arm_success:
        rospy.loginfo("Right arm has reached its final Cartesian target pose.")
    else:
        rospy.logerr("Right arm failed to reach the final Cartesian target pose.")

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
