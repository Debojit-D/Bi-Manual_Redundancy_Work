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

    # Set velocity and acceleration scaling for faster motion
    group.set_max_velocity_scaling_factor(0.8)  # Increase speed
    group.set_max_acceleration_scaling_factor(0.8)  # Increase acceleration

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
    rospy.init_node('left_arm_cartesian_move', anonymous=True)

    # Initialize the MoveIt Commander for the left arm
    left_arm_group = moveit_commander.MoveGroupCommander("left_panda_arm")

    # Define the base group to move the robot's base joint
    robot_group = moveit_commander.MoveGroupCommander("left_panda_arm")  # Adjust if necessary

    left_arm_group.set_planning_time(10)

    # Step 1: Rotate the base by 30 degrees
    base_rotation_angle = 30  # 30 degrees
    rospy.loginfo("Rotating the robot base by 30 degrees...")
    move_robot_base(robot_group, base_rotation_angle)

    # Define the Cartesian target pose for the left arm (first step)
    left_pose_target = geometry_msgs.msg.Pose()
    left_pose_target.position.x = 0.2999
    left_pose_target.position.y = 0.2210  # First step target
    left_pose_target.position.z = 1.0998

    left_roll = 1.54  # Example value
    left_pitch = 0.0  # Example value
    left_yaw = 0.0    # Example value

    left_quat = euler_to_quaternion(left_roll, left_pitch, left_yaw)
    left_pose_target.orientation.x = left_quat[0]
    left_pose_target.orientation.y = left_quat[1]
    left_pose_target.orientation.z = left_quat[2]
    left_pose_target.orientation.w = left_quat[3]

    rospy.loginfo("Starting Cartesian movement for the left arm to the first position...")

    # Step 2: Move the left arm to the first target position (y = 0.2210)
    move_to_cartesian_pose(left_arm_group, left_pose_target, "Left Arm", collision_check=False)

    # Update the position for the second step (y = 0.1510)
    left_pose_target.position.y = 0.1510

    rospy.loginfo("Moving to the final target position...")

    # Step 3: Move the left arm to the final target position (y = 0.1510)
    left_arm_success = move_to_cartesian_pose(left_arm_group, left_pose_target, "Left Arm", collision_check=False)
    
    if left_arm_success:
        rospy.loginfo("Left arm has reached its final Cartesian target pose.")
    else:
        rospy.logerr("Left arm failed to reach the final Cartesian target pose.")

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
