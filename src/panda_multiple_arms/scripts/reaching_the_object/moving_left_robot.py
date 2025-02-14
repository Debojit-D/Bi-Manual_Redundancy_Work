#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import FollowJointTrajectoryGoal, FollowJointTrajectoryAction
import actionlib

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles to quaternion."""
    return tf.quaternion_from_euler(roll, pitch, yaw)

def open_gripper(gripper_pub):
    """Send a trajectory command to open the gripper."""
    rospy.loginfo("Opening the gripper...")

    trajectory = JointTrajectory()
    trajectory.joint_names = ["left_arm_finger_joint1", "left_arm_finger_joint2"]  # Both fingers

    point = JointTrajectoryPoint()
    point.positions = [0.03, 0.03]  # Open position (adjust if needed)
    point.time_from_start = rospy.Duration(1.0)

    trajectory.points.append(point)

    gripper_pub.publish(trajectory)
    rospy.sleep(1.5)  # Allow time for execution

    rospy.loginfo("Gripper successfully opened.")
    
def close_gripper(gripper_pub):
    """Send a trajectory command to open the gripper."""
    rospy.loginfo("Opening the gripper...")

    trajectory = JointTrajectory()
    trajectory.joint_names = ["left_arm_finger_joint1", "left_arm_finger_joint2"]  # Both fingers

    point = JointTrajectoryPoint()
    point.positions = [0.00, 0.00]  # Open position (adjust if needed)
    point.time_from_start = rospy.Duration(1.0)

    trajectory.points.append(point)

    gripper_pub.publish(trajectory)
    rospy.sleep(1.5)  # Allow time for execution

    rospy.loginfo("Gripper successfully opened.")

def move_to_cartesian_pose(group, pose_target):
    """Move the robot arm to the specified Cartesian pose."""
    rospy.loginfo(f"Moving to target Cartesian pose: y = {pose_target.position.y}")

    group.set_max_velocity_scaling_factor(0.8)
    group.set_max_acceleration_scaling_factor(0.8)
    group.set_pose_target(pose_target)

    success = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    if success:
        rospy.loginfo(f"Successfully reached target pose at y = {pose_target.position.y}.")
    else:
        rospy.logerr(f"Failed to reach target pose at y = {pose_target.position.y}.")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('cartesian_move', anonymous=True)

    # Initialize MoveIt commander for the left arm
    arm_group = moveit_commander.MoveGroupCommander("left_arm")

    # Publisher for gripper command
    gripper_pub = rospy.Publisher(
        "/left_hand_controller/command", JointTrajectory, queue_size=10
    )

    arm_group.set_planning_time(10)

    # Step 0: Open the gripper before moving
    rospy.sleep(1)  # Ensure node is fully initialized
    open_gripper(gripper_pub)

    # Define the common Cartesian pose structure
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = 0.35
    pose_target.position.z = 0.34

    # Define orientation using Euler angles
    roll, pitch, yaw = 1.571, -0.7855, 0.0
    quat = euler_to_quaternion(roll, pitch, yaw)
    pose_target.orientation.x = quat[0]
    pose_target.orientation.y = quat[1]
    pose_target.orientation.z = quat[2]
    pose_target.orientation.w = quat[3]

    # Step 1: Move to y = 0.4
    pose_target.position.y = 0.4
    move_to_cartesian_pose(arm_group, pose_target)

    # Step 2: Move to y = 0.32
    pose_target.position.y = 0.32
    move_to_cartesian_pose(arm_group, pose_target)
    
    close_gripper(gripper_pub)

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
