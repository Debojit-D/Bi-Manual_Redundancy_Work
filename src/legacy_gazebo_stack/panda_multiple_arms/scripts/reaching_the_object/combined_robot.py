#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import tf.transformations as tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def euler_to_quaternion(roll, pitch, yaw):
    """Convert Euler angles (radians) to quaternion."""
    return tf.quaternion_from_euler(roll, pitch, yaw)

def open_gripper(gripper_pub, joint_names):
    """
    Open the specified gripper by sending a single-point JointTrajectory command.
    joint_names: e.g. ['left_arm_finger_joint1', 'left_arm_finger_joint2'].
    """
    rospy.loginfo("Opening the gripper...")

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = [0.02, 0.02]  # 'Open' position for each finger
    point.time_from_start = rospy.Duration(1.0)

    trajectory.points.append(point)
    gripper_pub.publish(trajectory)

    rospy.sleep(1.5)
    rospy.loginfo("Gripper opened successfully.")

def close_gripper(gripper_pub, joint_names):
    """
    Close the specified gripper by sending a single-point JointTrajectory command.
    joint_names: e.g. ['left_arm_finger_joint1', 'left_arm_finger_joint2'].
    """
    rospy.loginfo("Closing the gripper...")

    trajectory = JointTrajectory()
    trajectory.joint_names = joint_names

    point = JointTrajectoryPoint()
    point.positions = [0.0, 0.0]  # 'Closed' position for each finger
    point.time_from_start = rospy.Duration(1.0)

    trajectory.points.append(point)
    gripper_pub.publish(trajectory)

    rospy.sleep(1.5)
    rospy.loginfo("Gripper closed successfully.")

def move_arm_to_pose(arm_group, pose_target, arm_name="arm", wait=True):
    """
    Plan and execute a Cartesian move for the given arm group.
    If wait=False, the motion will be asynchronous (allows parallel moves).
    """
    rospy.loginfo(f"Moving {arm_name} to x={pose_target.position.x:.2f}, y={pose_target.position.y:.2f}, z={pose_target.position.z:.2f}")

    arm_group.set_max_velocity_scaling_factor(0.8)
    arm_group.set_max_acceleration_scaling_factor(0.8)

    arm_group.set_pose_target(pose_target)
    success = arm_group.go(wait=wait)  # Set wait=False for simultaneous moves

    arm_group.stop()
    arm_group.clear_pose_targets()

    if success:
        rospy.loginfo(f"{arm_name} reached target pose successfully.")
    else:
        rospy.logerr(f"{arm_name} failed to reach target pose.")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('dual_arm_control', anonymous=True)

    # 1. Create MoveGroupCommander for each arm
    left_arm_group = moveit_commander.MoveGroupCommander("left_arm")
    right_arm_group = moveit_commander.MoveGroupCommander("right_arm")

    # 2. Create Publishers for each gripper
    left_gripper_pub = rospy.Publisher("/left_hand_controller/command", JointTrajectory, queue_size=10)
    right_gripper_pub = rospy.Publisher("/right_hand_controller/command", JointTrajectory, queue_size=10)

    # Give time for pubs/subs to connect
    rospy.sleep(1.0)

    # 3. Optional: Open both grippers before motion
    open_gripper(left_gripper_pub, ["left_arm_finger_joint1", "left_arm_finger_joint2"])
    open_gripper(right_gripper_pub, ["right_arm_finger_joint1", "right_arm_finger_joint2"])

    # 4. Define planning time for each arm
    left_arm_group.set_planning_time(10)
    right_arm_group.set_planning_time(10)

    # 5. Define target poses for each arm
    left_pose = geometry_msgs.msg.Pose()
    left_pose.position.x = 0.35
    left_pose.position.y = 0.4
    left_pose.position.z = 0.34

    # Example Euler angles -> quaternion (roll, pitch, yaw)
    # left arm orientation
    l_roll, l_pitch, l_yaw = 1.571, -0.7855, 0.0
    l_quat = euler_to_quaternion(l_roll, l_pitch, l_yaw)
    left_pose.orientation.x = l_quat[0]
    left_pose.orientation.y = l_quat[1]
    left_pose.orientation.z = l_quat[2]
    left_pose.orientation.w = l_quat[3]

    right_pose = geometry_msgs.msg.Pose()
    right_pose.position.x = 0.35
    right_pose.position.y = -0.45
    right_pose.position.z = 0.34

    # right arm orientation
    r_roll, r_pitch, r_yaw = -1.54, 0.7855, 0.0
    r_quat = euler_to_quaternion(r_roll, r_pitch, r_yaw)
    right_pose.orientation.x = r_quat[0]
    right_pose.orientation.y = r_quat[1]
    right_pose.orientation.z = r_quat[2]
    right_pose.orientation.w = r_quat[3]

    # 6. Move arms
    # Option A: Move arms sequentially
    # move_arm_to_pose(left_arm_group, left_pose, arm_name="left_arm", wait=True)
    # move_arm_to_pose(right_arm_group, right_pose, arm_name="right_arm", wait=True)

    # Option B: Move arms simultaneously
    # Start left arm move (don't wait)
    move_arm_to_pose(left_arm_group, left_pose, arm_name="left_arm", wait=False)
    # Start right arm move (don't wait)
    move_arm_to_pose(right_arm_group, right_pose, arm_name="right_arm", wait=False)

    # If you want to wait until both are done, you can sleep or do additional checks
    rospy.sleep(3.0)  # Let them move in parallel for a few seconds

    # 7. Now close both grippers
    close_gripper(left_gripper_pub, ["left_arm_finger_joint1", "left_arm_finger_joint2"])
    close_gripper(right_gripper_pub, ["right_arm_finger_joint1", "right_arm_finger_joint2"])

    # Shut down MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
