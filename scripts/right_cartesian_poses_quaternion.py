#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def move_to_cartesian_pose(group, pose_target):
    rospy.loginfo("Moving right arm to the specified Cartesian pose...")
    
    # Debugging: Print out the target pose
    rospy.loginfo(f"Target position: x={pose_target.position.x}, y={pose_target.position.y}, z={pose_target.position.z}")
    rospy.loginfo(f"Target orientation: x={pose_target.orientation.x}, y={pose_target.orientation.y}, z={pose_target.orientation.z}, w={pose_target.orientation.w}")
    
    # Increase planning time
    group.set_planning_time(10)
    
    # Plan and execute the motion
    plan = group.go(wait=True)
    group.stop()
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
    right_pose_target.position.y = 0.38
    right_pose_target.position.z = 1.5
    right_pose_target.orientation.x = 0.0
    right_pose_target.orientation.y = 0.0
    right_pose_target.orientation.z = 0.0
    right_pose_target.orientation.w = 0.99

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
