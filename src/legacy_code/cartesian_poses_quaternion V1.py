#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg

def plan_cartesian_pose(group, pose_target, arm_name):
    rospy.loginfo(f"Planning {arm_name} to the specified Cartesian pose...")
    group.set_pose_target(pose_target)

    # Plan the motion
    plan = group.plan()

    # Check if the plan is a tuple and extract the trajectory if so
    if isinstance(plan, tuple):
        success = plan[0]
        plan = plan[1]  # Extract the RobotTrajectory
    else:
        success = bool(plan)

    if success and plan.joint_trajectory.points:
        rospy.loginfo(f"{arm_name} plan for the Cartesian pose is ready.")
        return plan
    else:
        rospy.logerr(f"Planning for {arm_name} to the Cartesian pose failed. Plan is invalid.")
        return None

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('dual_arm_cartesian_move', anonymous=True)

    # Initialize the MoveIt Commander for both arms
    left_arm_group = moveit_commander.MoveGroupCommander("left_panda_arm")
    right_arm_group = moveit_commander.MoveGroupCommander("right_panda_arm")

    # Increase planning time
    left_arm_group.set_planning_time(15)
    right_arm_group.set_planning_time(15)

    # Define the Cartesian target pose for the left arm
    left_pose_target = geometry_msgs.msg.Pose()
    left_pose_target.position.x = 0.399387
    left_pose_target.position.y = 0.400195
    left_pose_target.position.z = 1.3998
    left_pose_target.orientation.x = 0.0
    left_pose_target.orientation.y = 0
    left_pose_target.orientation.z = 0
    left_pose_target.orientation.w = 0.9999

    # Define the Cartesian target pose for the right arm
    right_pose_target = geometry_msgs.msg.Pose()
    right_pose_target.position.x = 0.30
    right_pose_target.position.y = 0.38
    right_pose_target.position.z = 1.497
    right_pose_target.orientation.x = -0.92
    right_pose_target.orientation.y = 0.38
    right_pose_target.orientation.z = 4.07
    right_pose_target.orientation.w = 6.416

    # Plan for the left arm first
    left_arm_plan = plan_cartesian_pose(left_arm_group, left_pose_target, "Left Arm")
    if left_arm_plan is None:
        return

    # Then plan for the right arm
    right_arm_plan = plan_cartesian_pose(right_arm_group, right_pose_target, "Right Arm")
    if right_arm_plan is None:
        return

    # Execute both plans simultaneously
    rospy.loginfo("Executing both plans simultaneously...")
    left_arm_group.execute(left_arm_plan, wait=False)
    right_arm_group.execute(right_arm_plan, wait=False)

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
