#!/usr/bin/env python

import sys
import rospy
import moveit_commander

def move_to_named_pose(group, pose_name, arm_name):
    rospy.loginfo(f"Planning {arm_name} to {pose_name}...")
    group.set_named_target(pose_name)

    # Plan the motion
    plan = group.plan()

    # Check if the plan is a tuple (which is typical in some configurations)
    if isinstance(plan, tuple):
        success = plan[0]
        plan = plan[1]
    else:
        success = bool(plan)

    if not success or plan.joint_trajectory.points == []:
        rospy.logerr(f"Planning for {arm_name} to {pose_name} failed. Plan is invalid.")
        return None

    rospy.loginfo(f"{arm_name} plan for {pose_name} is ready.")
    return plan

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('dual_arm_move_group_python', anonymous=True)

    # Initialize the MoveIt Commander for both arms
    left_arm_group = moveit_commander.MoveGroupCommander("left_panda_arm")
    right_arm_group = moveit_commander.MoveGroupCommander("right_panda_arm")

    # Plan the movements for both arms
    rospy.loginfo("Planning simultaneous movement...")
    
    left_arm_plan = move_to_named_pose(left_arm_group, "left_arm_home", "Left Arm")
    right_arm_plan = move_to_named_pose(right_arm_group, "right_arm_home", "Right Arm")
    
    if left_arm_plan and right_arm_plan:
        rospy.loginfo("Both arms have planned successfully. Starting execution...")

        # Execute the plans simultaneously
        left_arm_group.execute(left_arm_plan, wait=False)
        right_arm_group.execute(right_arm_plan, wait=False)
        
        rospy.loginfo("Both arms are moving to their home positions.")
    else:
        rospy.logerr("Planning failed for one or both arms.")

    # Wait for both arms to complete their movements
    left_arm_group.stop()
    right_arm_group.stop()

    # Shut down MoveIt cleanly
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
