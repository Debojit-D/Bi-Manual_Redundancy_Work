#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

def get_stored_joint_states():
    """Returns the stored joint states for both arms."""
    joint_states = {
        "left_arm": [1.1873478865081737, -0.34361007872620936, -2.8515249249158945, -2.466567173455826, 1.436445389837661, 1.6513742082925582, -1.97375918172429],
        "right_arm": [2.1971002523181813, 0.4047568079514874, -0.4936629870410323, -2.372335548992551, -1.350768531499954, 1.7214538285562835, 0.3819643044433836]

    }
    return joint_states

def command_joint_states(group_name, joint_values):
    """Send the given joint values to the specified MoveIt group."""
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_joint_value_target(joint_values)
    success = group.go(wait=True)
    group.stop()
    
    if success:
        rospy.loginfo(f"{group_name} successfully moved to stored joint state.")
    else:
        rospy.logerr(f"{group_name} failed to move to the stored joint state.")

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('command_joint_states', anonymous=True)

    joint_states = get_stored_joint_states()
    
    rospy.loginfo("Commanding stored joint states...")
    command_joint_states("left_panda_arm", joint_states["left_arm"])
    command_joint_states("right_panda_arm", joint_states["right_arm"])

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
