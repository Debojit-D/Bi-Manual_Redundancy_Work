#!/usr/bin/env python3

import sys
import rospy
import moveit_commander

def get_stored_joint_states():
    """Returns the stored joint states for both arms."""
    joint_states = {
        "left_arm": [2.297258895930298, 0.5694096828433883, -0.46355119174996506, -2.6511382754578543, -1.241805984144742, 1.7791346718596266, 1.5205211315748253],
        "right_arm": [-2.074780638644355, 0.5290955544871965, 0.2728170598451669, -2.645560031001368, 1.304932771962097, 1.675275882232139, -1.5501500864116853]
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
