#!/usr/bin/env python3

import sys
import math
import rospy
import moveit_commander

def get_stored_joint_states():
    """Returns the stored joint states for both arms."""
    joint_states = {
        "left_arm": [
        0.3645371110643265,
        -0.8025719462084532,
        -2.03897638954126,
        -2.442246401849028,
        1.072538894387179,
        2.1330330563753224,
        1.3111046117652199
    ],
        "right_arm": [2.1971002523181813, 0.4047568079514874, -0.4936629870410323, -2.372335548992551, -1.350768531499954, 1.7214538285562835, 0.3819643044433836]
    }
    return joint_states

def command_rotation(group_name, angle_offset):
    """
    Rotates the first joint of the given MoveIt group by the specified angle_offset (in radians).
    Assumes that the first joint (index 0) controls the rotation.
    """
    group = moveit_commander.MoveGroupCommander(group_name)
    current_joint_values = group.get_current_joint_values()
    # Rotate the first joint by adding the angle_offset
    current_joint_values[0] += angle_offset
    group.set_joint_value_target(current_joint_values)
    success = group.go(wait=True)
    group.stop()

    if success:
        rospy.loginfo(f"{group_name} successfully rotated by {angle_offset} radians.")
    else:
        rospy.logerr(f"{group_name} failed to rotate.")

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
    
    # Calculate 110 degrees in radians
    angle_rad = math.radians(110)  # ≈1.9199 radians
    
    # For left arm: 110 degree rotation clockwise (using a negative angle)
    # For right arm: 110 degree rotation anticlockwise (using a positive angle)
    rospy.loginfo("Performing the specified 110 degree rotations...")
    command_rotation("left_panda_arm", -angle_rad)
    command_rotation("right_panda_arm", angle_rad)
    
    # Now move the arms to the stored joint states
    rospy.loginfo("Commanding stored joint states...")
    command_joint_states("left_panda_arm", joint_states["left_arm"])
    command_joint_states("right_panda_arm", joint_states["right_arm"])

    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
