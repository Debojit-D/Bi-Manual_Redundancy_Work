#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

def attach_links():
    rospy.init_node('link_attacher')

    # Wait for the service to be available
    rospy.wait_for_service('/link_attacher_node/attach')
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)

    # Attach the left arm's end effector to the left face of the box
    attach_left = AttachRequest()
    attach_left.model_name_1 = "robot1"  # Change to your robot's name
    attach_left.link_name_1 = "left_end_effector"  # The left end effector link name
    attach_left.model_name_2 = "compliant_box"
    attach_left.link_name_2 = "left_face"  # The link on the box to attach to

    # Attach the right arm's end effector to the right face of the box
    attach_right = AttachRequest()
    attach_right.model_name_1 = "robot2"  # Change to your robot's name
    attach_right.link_name_1 = "right_end_effector"  # The right end effector link name
    attach_right.model_name_2 = "compliant_box"
    attach_right.link_name_2 = "right_face"  # The link on the box to attach to

    # Call the service to attach the links
    try:
        attach_srv.call(attach_left)
        rospy.loginfo("Left end effector attached to box.")
        attach_srv.call(attach_right)
        rospy.loginfo("Right end effector attached to box.")
    except rospy.ServiceException as e:
        rospy.logerr("Failed to attach links: %s", e)

if __name__ == '__main__':
    try:
        attach_links()
    except rospy.ROSInterruptException:
        pass
