#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse

def detach_link(model_name_1, link_name_1, model_name_2, link_name_2):
    """
    Detach two links in Gazebo using the gazebo_ros_link_attacher service.
    """
    rospy.wait_for_service('/link_attacher_node/detach')
    try:
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2
        
        rospy.loginfo(f"Detaching {link_name_1} from {link_name_2}...")
        response = detach_srv(req)

        if response.ok:
            rospy.loginfo(f"Successfully detached {link_name_1} from {link_name_2}")
        else:
            rospy.logerr(f"Failed to detach {link_name_1} from {link_name_2}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def attach_link(model_name_1, link_name_1, model_name_2, link_name_2):
    """
    Attach two links using the gazebo_ros_link_attacher service.
    """
    rospy.wait_for_service('/link_attacher_node/attach')
    try:
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2
        
        rospy.loginfo(f"Attaching {link_name_1} to {link_name_2}...")
        response = attach_srv(req)

        if response.ok:
            rospy.loginfo(f"Successfully attached {link_name_1} to {link_name_2}")
        else:
            rospy.logerr(f"Failed to attach {link_name_1} to {link_name_2}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def reset_world():
    """
    Reset the world in Gazebo (positions, velocities, etc.).
    """
    rospy.wait_for_service('/gazebo/reset_world')
    try:
        reset_world_srv = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world_srv()
        rospy.loginfo("World has been reset.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to reset world: {e}")

def main():
    rospy.init_node('gazebo_detach_reset_attach_node')

    # Detach the right arm's end effector from the box
    detach_link('panda_multiple_arms', 'right_arm_link7', 'compliant_box', 'box_link')

    # Detach the left arm's end effector from the box
    detach_link('panda_multiple_arms', 'left_arm_link7', 'compliant_box', 'box_link')

    # Reset the Gazebo world
    reset_world()

    # Re-attach the right arm's end effector to the box
    attach_link('panda_multiple_arms', 'right_arm_link7', 'compliant_box', 'box_link')

    # Re-attach the left arm's end effector to the box
    attach_link('panda_multiple_arms', 'left_arm_link7', 'compliant_box', 'box_link')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
