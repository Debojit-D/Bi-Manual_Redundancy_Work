#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from controller_manager_msgs.srv import SwitchController
from std_srvs.srv import Empty  # Correct import for Empty service

def attach_link(model_name_1, link_name_1, model_name_2, link_name_2):
    """Attach two links using the gazebo_ros_link_attacher service."""
    rospy.wait_for_service('/link_attacher_node/attach')
    try:
        attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2
        response = attach_srv(req)
        if response:
            rospy.loginfo(f"Successfully attached {link_name_1} to {link_name_2}")
        else:
            rospy.logerr(f"Failed to attach {link_name_1} to {link_name_2}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def detach_link(model_name_1, link_name_1, model_name_2, link_name_2):
    """Detach two links using the gazebo_ros_link_attacher service."""
    rospy.wait_for_service('/link_attacher_node/detach')
    try:
        detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        req = AttachRequest()
        req.model_name_1 = model_name_1
        req.link_name_1 = link_name_1
        req.model_name_2 = model_name_2
        req.link_name_2 = link_name_2
        response = detach_srv(req)
        if response:
            rospy.loginfo(f"Successfully detached {link_name_1} from {link_name_2}")
        else:
            rospy.logerr(f"Failed to detach {link_name_1} from {link_name_2}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def reset_world():
    """Reset the world in Gazebo (positions, velocities, etc.)."""
    rospy.wait_for_service('/gazebo/reset_world')
    try:
        reset_world_srv = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world_srv()
        rospy.loginfo("World has been reset.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to reset world: {e}")

def stop_arm_controllers():
    """Stop the robot's arm controllers to freeze the arms after movement."""
    rospy.wait_for_service('/controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        stop_controllers = ['right_arm_trajectory_controller', 'left_arm_trajectory_controller']
        
        # SwitchController requires 5 arguments: start_controllers, stop_controllers, strictness, start_asap, and timeout
        response = switch_controller([], stop_controllers, 2, True, 0.0)
        if response.ok:
            rospy.loginfo("Successfully stopped the arm controllers.")
        else:
            rospy.logerr("Failed to stop the arm controllers.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('attach_detach_reset_node')

    # Attach the right arm's end effector to the box
    attach_link('panda_multiple_arms', 'right_arm_link7', 'compliant_box', 'box_link')

    # Attach the left arm's end effector to the box
    attach_link('panda_multiple_arms', 'left_arm_link7', 'compliant_box', 'box_link')

    # Detach the right arm's end effector from the box
    detach_link('panda_multiple_arms', 'right_arm_link7', 'compliant_box', 'box_link')

    # Detach the left arm's end effector from the box
    detach_link('panda_multiple_arms', 'left_arm_link7', 'compliant_box', 'box_link')

    # Reset the Gazebo world
    reset_world()

    # Attach the right arm's end effector to the box again
    attach_link('panda_multiple_arms', 'right_arm_link7', 'compliant_box', 'box_link')

    # Attach the left arm's end effector to the box again
    attach_link('panda_multiple_arms', 'left_arm_link7', 'compliant_box', 'box_link')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
