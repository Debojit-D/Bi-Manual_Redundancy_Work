#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState
from controller_manager_msgs.srv import SwitchController

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

def freeze_box(model_name):
    """Set the model state of the box to make it immovable."""
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state_srv = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        model_state = ModelState()
        model_state.model_name = model_name
        model_state.pose.position.x = 0
        model_state.pose.position.y = 0
        model_state.pose.position.z = 1
        model_state.pose.orientation.x = 0
        model_state.pose.orientation.y = 0
        model_state.pose.orientation.z = 0
        model_state.pose.orientation.w = 1
        model_state.twist.linear.x = 0
        model_state.twist.linear.y = 0
        model_state.twist.linear.z = 0
        model_state.twist.angular.x = 0
        model_state.twist.angular.y = 0
        model_state.twist.angular.z = 0
        model_state.reference_frame = 'world'
        response = set_model_state_srv(model_state)
        if response.success:
            rospy.loginfo(f"Successfully set {model_name} to immovable state")
        else:
            rospy.logerr(f"Failed to set {model_name} to immovable state")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

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
    rospy.init_node('attach_and_freeze_box')

    # Attach the right arm's end effector to the box
    attach_link('panda_multiple_arms', 'right_arm_link7', 'compliant_box', 'box_link')

    # Attach the left arm's end effector to the box
    attach_link('panda_multiple_arms', 'left_arm_link7', 'compliant_box', 'box_link')

    # Set the box to immovable
    #freeze_box('compliant_box')

    # Stop the arm controllers to prevent further motion
    #stop_arm_controllers()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
