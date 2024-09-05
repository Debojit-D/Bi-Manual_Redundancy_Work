#!/usr/bin/env python

import rospy
from controller_manager_msgs.srv import SwitchController

def stop_controllers():
    """Stop the joint_state_controller, left_arm_trajectory_controller, and right_arm_trajectory_controller."""
    try:
        rospy.wait_for_service('/controller_manager/switch_controller')
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        
        # Controllers to stop
        stop_controllers = [
            'joint_state_controller', 
            'left_arm_trajectory_controller', 
            'right_arm_trajectory_controller'
        ]
        
        # SwitchController requires 5 arguments: start_controllers, stop_controllers, strictness, start_asap, and timeout
        response = switch_controller([], stop_controllers, 2, True, 0.0)
        
        if response.ok:
            rospy.loginfo("Successfully stopped the controllers: joint_state_controller, left_arm_trajectory_controller, right_arm_trajectory_controller.")
        else:
            rospy.logerr("Failed to stop the controllers.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

if __name__ == '__main__':
    rospy.init_node('stop_all_controllers')
    stop_controllers()
