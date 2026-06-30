#!/usr/bin/env python

import rospy
from std_srvs.srv import Empty
from controller_manager_msgs.srv import SwitchController
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

def reset_simulation():
    """
    Reset the simulation time in Gazebo.
    """
    rospy.wait_for_service('/gazebo/reset_simulation')
    try:
        reset_sim = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
        reset_sim()
        rospy.loginfo("Simulation time has been reset.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to reset simulation: {e}")

def reset_world():
    """
    Detach links and reset the world in Gazebo (positions, velocities, etc.).
    """
    # Detach the links before resetting the world
    rospy.loginfo("Detaching links before resetting the world...")
    detach_link('panda_multiple_arms', 'right_arm_link7', 'compliant_box', 'box_link')
    detach_link('panda_multiple_arms', 'left_arm_link7', 'compliant_box', 'box_link')

    # Reset the world
    rospy.wait_for_service('/gazebo/reset_world')
    try:
        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        reset_world()
        rospy.loginfo("World has been reset.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to reset world: {e}")

def restart_controllers(robot_name):
    """
    Restart the controllers for the robot.
    """
    rospy.wait_for_service('/controller_manager/switch_controller')
    try:
        switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        
        start_controllers = [f'{robot_name}_arm_controller', f'{robot_name}_gripper_controller']
        stop_controllers = [f'{robot_name}_arm_controller', f'{robot_name}_gripper_controller']
        
        response = switch_controller(start_controllers=start_controllers, stop_controllers=stop_controllers, strictness=2)
        
        if response.ok:
            rospy.loginfo(f"Controllers for {robot_name} restarted successfully.")
        else:
            rospy.logerr(f"Failed to restart contr2ollers for {robot_name}.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Failed to switch controllers: {e}")

def main():
    rospy.init_node('gazebo_reset_with_detach_node')

    # Prompt user to select action
    rospy.loginfo("Select an option:")
    rospy.loginfo("1. Reset Simulation Time")
    rospy.loginfo("2. Detach Links and Reset World (positions, velocities)")
    rospy.loginfo("3. Restart Controllers for Franka Robots")
    
    option = input("Enter your choice (1, 2, or 3): ")

    if option == '1':
        reset_simulation()
    elif option == '2':
        # Detach links and reset world
        reset_world()
    elif option == '3':
        # Assuming you have two Franka robots: left_arm and right_arm
        restart_controllers("left_arm")
        restart_controllers("right_arm")
    else:
        rospy.loginfo("Invalid option selected. Exiting.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
