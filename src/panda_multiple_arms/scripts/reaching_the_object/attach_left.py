#!/usr/bin/env python3

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest

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
        if response.ok:
            rospy.loginfo(f"✅ Successfully attached {link_name_1} to {link_name_2}")
        else:
            rospy.logerr(f"❌ Failed to attach {link_name_1} to {link_name_2}")
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
        if response.ok:
            rospy.loginfo(f"✅ Successfully detached {link_name_1} from {link_name_2}")
        else:
            rospy.logerr(f"❌ Failed to detach {link_name_1} from {link_name_2}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def main():
    rospy.init_node('attach_table_to_robot', anonymous=True)

    # Define robot and table names
    robot_name = "panda_multiple_arms"
    arm_link = "left_arm_leftfinger"
    table_name = "table"
    table_link = "table_link"

    rospy.sleep(2)  # Give time for services to initialize

    # Attach the table to the left arm
    rospy.loginfo("Attempting to attach table to left arm...")
    attach_link(robot_name, arm_link, table_name, table_link)
    
    detach_link(robot_name, arm_link, table_name, table_link)
    
    attach_link(robot_name, arm_link, table_name, table_link)
    
    # Define robot and table names
    robot_name = "panda_multiple_arms"
    arm_link = "left_arm_rightfinger"
    table_name = "table"
    table_link = "table_link"
    
    attach_link(robot_name, arm_link, table_name, table_link)
    
    detach_link(robot_name, arm_link, table_name, table_link)
    
    attach_link(robot_name, arm_link, table_name, table_link)
    
    
    # Define robot and table names
    robot_name = "panda_multiple_arms"
    arm_link = "right_arm_leftfinger"
    table_name = "table"
    table_link = "table_link"

    rospy.sleep(2)  # Give time for services to initialize

    # Attach the table to the left arm
    rospy.loginfo("Attempting to attach table to left arm...")
    attach_link(robot_name, arm_link, table_name, table_link)
    
    detach_link(robot_name, arm_link, table_name, table_link)
    
    attach_link(robot_name, arm_link, table_name, table_link)
    
    # Define robot and table names
    robot_name = "panda_multiple_arms"
    arm_link = "right_arm_rightfinger"
    table_name = "table"
    table_link = "table_link"
    
    attach_link(robot_name, arm_link, table_name, table_link)
    
    detach_link(robot_name, arm_link, table_name, table_link)
    
    attach_link(robot_name, arm_link, table_name, table_link)

    #rospy.sleep(5)  # Wait before detaching for testing

    # Detach the table
    #rospy.loginfo("Attempting to detach table from left arm...")
    #detach_link(robot_name, arm_link, table_name, table_link)

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
