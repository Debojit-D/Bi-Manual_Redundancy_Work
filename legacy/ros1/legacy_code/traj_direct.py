#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from sensor_msgs.msg import JointState

def call_compute_ik(group_name, current_position, target_z, current_orientation):
    """
    Call the compute_ik service to get joint positions for a target z position.
    """
    # Create a request for the compute_ik service
    ik_service_name = '/compute_ik'
    rospy.wait_for_service(ik_service_name)
    compute_ik_service = rospy.ServiceProxy(ik_service_name, GetPositionIK)
    
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = group_name
    ik_request.ik_request.pose_stamped.header.frame_id = "world"
    
    # Set the target position (modifying only the z-axis)
    ik_request.ik_request.pose_stamped.pose.position.x = current_position[0]
    ik_request.ik_request.pose_stamped.pose.position.y = current_position[1]
    ik_request.ik_request.pose_stamped.pose.position.z = target_z
    
    # Set the orientation (keep it the same)
    ik_request.ik_request.pose_stamped.pose.orientation.x = current_orientation[0]
    ik_request.ik_request.pose_stamped.pose.orientation.y = current_orientation[1]
    ik_request.ik_request.pose_stamped.pose.orientation.z = current_orientation[2]
    ik_request.ik_request.pose_stamped.pose.orientation.w = current_orientation[3]
    
    # No need for constraints or robot state here
    ik_request.ik_request.avoid_collisions = False
    ik_request.ik_request.timeout.secs = 5
    
    try:
        response = compute_ik_service(ik_request)
        if response.error_code.val == 1:  # Success
            return response.solution.joint_state.position
        else:
            rospy.logerr("IK failed with error code: {}".format(response.error_code.val))
            return None
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s" % e)
        return None

def send_trajectory():
    # Initialize the ROS node
    rospy.init_node('send_trajectory_to_dual_arms')

    # Publishers for both arms
    left_arm_pub = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
    right_arm_pub = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=10)

    rospy.sleep(1)  # Give some time for the publishers to connect

    # Current positions and orientations
    current_left_position = [0.29967559917129444, 0.2587901622949299, 1.097020746832174]  # X, Y, Z for left arm
    current_right_position = [0.2999945346343475, -0.25866455520133935, 1.0965758148312734]  # X, Y, Z for right arm
    current_left_orientation = [-0.6961566957411088, -2.2910830153599626e-05, -0.0006202776885731454, -0.7178895943705441]  # Quaternion for left arm
    current_right_orientation = [-0.6962412014222492, 0.00013486178472227328, 2.5633206280684107e-05, 0.7178078925432207]  # Quaternion for right arm

    # Target Z-axis for both arms
    target_z = 1.5

    # Call compute_ik for both arms
    left_joint_positions = call_compute_ik('left_panda_arm', current_left_position, target_z, current_left_orientation)
    right_joint_positions = call_compute_ik('right_panda_arm', current_right_position, target_z, current_right_orientation)

    if left_joint_positions is None or right_joint_positions is None:
        rospy.logerr("Failed to compute IK for one or both arms.")
        return

    # Define the trajectory for the left arm
    left_arm_trajectory = JointTrajectory()
    left_arm_trajectory.joint_names = ['left_arm_joint1', 'left_arm_joint2', 'left_arm_joint3',
                                       'left_arm_joint4', 'left_arm_joint5', 'left_arm_joint6', 
                                       'left_arm_joint7']

    # Create a trajectory point for the left arm
    left_arm_point = JointTrajectoryPoint()
    left_arm_point.positions = left_joint_positions
    left_arm_point.time_from_start = rospy.Duration(2)  # Move over 2 seconds
    left_arm_trajectory.points.append(left_arm_point)

    # Define the trajectory for the right arm
    right_arm_trajectory = JointTrajectory()
    right_arm_trajectory.joint_names = ['right_arm_joint1', 'right_arm_joint2', 'right_arm_joint3',
                                        'right_arm_joint4', 'right_arm_joint5', 'right_arm_joint6',
                                        'right_arm_joint7']

    # Create a trajectory point for the right arm
    right_arm_point = JointTrajectoryPoint()
    right_arm_point.positions = right_joint_positions
    right_arm_point.time_from_start = rospy.Duration(2)  # Move over 2 seconds
    right_arm_trajectory.points.append(right_arm_point)

    # Publish the trajectories to both arms
    left_arm_pub.publish(left_arm_trajectory)
    right_arm_pub.publish(right_arm_trajectory)

    rospy.loginfo("Trajectories sent to both arms.")

if __name__ == '__main__':
    try:
        send_trajectory()
    except rospy.ROSInterruptException:
        pass
