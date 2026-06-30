### Working Code ##### 

#!/usr/bin/env python

import rospy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest
from moveit_msgs.msg import OrientationConstraint
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
import numpy as np
import tf.transformations as tf
import csv

def get_box_state():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        box_state = get_model_state('compliant_box', 'world')  # 'world' frame
        position = box_state.pose.position
        orientation = box_state.pose.orientation
        return position, orientation
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None, None

def call_compute_ik(group_name, target_position, target_orientation):
    """ Call the compute_ik service to get joint positions for a target position and orientation. """
    ik_service_name = '/compute_ik'
    rospy.wait_for_service(ik_service_name)
    compute_ik_service = rospy.ServiceProxy(ik_service_name, GetPositionIK)
    
    ik_request = GetPositionIKRequest()
    ik_request.ik_request.group_name = group_name
    ik_request.ik_request.pose_stamped.header.frame_id = "world"
    
    # Set the target position (modify only the z-axis)
    ik_request.ik_request.pose_stamped.pose.position.x = target_position[0]
    ik_request.ik_request.pose_stamped.pose.position.y = target_position[1]
    ik_request.ik_request.pose_stamped.pose.position.z = target_position[2]
    
    # Set the orientation (keep it the same as passed in target_orientation)
    ik_request.ik_request.pose_stamped.pose.orientation.x = target_orientation[0]
    ik_request.ik_request.pose_stamped.pose.orientation.y = target_orientation[1]
    ik_request.ik_request.pose_stamped.pose.orientation.z = target_orientation[2]
    ik_request.ik_request.pose_stamped.pose.orientation.w = target_orientation[3]
    
    ik_request.ik_request.avoid_collisions = False
    ik_request.ik_request.timeout.secs = 5
    
    try:
        response = compute_ik_service(ik_request)
        if response.error_code.val == 1:  # Success
            return response.solution.joint_state.position
        else:
            rospy.logerr(f"IK failed with error code: {response.error_code.val}")
            return None
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None
    
def set_box_state(position, orientation):
    """Set the box position and orientation in Gazebo."""
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        box_state = ModelState()
        box_state.model_name = 'compliant_box'
        box_state.pose.position.x = position[0]
        box_state.pose.position.y = position[1]
        box_state.pose.position.z = position[2]
        box_state.pose.orientation.x = orientation[0]
        box_state.pose.orientation.y = orientation[1]
        box_state.pose.orientation.z = orientation[2]
        box_state.pose.orientation.w = orientation[3]
        
        set_model_state(box_state)
        rospy.loginfo(f"Moved box to Position: {position}, Orientation: {orientation}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")

def move_box_and_arms(z_trajectory, orientation, left_arm_pub, right_arm_pub, marker_pub, delay=0.01):
    """ Move the box along the trajectory and simultaneously command the robot arms to move along with the box. """
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD
    marker.scale.x = 0.01
    marker.color.a = 1.0
    marker.color.g = 1.0

    # Get the initial box state for fixed x and y coordinates
    initial_pos, _ = get_box_state()
    fixed_x = initial_pos.x
    fixed_y = initial_pos.y

    for z in z_trajectory:
        position = [fixed_x, fixed_y, z]
        
        # Move the box to the next position (only z is changing)
        set_box_state(position, orientation)

        # Use IK to compute the positions for both arms based on the box's position
        left_joint_positions = call_compute_ik('left_panda_arm', position, orientation)
        right_joint_positions = call_compute_ik('right_panda_arm', position, orientation)

        if left_joint_positions is None or right_joint_positions is None:
            rospy.logerr("Failed to compute IK for one or both arms.")
            continue

        # Publish the trajectories to the left and right arms
        send_trajectory(left_arm_pub, 'left_arm', left_joint_positions)
        send_trajectory(right_arm_pub, 'right_arm', right_joint_positions)

        # Publish marker for visualization
        point = Point(x=fixed_x, y=fixed_y, z=z)
        marker.points.append(point)
        marker_pub.publish(marker)

        rospy.sleep(delay)

def send_trajectory(arm_pub, arm_name, joint_positions):
    """ Send a joint trajectory to the specified robot arm. """
    arm_trajectory = JointTrajectory()
    arm_trajectory.joint_names = [f'{arm_name}_joint{i+1}' for i in range(7)]  # Assumes 7 joints for Franka

    arm_point = JointTrajectoryPoint()
    arm_point.positions = joint_positions
    arm_point.time_from_start = rospy.Duration(1)  # Move over 1 second
    arm_trajectory.points.append(arm_point)

    arm_pub.publish(arm_trajectory)

def generate_z_trajectory(start_z, end_z, num_steps=500):
    """ Generate a smooth trajectory for the z-coordinate using cubic interpolation. """
    return np.linspace(start_z, end_z, num_steps)

def main():
    rospy.init_node('box_and_robot_mover')

    # Publishers for both arms
    left_arm_pub = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
    right_arm_pub = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=10)

    # Create a publisher for RViz markers
    marker_pub = rospy.Publisher('/box_trajectory_marker', Marker, queue_size=10)

    # Get the box's initial state
    initial_pos, initial_orient = get_box_state()

    if initial_pos is None:
        rospy.logerr("Failed to get initial box position.")
        return

    # Define the z-axis trajectory (only z changes, x and y stay the same)
    start_z = initial_pos.z
    end_z = 2  # Final z position
    z_trajectory = generate_z_trajectory(start_z, end_z)

    # Use the same orientation throughout the movement
    orientation = [initial_orient.x, initial_orient.y, initial_orient.z, initial_orient.w]

    # Move the box and synchronize the robot arms
    move_box_and_arms(z_trajectory, orientation, left_arm_pub, right_arm_pub, marker_pub)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
