#!/usr/bin/env python3

import rospy
from gazebo_msgs.srv import GetModelState, SetModelState
from gazebo_msgs.msg import ModelState
import numpy as np
import tf.transformations as tf
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from scipy.interpolate import CubicSpline
import csv
import os

def get_box_state():
    rospy.wait_for_service('/gazebo/get_model_state')
    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        box_state = get_model_state('compliant_box', 'world')  # 'world' frame
        
        position = box_state.pose.position
        orientation = box_state.pose.orientation
        
        rospy.loginfo(f"Box Position: x={position.x}, y={position.y}, z={position.z}")
        rospy.loginfo(f"Box Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
        
        return position, orientation
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return None, None

def slerp_quaternion(start_quat, end_quat, t, t_final):
    """SLERP (Spherical Linear Interpolation) for smooth quaternion transitions."""
    tau = t / t_final
    return tf.quaternion_slerp(start_quat, end_quat, tau)

def generate_position_spline_trajectory(points, total_time, time_step):
    """Generate a smooth position trajectory using cubic spline interpolation."""
    num_points = len(points)
    t_points = np.linspace(0, total_time, num_points)

    # Separate x, y, z components
    x_points = [p[0] for p in points]
    y_points = [p[1] for p in points]
    z_points = [p[2] for p in points]

    # Create cubic splines for each component
    x_spline = CubicSpline(t_points, x_points)
    y_spline = CubicSpline(t_points, y_points)
    z_spline = CubicSpline(t_points, z_points)

    # Create the time steps
    t_values = np.arange(0, total_time, time_step)
    
    # Generate the smooth trajectory
    position_trajectory = []
    for t in t_values:
        position = [x_spline(t), y_spline(t), z_spline(t)]
        position_trajectory.append(position)

    return position_trajectory, t_values

def generate_orientation_trajectory(orientations, t_values, total_time):
    """Generate smooth orientation transitions using SLERP."""
    orientation_trajectory = []
    num_points = len(orientations)

    # Generate orientation trajectory by SLERP for each segment
    for i in range(num_points - 1):
        start_orient = orientations[i]
        end_orient = orientations[i + 1]
        segment_time = total_time / (num_points - 1)

        for t in t_values[t_values <= segment_time]:
            interpolated_orientation = slerp_quaternion(start_orient, end_orient, t, segment_time)
            orientation_trajectory.append(interpolated_orientation)

    # Ensure the orientation trajectory has the same number of points as the position trajectory
    while len(orientation_trajectory) < len(t_values):
        orientation_trajectory.append(orientations[-1])

    return orientation_trajectory

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

def save_trajectory_to_csv(position, orientation, file_name='box_trajectory.csv'):
    """Save the position and orientation to a CSV file immediately after calculation."""
    # Overwrite the file initially, append after that
    with open(file_name, mode='a', newline='') as file:
        writer = csv.writer(file)
        row = position + orientation.tolist()  # Convert orientation to a list
        writer.writerow(row)

def initialize_csv_file(file_name='box_trajectory.csv'):
    """Initialize the CSV file by overwriting the old one and writing the header."""
    with open(file_name, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Position_X', 'Position_Y', 'Position_Z', 'Orientation_X', 'Orientation_Y', 'Orientation_Z', 'Orientation_W'])

def move_box_along_trajectory(position_trajectory, orientation_trajectory, marker_pub, time_step):
    """Move the box along the calculated position and orientation trajectory, and publish markers for RViz."""
    
    marker = Marker()
    marker.header.frame_id = "world"
    marker.type = marker.LINE_STRIP
    marker.action = marker.ADD

    # Set marker properties
    marker.scale.x = 0.01  # Line thickness
    marker.color.a = 1.0   # Opacity
    marker.color.r = 0.0   # Red
    marker.color.g = 1.0   # Green
    marker.color.b = 0.0   # Blue

    for i in range(len(position_trajectory)):
        set_box_state(position_trajectory[i], orientation_trajectory[i])

        # Add points to the marker for visualization in RViz
        point = Point()
        point.x = position_trajectory[i][0]
        point.y = position_trajectory[i][1]
        point.z = position_trajectory[i][2]
        marker.points.append(point)

        # Publish the marker
        marker_pub.publish(marker)

        # Save the current position and orientation to CSV
        save_trajectory_to_csv(position_trajectory[i], orientation_trajectory[i])

        rospy.sleep(time_step)  # Use the unified time step here

def move_box_to_target():
    """Main function to control the movement of the box along the trajectory and publish RViz markers."""
    # Initialize the CSV file by overwriting the old one
    initialize_csv_file()

    # Get the initial position and orientation of the box
    initial_pos, initial_orient = get_box_state()

    if initial_pos is None:
        rospy.logerr("Failed to get initial position and orientation of the box.")
        return

    # Hardcoded positions and orientations (modifiable)
    initial_pos_list = [initial_pos.x, initial_pos.y, initial_pos.z]
    point_1 = [initial_pos.x, initial_pos.y, 1.5]  # Intermediate point 1
    target_position = [0.2960, -0.2958, 1.1]  # Target position

    # Quaternions for the corresponding positions (modifiable orientations)
    initial_orient_list = [initial_orient.x, initial_orient.y, initial_orient.z, initial_orient.w]
    orient_1 = [initial_orient.x, initial_orient.y, initial_orient.z, initial_orient.w]  # Intermediate orientation
    target_orientation = [0, 0, 0, 1]
    #target_orientation = [0, 0, -0.5157, 0.7157]  # Target orientation

    # All points and orientations in sequence (initial -> point 1 -> target)
    points = [initial_pos_list, point_1, target_position]
    orientations = [initial_orient_list, orient_1, target_orientation]

    # Parameters to control speed and accuracy
    total_time = 100.0  # Total time for the movement
    time_step = 0.01  # Unified time step for both position and orientation

    # Generate a smooth position trajectory using cubic spline interpolation
    position_trajectory, t_values = generate_position_spline_trajectory(points, total_time, time_step)

    # Generate orientation trajectory using SLERP
    orientation_trajectory = generate_orientation_trajectory(orientations, t_values, total_time)

    # Create a publisher for RViz markers
    marker_pub = rospy.Publisher('/box_trajectory_marker', Marker, queue_size=10)

    # Move the box along the generated trajectory and visualize the trajectory in RViz
    move_box_along_trajectory(position_trajectory, orientation_trajectory, marker_pub, time_step)

if __name__ == '__main__':
    try:
        # Initialize the ROS node
        rospy.init_node('box_trajectory_mover')

        # Move the box along the trajectory and publish the trajectory in RViz
        move_box_to_target()

    except rospy.ROSInterruptException:
        pass
