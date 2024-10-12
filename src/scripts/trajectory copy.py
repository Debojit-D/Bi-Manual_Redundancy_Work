#!/usr/bin/env python

import rospy
import pandas as pd
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def load_trajectory_from_csv(file_path):
    """
    Load trajectory data from a CSV file.
    
    Args:
        file_path (str): Path to the CSV file.
    
    Returns:
        tuple: Two lists containing the trajectories for the left and right arms.
    """
    # Load the CSV file
    data = pd.read_csv(file_path)
    
    # Filter out rows containing NaN values
    data = data.dropna()

    # Separate the data for left and right arms using correct column names
    left_arm_trajectory = data[['Left_Arm_Joint_1', 'Left_Arm_Joint_2', 'Left_Arm_Joint_3', 
                                'Left_Arm_Joint_4', 'Left_Arm_Joint_5', 'Left_Arm_Joint_6', 
                                'Left_Arm_Joint_7']].values

    right_arm_trajectory = data[['Right_Arm_Joint_1', 'Right_Arm_Joint_2', 'Right_Arm_Joint_3', 
                                 'Right_Arm_Joint_4', 'Right_Arm_Joint_5', 'Right_Arm_Joint_6', 
                                 'Right_Arm_Joint_7']].values
    
    return left_arm_trajectory, right_arm_trajectory

def send_trajectory_from_csv(csv_file_path):
    # Initialize the ROS node
    rospy.init_node('send_trajectory_from_csv')

    # Publishers for both arms
    left_arm_pub = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
    right_arm_pub = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=10)

    rospy.sleep(1)  # Give some time for the publishers to connect

    # Load the trajectory data from the CSV file
    left_arm_points, right_arm_points = load_trajectory_from_csv(csv_file_path)

    # Define the trajectory for the left arm
    left_arm_trajectory = JointTrajectory()
    left_arm_trajectory.joint_names = ["Left_Arm_Joint_1", "Left_Arm_Joint_2", "Left_Arm_Joint_3", 
                                       "Left_Arm_Joint_4", "Left_Arm_Joint_5", "Left_Arm_Joint_6", 
                                       "Left_Arm_Joint_7"]

    # Populate the trajectory message for the left arm
    for i, positions in enumerate(left_arm_points):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(2 * (i + 1))  # Set time for each point
        left_arm_trajectory.points.append(point)

    # Define the trajectory for the right arm
    right_arm_trajectory = JointTrajectory()
    right_arm_trajectory.joint_names = ["Right_Arm_Joint_1", "Right_Arm_Joint_2", "Right_Arm_Joint_3", 
                                        "Right_Arm_Joint_4", "Right_Arm_Joint_5", "Right_Arm_Joint_6", 
                                        "Right_Arm_Joint_7"]

    # Populate the trajectory message for the right arm
    for i, positions in enumerate(right_arm_points):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(2 * (i + 1))  # Set time for each point
        right_arm_trajectory.points.append(point)

    # Publish the trajectories
    left_arm_pub.publish(left_arm_trajectory)
    right_arm_pub.publish(right_arm_trajectory)

    rospy.loginfo("Trajectories sent to both arms from CSV.")

if __name__ == '__main__':
    try:
        # Path to your CSV file containing the trajectory
        csv_file_path = 'joint_angles.csv'
        
        send_trajectory_from_csv(csv_file_path)
    except rospy.ROSInterruptException:
        pass
