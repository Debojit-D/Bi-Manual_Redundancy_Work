#!/usr/bin/env python

import rospy
import csv
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def read_csv_and_send_trajectory(file_name='joint_angles.csv', point_delay=0.001):
    # Initialize the ROS node
    rospy.init_node('send_trajectory_from_csv')

    # Publishers for both arms
    left_arm_pub = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
    right_arm_pub = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=10)

    rospy.sleep(1)  # Give some time for the publishers to connect

    # Initialize the trajectory messages
    left_arm_trajectory = JointTrajectory()
    right_arm_trajectory = JointTrajectory()

    left_arm_trajectory.joint_names = ["left_arm_joint1", "left_arm_joint2", "left_arm_joint3", 
                                       "left_arm_joint4", "left_arm_joint5", "left_arm_joint6", 
                                       "left_arm_joint7"]
    
    right_arm_trajectory.joint_names = ["right_arm_joint1", "right_arm_joint2", "right_arm_joint3", 
                                        "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", 
                                        "right_arm_joint7"]

    # Read the CSV file
    with open(file_name, 'r') as file:
        reader = csv.reader(file)
        next(reader)  # Skip the header

        # Initialize the time variable
        time_from_start = rospy.Duration(0)
        row_number = 1  # Keep track of the row number for logging

        for row in reader:
            # Convert the row to a numpy array and check for NaN values
            try:
                row_values = np.array([float(x) if x.lower() != 'nan' else np.nan for x in row])
            except ValueError:
                rospy.logwarn(f"Skipping row {row_number} due to invalid data.")
                row_number += 1
                continue

            if np.isnan(row_values).any():
                rospy.logwarn(f"Skipping row {row_number} due to NaN values")
                row_number += 1
                continue  # Skip rows with NaN values

            # Separate the left and right arm joint angles
            left_arm_angles = row_values[:7]
            right_arm_angles = row_values[7:]

            # Create JointTrajectoryPoint for left arm
            left_point = JointTrajectoryPoint()
            left_point.positions = left_arm_angles
            left_point.time_from_start = time_from_start

            # Create JointTrajectoryPoint for right arm
            right_point = JointTrajectoryPoint()
            right_point.positions = right_arm_angles
            right_point.time_from_start = time_from_start

            # Publish the point for both arms
            left_arm_trajectory.points = [left_point]
            right_arm_trajectory.points = [right_point]

            left_arm_pub.publish(left_arm_trajectory)
            right_arm_pub.publish(right_arm_trajectory)

            # Log the published point
            rospy.loginfo(f"Published trajectory point {row_number} with {point_delay} seconds delay")

            # Increment the time for the next point by `point_delay` seconds
            time_from_start += rospy.Duration(point_delay)
            row_number += 1

            # Add a delay between each published point for smooth real-time execution
            rospy.sleep(point_delay)

    rospy.loginfo(f"Finished sending trajectory from {file_name}")

if __name__ == '__main__':
    try:
        read_csv_and_send_trajectory('joint_angles.csv', point_delay=0.001)  # Adjust point_delay as needed
    except rospy.ROSInterruptException:
        pass
