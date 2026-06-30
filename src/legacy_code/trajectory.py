##### This code makes both the frankas move 
#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def send_trajectory():
    # Initialize the ROS node
    rospy.init_node('send_trajectory_to_dual_arms')

    # Publishers for both arms
    left_arm_pub = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
    right_arm_pub = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=10)

    rospy.sleep(1)  # Give some time for the publishers to connect

    # Define the trajectory for the left arm
    left_arm_trajectory = JointTrajectory()
    left_arm_trajectory.joint_names = ["left_arm_joint1", "left_arm_joint2", "left_arm_joint3", 
                                       "left_arm_joint4", "left_arm_joint5", "left_arm_joint6", 
                                       "left_arm_joint7"]

    # Define the trajectory points (e.g., 5 points)
    left_arm_points = [
    [0.0, -0.5, 0.0, -1.0, 0.0, 1.0, 0.5],
    [0.1, -0.48, 0.05, -0.95, 0.05, 0.95, 0.48],
    [0.2, -0.46, 0.1, -0.9, 0.1, 0.9, 0.46],
    [0.3, -0.44, 0.15, -0.85, 0.15, 0.85, 0.44],
    [0.4, -0.42, 0.2, -0.8, 0.2, 0.8, 0.42],
    [0.5, -0.4, 0.25, -0.75, 0.25, 0.75, 0.4],
    [0.6, -0.38, 0.3, -0.7, 0.3, 0.7, 0.38],
    [0.7, -0.36, 0.35, -0.65, 0.35, 0.65, 0.36],
    [0.8, -0.34, 0.4, -0.6, 0.4, 0.6, 0.34],
    [0.9, -0.32, 0.45, -0.55, 0.45, 0.55, 0.32],
    [1.0, -0.3, 0.5, -0.5, 0.5, 0.5, 0.3],
    [1.1, -0.28, 0.55, -0.45, 0.55, 0.45, 0.28],
    [1.2, -0.26, 0.6, -0.4, 0.6, 0.4, 0.26],
    [1.3, -0.24, 0.65, -0.35, 0.65, 0.35, 0.24],
    [1.4, -0.22, 0.7, -0.3, 0.7, 0.3, 0.22],
    [1.5, -0.2, 0.75, -0.25, 0.75, 0.25, 0.2],
    [1.6, -0.18, 0.8, -0.2, 0.8, 0.2, 0.18],
    [1.7, -0.16, 0.85, -0.15, 0.85, 0.15, 0.16],
    [1.8, -0.14, 0.9, -0.1, 0.9, 0.1, 0.14],
    [1.9, -0.12, 0.95, -0.05, 0.95, 0.05, 0.12]
]


    # Populate the trajectory message
    for i, positions in enumerate(left_arm_points):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(2 * (i + 1))
        left_arm_trajectory.points.append(point)

    # Define the trajectory for the right arm
    right_arm_trajectory = JointTrajectory()
    right_arm_trajectory.joint_names = ["right_arm_joint1", "right_arm_joint2", "right_arm_joint3", 
                                        "right_arm_joint4", "right_arm_joint5", "right_arm_joint6", 
                                        "right_arm_joint7"]

    # Define the trajectory points (e.g., 5 points)
    right_arm_points = [
    [-0.0, 1.5, -2.0, 1.0, -0.0, -1.0, -0.5],
    [-0.1, 1.48, -2.05, 0.95, -0.05, -0.95, -0.48],
    [-0.2, 1.46, -2.1, 0.9, -0.1, -0.9, -0.46],
    [-0.3, 1.44, -2.15, 0.85, -0.15, -0.85, -0.44],
    [-0.4, 1.42, -2.2, 0.8, -0.2, -0.8, -0.42],
    [-0.5, 1.4, -2.25, 0.75, -0.25, -0.75, -0.4],
    [-0.6, 1.38, -2.3, 0.7, -0.3, -0.7, -0.38],
    [-0.7, 1.36, -2.35, 0.65, -0.35, -0.65, -0.36],
    [-0.8, 1.34, -2.4, 0.6, -0.4, -0.6, -0.34],
    [-0.9, 1.32, -2.45, 0.55, -0.45, -0.55, -0.32],
    [-1.0, 1.3, -2.5, 0.5, -0.5, -0.5, -0.3],
    [-1.1, 1.28, -2.55, 0.45, -0.55, -0.45, -0.28],
    [-1.2, 1.26, -2.6, 0.4, -0.6, -0.4, -0.26],
    [-1.3, 1.24, -2.65, 0.35, -0.65, -0.35, -0.24],
    [-1.4, 1.22, -2.7, 0.3, -0.7, -0.3, -0.22],
    [-1.5, 1.2, -2.75, 0.25, -0.75, -0.25, -0.2],
    [-1.6, 1.18, -2.8, 0.2, -0.8, -0.2, -0.18],
    [-1.7, 1.16, -2.85, 0.15, -0.85, -0.15, -0.16],
    [-1.8, 1.14, -2.9, 0.1, -0.9, -0.1, -0.14],
    [-1.9, 1.12, -2.95, 0.05, -0.95, -0.05, -0.12]
]


    # Populate the trajectory message
    for i, positions in enumerate(right_arm_points):
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = rospy.Duration(2 * (i + 1))
        right_arm_trajectory.points.append(point)

    # Publish the trajectories
    left_arm_pub.publish(left_arm_trajectory)
    right_arm_pub.publish(right_arm_trajectory)

    rospy.loginfo("Trajectories sent to both arms.")

if __name__ == '__main__':
    try:
        send_trajectory()
    except rospy.ROSInterruptException:
        pass