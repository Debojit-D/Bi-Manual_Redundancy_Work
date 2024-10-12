#!/usr/bin/env python3

"""
This ROS node publishes both the end-effector poses and the Jacobian matrices
for the left and right manipulators of a robot.
"""

import rospy
import tf
import tf2_ros

import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray 
from moveit_commander import RobotCommander

class ManipulatorStatePublisher:

    def __init__(self):
        rospy.init_node('manipulator_state_publisher', anonymous=True, log_level=rospy.WARN)

        # Initialize TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Logging to check initialization process
        rospy.loginfo("Initializing ManipulatorStatePublisher")
        
        # Subscribers to joint states
        rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)
        
        # Publishers for end-effector poses.
        self.left_ee_pub = rospy.Publisher("/left_end_effector_pose", PoseStamped, queue_size=10)
        self.right_ee_pub = rospy.Publisher("/right_end_effector_pose", PoseStamped, queue_size=10)
        
        # Publishers for manipulator jacobian matrices.
        self.left_jaco_pub = rospy.Publisher("/left_manipulator_jacobian", Float64MultiArray, queue_size=25)
        self.right_jaco_pub = rospy.Publisher("/right_manipulator_jacobian", Float64MultiArray, queue_size=25)
        
        # Robot Commander for kinematics
        self.robot = RobotCommander()
        
        # TF2 listener
        self.tf_buffer = tf2_ros.Buffer()
        rospy.loginfo("TF buffer initialized")
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo("TF listener initialized")

    
    def joint_states_callback(self, joint_states):
        # Get and publish the pose of the left and right end-effectors
        self.publish_end_effector_poses()

        # Calculate and publish the Jacobians for the left and right manipulators
        self.publish_jacobians()

    def publish_end_effector_poses(self):
        try:
            # Get the pose of the left and right end-effectors using TF
            while not self.tf_buffer.can_transform("world", "left_arm_link7", rospy.Time(), rospy.Duration(1.0)):
                rospy.logwarn("Waiting for TF transform between 'world' and 'left_arm_link7'")
                rospy.sleep(1.0)

             # Wait for the right arm transform to be available
            while not self.tf_buffer.can_transform("world", "right_arm_link7", rospy.Time(), rospy.Duration(1.0)):
                rospy.logwarn("Waiting for TF transform between 'world' and 'right_arm_link7'")
                rospy.sleep(1.0)

            left_transform = self.tf_buffer.lookup_transform("world", "left_arm_link7", rospy.Time())
            right_transform = self.tf_buffer.lookup_transform("world", "right_arm_link7", rospy.Time())
            
            # Create PoseStamped messages for left and right end-effectors
            left_pose = PoseStamped()
            left_pose.header.stamp = rospy.Time.now()
            left_pose.header.frame_id = "world"
            left_pose.pose.position.x = left_transform.transform.translation.x
            left_pose.pose.position.y = left_transform.transform.translation.y
            left_pose.pose.position.z = left_transform.transform.translation.z
            left_pose.pose.orientation = left_transform.transform.rotation

            right_pose = PoseStamped()
            right_pose.header.stamp = rospy.Time.now()
            right_pose.header.frame_id = "world"
            right_pose.pose.position.x = right_transform.transform.translation.x
            right_pose.pose.position.y = right_transform.transform.translation.y
            right_pose.pose.position.z = right_transform.transform.translation.z
            right_pose.pose.orientation = right_transform.transform.rotation

            # Publish the end-effector poses
            self.left_ee_pub.publish(left_pose)
            self.right_ee_pub.publish(right_pose)
            
            rospy.loginfo("Published left and right end-effector poses.")
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("TF lookup failed: {}".format(e))

    def publish_jacobians(self):
        # Calculate Jacobians
        left_jacobian = self.calculate_jacobian("left_panda_arm")
        right_jacobian = self.calculate_jacobian("right_panda_arm")

        # Convert Jacobians to Float64MultiArray messages
        left_jacobian_msg = Float64MultiArray(data=left_jacobian.flatten().tolist())
        right_jacobian_msg = Float64MultiArray(data=right_jacobian.flatten().tolist())

        # Publish the Jacobians
        self.left_jaco_pub.publish(left_jacobian_msg)
        self.right_jaco_pub.publish(right_jacobian_msg)

        rospy.loginfo("Published left and right manipulator Jacobians.")

    def calculate_jacobian(self, group_name):
        """
        Calculate the Jacobian matrix for a given arm group.
        
        Parameters:
        - group_name: The name of the MoveIt planning group (e.g., "left_arm" or "right_arm").
        
        Returns:
        - jacobian: The Jacobian matrix as a numpy array.
        """
        # Get the robot group for the arm
        group = self.robot.get_group(group_name)
        
        # Get the joint values
        joint_values = group.get_current_joint_values()
        
        # Calculate the Jacobian matrix
        jacobian = group.get_jacobian_matrix(joint_values)
        
        return np.array(jacobian)

    def run(self):
        rate = rospy.Rate(100)  # Set the rate to 100 Hz
        while not rospy.is_shutdown():
        # Call the joint_states_callback function at 100 Hz
            self.joint_states_callback(None)  # You can pass None since you're manually calling it
            rate.sleep()

if __name__ == '__main__':
    try:
        state_publisher = ManipulatorStatePublisher()
        state_publisher.run()
    except rospy.ROSInterruptException:
        pass
