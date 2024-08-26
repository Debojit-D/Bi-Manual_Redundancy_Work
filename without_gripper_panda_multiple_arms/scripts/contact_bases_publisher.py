#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tf_trans
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

# Global variables to store the object orientation
object_orientation = None

# Initial contact frame vectors in the object frame (defined locally)
initial_si_l = np.array([1, 0, 0])
initial_ti_l = np.array([0, 0, 1])
initial_ni_l = np.array([0, -1, 0])

initial_si_r = np.array([0, 0, 1])
initial_ti_r = np.array([1, 0, 0])
initial_ni_r = np.array([0, 1, 0])

def object_pose_callback(msg):
    global object_orientation
    quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    
    # Normalize the quaternion
    norm = np.linalg.norm(quaternion)
    quaternion = [q / norm for q in quaternion]
    
    # Convert quaternion to Euler angles
    euler_angles = quaternion_to_euler(quaternion)
    
    # Ensure the angles are scalars
    roll, pitch, yaw = map(float, euler_angles)
    
    # Convert Euler angles to rotation matrix
    object_orientation = euler_to_rotation_matrix([roll, pitch, yaw])
    
    # Publish the updated contact frame basis vectors
    publish_contact_frame_basis()

def quaternion_to_euler(quaternion):
    """
    Converts a normalized quaternion into Euler angles (roll, pitch, yaw).
    """
    roll, pitch, yaw = tf_trans.euler_from_quaternion(quaternion)
    return [roll, pitch, yaw]

def euler_to_rotation_matrix(euler_angles):
    """
    Converts Euler angles (roll, pitch, yaw) into a rotation matrix.
    """
    roll, pitch, yaw = map(float, euler_angles)  # Ensure angles are scalars

    # Rotation matrix for roll (rotation around x-axis)
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    # Rotation matrix for pitch (rotation around y-axis)
    R_y = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    # Rotation matrix for yaw (rotation around z-axis)
    R_z = np.array([
        [float(np.cos(yaw)), float(-np.sin(yaw)), 0],
        [float(np.sin(yaw)), float(np.cos(yaw)), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix
    R = np.dot(R_z, np.dot(R_y, R_x))

    return R

def publish_contact_frame_basis():
    """
    Publish the contact frame basis vectors for the left and right contacts.
    """
    left_basis_msg = Float64MultiArray()
    right_basis_msg = Float64MultiArray()

    # Transform initial basis vectors using the object's current orientation
    si_l = np.dot(object_orientation, initial_si_l)
    ti_l = np.dot(object_orientation, initial_ti_l)
    ni_l = np.dot(object_orientation, initial_ni_l)

    si_r = np.dot(object_orientation, initial_si_r)
    ti_r = np.dot(object_orientation, initial_ti_r)
    ni_r = np.dot(object_orientation, initial_ni_r)

    # Combine into 3x3 matrices for left and right contact frames
    left_contact_frame_matrix = np.column_stack((si_l, ti_l, ni_l))
    right_contact_frame_matrix = np.column_stack((si_r, ti_r, ni_r))

    # Flatten the matrices and assign to the message data
    left_basis_msg.data = left_contact_frame_matrix.flatten()
    right_basis_msg.data = right_contact_frame_matrix.flatten()

    # Publish the messages
    left_contact_basis_pub.publish(left_basis_msg)
    right_contact_basis_pub.publish(right_basis_msg)

def ros_node_setup():
    global left_contact_basis_pub, right_contact_basis_pub
    
    rospy.init_node('contact_frame_publisher', anonymous=True)

    # Publishers for contact frame basis vectors
    left_contact_basis_pub = rospy.Publisher("/left_contact_frame_basis", Float64MultiArray, queue_size=10)
    right_contact_basis_pub = rospy.Publisher("/right_contact_frame_basis", Float64MultiArray, queue_size=10)

    # Subscribe to the object pose topic
    rospy.Subscriber("/compliant_box_pose", Pose, object_pose_callback)

    # Keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        ros_node_setup()
    except rospy.ROSInterruptException:
        pass
