#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tf_trans
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

# Global variables to store the Jacobian matrices
left_jacobian = None
right_jacobian = None
object_position = None
object_orientation = None

si_l = None
ti_l = None
ni_l = None

si_r = None
ti_r = None
ni_r = None

# Hardcoded position vectors for simplicity
bi_l = np.array([0.1, 0.0, 0.0])  # Example position vector for the left contact point
bi_r = np.array([-0.1, 0.0, 0.0])  # Example position vector for the right contact point

def left_contact_basis_callback(msg):
    global si_l, ti_l, ni_l
    basis_matrix = np.array(msg.data).reshape(3, 3)
    si_l, ti_l, ni_l = basis_matrix[:, 0], basis_matrix[:, 1], basis_matrix[:, 2]

def right_contact_basis_callback(msg):
    global si_r, ti_r, ni_r
    basis_matrix = np.array(msg.data).reshape(3, 3)
    si_r, ti_r, ni_r = basis_matrix[:, 0], basis_matrix[:, 1], basis_matrix[:, 2]

def left_jacobian_callback(msg):
    global left_jacobian
    left_jacobian = np.array(msg.data).reshape(6, 7)

def right_jacobian_callback(msg):
    global right_jacobian
    right_jacobian = np.array(msg.data).reshape(6, 7)

def object_pose_callback(msg):
    global object_position, object_orientation
    object_position = np.array([msg.position.x, msg.position.y, msg.position.z])
    quaternion = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    euler_angles = quaternion_to_euler(quaternion)
    object_orientation = euler_to_rotation_matrix(euler_angles)

def quaternion_to_euler(quaternion):
    """Converts a quaternion into Euler angles (roll, pitch, yaw)."""
    return tf_trans.euler_from_quaternion(quaternion)

def euler_to_rotation_matrix(euler_angles):
    """Converts Euler angles (roll, pitch, yaw) into a rotation matrix."""
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
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    # Combined rotation matrix
    return np.dot(R_z, np.dot(R_y, R_x))

def calculate_rotation_matrix_from_object_orientation():
    """Returns the rotation matrix from the object's orientation."""
    global object_orientation
    return object_orientation if object_orientation is not None else np.eye(3)  # Return identity matrix if orientation is not available

def grasp_sub_matrix_calculator(rotation_matrix_input, si, ti, ni, bi):
    """
    Calculate the grasp sub-matrix G_i corresponding to a single contact point.
    """
    G_i = np.zeros((6, 3))
    G_i[0:3, 0] = si
    G_i[0:3, 1] = ti
    G_i[0:3, 2] = ni
    G_i[3:, 0] = np.cross(np.dot(rotation_matrix_input, bi), si)
    G_i[3:, 1] = np.cross(np.dot(rotation_matrix_input, bi), ti)
    G_i[3:, 2] = np.cross(np.dot(rotation_matrix_input, bi), ni)
    return G_i

def hand_jacobian_calculator(Wpki_list, Rpki_list, manipulator_half_jacobian_list):
    """
    Calculate the hand Jacobian matrix.
    """
    num_fingers = len(manipulator_half_jacobian_list)
    dof = manipulator_half_jacobian_list[0].shape[1]  
    Jh = np.zeros((3 * num_fingers, dof * num_fingers))
    
    for i in range(num_fingers):
        Wpki = Wpki_list[i] 
        Rpki = Rpki_list[i] 
        Ji = manipulator_half_jacobian_list[i]
        Jh[3 * i:3 * i + 3, dof * i:dof * i + dof] = np.dot(Wpki, np.dot(Rpki, Ji))
    return Jh

def publish_grasp_matrix():
    """Calculate and publish the full grasp matrix."""
    if any(x is None for x in [si_l, ti_l, ni_l, si_r, ti_r, ni_r, object_orientation]):
        return

    rotation_matrix_input = calculate_rotation_matrix_from_object_orientation()
    G_left = grasp_sub_matrix_calculator(rotation_matrix_input, si_l, ti_l, ni_l, bi_l)
    G_right = grasp_sub_matrix_calculator(rotation_matrix_input, si_r, ti_r, ni_r, bi_r)
    G = np.hstack((G_left, G_right))

    # rospy.loginfo(f"Grasp Matrix:\n{G}")
    grasp_matrix_msg = Float64MultiArray(data=G.flatten())
    grasp_matrix_pub.publish(grasp_matrix_msg)

def publish_hand_jacobian():
    """Calculate and publish the hand Jacobian matrix."""
    if any(x is None for x in [left_jacobian, right_jacobian, si_l, ti_l, ni_l, si_r, ti_r, ni_r, object_orientation]):
        return

    Wpki_left = np.column_stack((si_l, ti_l, ni_l))
    Wpki_right = np.column_stack((si_r, ti_r, ni_r))
    Rpki_left = np.eye(3)
    Rpki_right = np.eye(3)
    left_half_jacobian = left_jacobian[:3, :] 
    right_half_jacobian = right_jacobian[:3, :] 

    Jh = hand_jacobian_calculator([Wpki_left, Wpki_right], [Rpki_left, Rpki_right], [left_half_jacobian, right_half_jacobian])

    # rospy.loginfo(f"Hand Jacobian:\n{Jh}")
    hand_jacobian_msg = Float64MultiArray(data=Jh.flatten())
    hand_jacobian_pub.publish(hand_jacobian_msg)

def ros_node_setup():
    global grasp_matrix_pub, hand_jacobian_pub

    rospy.init_node('grasp_matrix_hand_jacobian_publisher', anonymous=True)

    grasp_matrix_pub = rospy.Publisher("/grasp_matrix", Float64MultiArray, queue_size=20)
    hand_jacobian_pub = rospy.Publisher("/hand_jacobian", Float64MultiArray, queue_size=20)

    rospy.Subscriber("/left_manipulator_jacobian", Float64MultiArray, left_jacobian_callback)
    rospy.Subscriber("/right_manipulator_jacobian", Float64MultiArray, right_jacobian_callback)
    rospy.Subscriber("/left_contact_frame_basis", Float64MultiArray, left_contact_basis_callback)
    rospy.Subscriber("/right_contact_frame_basis", Float64MultiArray, right_contact_basis_callback)
    rospy.Subscriber("/compliant_box_pose", Pose, object_pose_callback)

    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        publish_grasp_matrix()
        publish_hand_jacobian()
        rate.sleep()

if __name__ == '__main__':
    try:
        ros_node_setup()
    except rospy.ROSInterruptException:
        pass
