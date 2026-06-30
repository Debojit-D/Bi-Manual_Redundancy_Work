#!/usr/bin/env python3

import rospy
import numpy as np
import tf.transformations as tf_trans
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

# Global variables
object_orientation = np.eye(3)  # Initialize as identity matrix

# Contact basis vectors defined in the **object frame**
Wpk_of_l = np.array([
    [0, 0, -1],  # x-axis in object frame
    [-1, 0, 0],  # y-axis in object frame
    [0, 1, 0]    # z-axis in object frame
])

Wpk_of_r = np.array([
    [0, 0, 1],   # x-axis in object frame
    [1, 0, 0],   # y-axis in object frame
    [0, 1, 0]    # z-axis in object frame
])


def table_pose_callback(msg):
    """
    Callback function to update the object's orientation from the /table_pose topic.
    Converts quaternion to a rotation matrix and prints it.
    """
    global object_orientation

    # Extract quaternion from Pose message
    quat = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]

    # Convert quaternion to 4x4 transformation matrix
    transformation_matrix = tf_trans.quaternion_matrix(quat)

    # Extract the **3x3 rotation matrix**
    object_orientation = transformation_matrix[:3, :3]  # Rotation of object w.r.t world

    #rospy.loginfo("Updated Object Rotation Matrix (World ← Object):\n{}".format(object_orientation))

    # Publish updated contact frame basis in world frame
    publish_contact_frame_basis()


def publish_contact_frame_basis():
    """
    Publish the contact frame basis vectors in the world frame.
    Uses the rotation matrix to transform contact basis vectors from object frame to world frame.
    """
    left_basis_msg = Float64MultiArray()
    right_basis_msg = Float64MultiArray()

    # Pre-multiply rotation matrix (World ← Object) to contact basis (Object Frame)
    Wpk_wf_l = np.dot(object_orientation, Wpk_of_l)  # Left contact frame in world frame
    Wpk_wf_r = np.dot(object_orientation, Wpk_of_r)  # Right contact frame in world frame

    # Flatten the matrices and assign to the message data
    left_basis_msg.data = Wpk_wf_l.flatten()
    right_basis_msg.data = Wpk_wf_r.flatten()

    # Publish the transformed contact frames
    left_contact_basis_pub.publish(left_basis_msg)
    right_contact_basis_pub.publish(right_basis_msg)

    #rospy.loginfo("Published Transformed Contact Frames in World Frame.")


def ros_node_setup():
    global left_contact_basis_pub, right_contact_basis_pub

    rospy.init_node('contact_frame_publisher', anonymous=True)

    # Publishers for contact frame basis vectors
    left_contact_basis_pub = rospy.Publisher("/left_contact_frame_basis", Float64MultiArray, queue_size=10)
    right_contact_basis_pub = rospy.Publisher("/right_contact_frame_basis", Float64MultiArray, queue_size=10)

    # Subscribe to the object pose topic
    rospy.Subscriber("/table_pose", Pose, table_pose_callback)

    rospy.loginfo("Contact Frame Publisher Node Initialized.")
    
    rospy.spin()  # Keep the node running


if __name__ == '__main__':
    try:
        ros_node_setup()
    except rospy.ROSInterruptException:
        pass
