import rospy
import numpy as np
import csv

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import tf.transformations as tf

# Global variables to store the G matrix and Hand Jacobian
G_matrix = None
Hand_Jacobian = None
current_joint_states = None
box_pose = None
trajectory_data = None

def save_joint_angles_to_csv(file_name, joint_angles):

    with open(file_name, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(joint_angles)  # Write the joint angles as a new row

def grad_manipulability(A, W, delta=1e-6):
    num_joints = A.shape[1]
    grad_W = np.zeros(num_joints)
    W_current = W
    
    # Loop over each joint and compute finite difference gradient
    for i in range(num_joints):
        A_perturbed = A.copy()
        A_perturbed[:, i] += delta 
    
        W_perturbed = np.sqrt(np.linalg.det(A_perturbed @ A_perturbed.T))
        grad_W[i] = (W_perturbed - W_current) / delta
    
    return grad_W

def compute_A(G_matrix, Hand_Jacobian):
    if G_matrix is None or Hand_Jacobian is None:
        rospy.logwarn("G_matrix or Hand_Jacobian not received yet!")
        return None
    # Calculate the transpose of the inverse of G_matrix
    G_inv_transpose = np.linalg.pinv(G_matrix).T
    # Compute A as the product of G_inv_transpose and Hand_Jacobian
    A = np.dot(G_inv_transpose, Hand_Jacobian)
    return A

# Function to compute gradient of manipulability w.r.t. joint angles
def compute_phi_dot_opt(A, optimization_type, gain=0.1):
    
    if optimization_type == "velocity_manipulability":
        W = velocity_manipulability(A)
    elif optimization_type == "force_manipulability":
        W = force_manipulability(A)
    elif optimization_type == "directional_force_manipulability":
        W = directional_force_manipulability(A)
    else:
        raise ValueError("Invalid optimization type")

    grad_W = grad_manipulability(A, W)

    phi_dot_opt = gain * grad_W
    
    return phi_dot_opt

# Function to compute the third term: (I - A_pinv * A) * phi_dot_opt
def compute_null_space_component(A):
    A_pinv = np.linalg.pinv(A)

    I = np.eye(A.shape[1])

    null_space_matrix = I - np.dot(A_pinv, A)

    null_space_component = null_space_matrix

    return null_space_component

def velocity_manipulability(A):
    AA_T = np.dot(A, A.T)
    return np.sqrt(np.linalg.det(AA_T))

def force_manipulability(A):
    AA_T = np.dot(A, A.T)
    AA_T_pinv = np.linalg.pinv(AA_T)  # Pseudo-inverse of AA^T
    return np.sqrt(np.linalg.det(AA_T_pinv))

def directional_force_manipulability(A, F):
    AA_T = np.dot(A, A.T)
    first_term = AA_T / np.trace(AA_T)
    second_term = F / np.trace(F)
    return np.sqrt(np.trace(first_term - second_term))

def q_forward_kinematics(phi):
    global box_pose
    if box_pose is None:
        rospy.logwarn("Box pose not available yet!")
        return np.zeros(6)  # Return zeros until the box pose is available
    return np.array(box_pose)  # Return the current box position and orientation (6 components)

# Callback for the G matrix
def grasp_matrix_callback(msg):
    global G_matrix
    G_matrix = np.array(msg.data).reshape(6, 6)  # Assuming 6x6 Grasp Matrix

# Callback for the Hand Jacobian
def hand_jacobian_callback(msg):
    global Hand_Jacobian
    Hand_Jacobian = np.array(msg.data).reshape(6, 14)  # Assuming 6 rows, 14 columns for 2x 7 DOF robots

# Callback function for joint states
def joint_state_callback(msg):
    global current_joint_states
    current_joint_states = np.array(msg.position)  # Assuming 14 joint states (2 arms with 7 DOF each)

# Callback function for /compliant_box_pose topic
def compliant_box_pose_callback(msg):
    global box_pose
    # The msg contains the position and orientation of the box
    position = [msg.position.x, msg.position.y, msg.position.z]
    orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    
    # Combine position and orientation into a single array of 6 components
    box_pose = position + orientation

# Load your stored trajectory from file (from your previous code)
def load_trajectory(file_name='box_trajectory.csv'):
    data = np.loadtxt(file_name, delimiter=',', skiprows=1)
    positions = data[:, :3]
    orientations = data[:, 3:]
    return positions, orientations

def compute_q_dot_d(current_index, delta_t, positions, orientations):
    if current_index >= len(positions) - 1:
        rospy.logwarn("Reached the end of the trajectory. Using zero velocity.")
        return np.zeros(7)  # Return zero for both position (3) and orientation (4) velocities
    
    # Finite difference to compute position velocity
    q_dot_d_position = (positions[current_index + 1] - positions[current_index]) / delta_t

    # Orientation difference between consecutive quaternions
    q1 = orientations[current_index]  # Current quaternion
    q2 = orientations[current_index + 1]  # Next quaternion

    # Compute the angular velocity using quaternion difference
    # First, compute quaternion difference
    q_diff = tf.quaternion_multiply(tf.quaternion_inverse(q1), q2)

    # Convert the quaternion difference to an angular velocity representation
    angular_velocity = 2 * np.array([q_diff[0], q_diff[1], q_diff[2]]) / delta_t

    # Combine position velocity and angular velocity
    q_dot_d = np.concatenate([q_dot_d_position, angular_velocity])

    return q_dot_d

def quaternion_to_angular_velocity(q_current, q_desired):
    """
    Converts the difference between two quaternions into angular velocity.
    Args:
        q_current: Current quaternion [x, y, z, w].
        q_desired: Desired quaternion [x, y, z, w].
    Returns:
        Angular velocity as a 3-element vector.
    """
    q_diff = tf.quaternion_multiply(tf.quaternion_inverse(q_current), q_desired)
    angular_velocity = 2 * np.array([q_diff[0], q_diff[1], q_diff[2]])  # First 3 elements represent angular velocity
    return angular_velocity

# Main function to compute φ(t + Δt)
def compute_next_phi(A, current_index, q_d, positions, orientations, delta_t=0.001, K_p=10, Gain_opt=10):
    global current_joint_states

    if current_joint_states is None:
        rospy.logwarn("Joint states not available yet!")
        return None

    phi_t = current_joint_states

    # Compute the desired object velocity (q_dot_d) using finite differences
    q_dot_d = compute_q_dot_d(current_index, delta_t, positions, orientations)

    # Compute forward kinematics to get the current object pose (position + orientation)
    q_fwdkinematics = q_forward_kinematics(phi_t)

    # Desired full pose: combine position and orientation
    q_d_full = np.concatenate([q_d, orientations[current_index]])  # Combine position and orientation

    # Compute error e = q_d_full - q_fwdkinematics (comparing both position and orientation)
    e_position = q_d_full[:3] - q_fwdkinematics[:3]  # First 3 elements for position error
    e_orientation = quaternion_to_angular_velocity(q_fwdkinematics[3:], orientations[current_index])

    # Combine position and orientation errors
    e = np.concatenate([e_position, e_orientation])  # Now e is a 6-element vector

    # Compute the pseudo-inverse of A
    A_pinv = np.linalg.pinv(A)

    optimization_type = "velocity_manipulability"

    phi_dot_opt = compute_phi_dot_opt(A, optimization_type)

    # Compute the first two terms: φ(t + Δt) = φ(t) + A†(q_dot_d + K_p * e) Δt
    second_term = np.dot(A_pinv, (q_dot_d + K_p * e))  # Joint velocity

    third_term = np.dot(compute_null_space_component(A), phi_dot_opt)

    phi_next = phi_t + second_term * delta_t + third_term  # Updated joint angles

    return phi_next

def main():
    rospy.init_node('manipulability_control_node')

    # Initialize the subscriber for joint states
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    # Initialize the subscriber for compliant box pose
    rospy.Subscriber('/compliant_box_pose', Pose, compliant_box_pose_callback)

    # Initialize the subscriber for the grasp matrix (assuming it's published as a Float64MultiArray)
    rospy.Subscriber('/grasp_matrix', Float64MultiArray, grasp_matrix_callback)

    # Initialize the subscriber for the hand Jacobian (assuming it's published as a Float64MultiArray)
    rospy.Subscriber('/hand_jacobian', Float64MultiArray, hand_jacobian_callback)

    rospy.loginfo("Waiting for data from topics...")
    
    # Load trajectory (desired positions)
    positions, orientations = load_trajectory('box_trajectory.csv')

    current_index = 0  # Index to keep track of the current point in the trajectory
    delta_t = 0.001  # Time step

    # File name for storing joint angles
    csv_file_name = 'joint_angles.csv'

    # Add headers to the CSV file
    with open(csv_file_name, mode='w', newline='') as file:
        writer = csv.writer(file)
        # Headers: Left_Arm_Joint_1, ..., Left_Arm_Joint_7, Right_Arm_Joint_1, ..., Right_Arm_Joint_7
        writer.writerow([f'Left_Arm_Joint_{i+1}' for i in range(7)] + [f'Right_Arm_Joint_{i+1}' for i in range(7)])

    while not rospy.is_shutdown():
        # Assuming that A and q_d are available
        A = compute_A(G_matrix, Hand_Jacobian)
        if A is not None:
            q_d = positions[current_index]  # The desired position at the current time step
            phi_next = compute_next_phi(A, current_index, q_d, positions, orientations, delta_t=delta_t)
            if phi_next is not None:
                rospy.loginfo(f"Next joint angles: {phi_next}")

                # Save the joint angles to the CSV file
                save_joint_angles_to_csv(csv_file_name, phi_next)
            current_index += 1
            if current_index >= len(positions) - 1:
                rospy.loginfo("Reached the end of the trajectory.")
                break

        rospy.sleep(delta_t)


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
