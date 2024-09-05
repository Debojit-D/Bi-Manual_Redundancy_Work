import rospy
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose

# Global variables to store the G matrix and Hand Jacobian
G_matrix = None
Hand_Jacobian = None
current_joint_states = None
box_pose = None
trajectory_data = None 

# Function to compute matrix A from the G matrix and Hand Jacobian
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
def compute_phi_dot_opt(A, optimization_type):
    if optimization_type=="velocity_manipulability":
        W = velocity_manipulability(A)
    elif optimization_type=="force_manipulability":
        W = force_manipulability(A)
    elif optimization_type=="directional_force_manipulability":
        W = directional_force_manipulability(A)

# Function to compute the third term: (I - A_pinv * A) * phi_dot_opt
def compute_null_space_component(A):
    # Compute the pseudo-inverse of A
    A_pinv = np.linalg.pinv(A)

    # Identity matrix of appropriate size
    I = np.eye(A.shape[1])

    # Null-space projection matrix (I - A_pinv * A)
    null_space_matrix = I - np.dot(A_pinv, A)

    # Compute the null-space component
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

# Function to compute q_dot_d using finite differences
def compute_q_dot_d(current_index, delta_t, positions):
    if current_index >= len(positions) - 1:
        rospy.logwarn("Reached the end of the trajectory. Using zero velocity.")
        return np.zeros(3)
    
    # Finite difference to compute velocity
    q_dot_d = (positions[current_index + 1] - positions[current_index]) / delta_t
    return q_dot_d

# Main function to compute φ(t + Δt)
def compute_next_phi(A, current_index, q_d, positions, delta_t=0.001, K_p=10, Gain_opt=10):
    global current_joint_states

    if current_joint_states is None:
        rospy.logwarn("Joint states not available yet!")
        return None

    phi_t = current_joint_states

    # Compute the desired object velocity (q_dot_d) using finite differences
    q_dot_d = compute_q_dot_d(current_index, delta_t, positions)

    # Compute forward kinematics to get the current object position
    q_fwdkinematics = q_forward_kinematics(phi_t)

    # Compute error e = q_d - q_fwdkinematics
    e = q_d - q_fwdkinematics

    # Compute the pseudo-inverse of A
    A_pinv = np.linalg.pinv(A)

    optimization_type = "velocity_manipulability"

    phi_dot_opt = compute_phi_dot_opt(A,optimization_type)

    # Compute the first two terms: φ(t + Δt) = φ(t) + A†(q_dot_d + K_p * e) Δt
    second_term = np.dot(A_pinv, (q_dot_d + K_p * e))  # Joint velocity

    third_term = np.dot(compute_null_space_component(A, phi_dot_opt), phi_dot_opt)

    phi_next = phi_t + second_term * delta_t + third_term # Updated joint angles

    return phi_next

def main():
    rospy.init_node('manipulability_control_node')

    # Initialize the subscriber for joint states
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)

    # Initialize the subscriber for compliant box pose
    rospy.Subscriber('/compliant_box_pose', Pose, compliant_box_pose_callback)

    # Initialize the subscriber for the grasp matrix (assuming it's published as a Float64MultiArray)
    rospy.Subscriber('/grasp_matrix_topic', Float64MultiArray, grasp_matrix_callback)

    # Initialize the subscriber for the hand Jacobian (assuming it's published as a Float64MultiArray)
    rospy.Subscriber('/hand_jacobian_topic', Float64MultiArray, hand_jacobian_callback)

    rospy.loginfo("Waiting for data from topics...")
    
    # Load trajectory (desired positions)
    positions, _ = load_trajectory('box_trajectory.csv')

    current_index = 0  # Index to keep track of the current point in the trajectory
    delta_t = 0.001  # Time step

    while not rospy.is_shutdown():
        # Assuming that A and q_d are available
        A = compute_A(G_matrix, Hand_Jacobian)
        if A is not None:
            q_d = positions[current_index]  # The desired position at the current time step
            phi_next = compute_next_phi(A, current_index, q_d, positions, delta_t=delta_t)
            rospy.loginfo(f"Next joint angles: {phi_next}")

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
