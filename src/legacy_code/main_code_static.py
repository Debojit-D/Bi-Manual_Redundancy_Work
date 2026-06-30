import rospy
import numpy as np
import csv

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
from gazebo_msgs.srv import SetModelState
import tf.transformations as tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_msgs.msg import ModelState


# Global variables to store the G matrix and Hand Jacobian
G_matrix = None
Hand_Jacobian = None
current_joint_states = None
box_pose = None
trajectory_data = None

def save_joint_angles_to_csv(file_name, joint_angles):
    """
    Append the joint angles to a CSV file.
    
    Args:
        file_name (str): The name of the CSV file to write to.
        joint_angles (list or array): The list of joint angles (14 elements).
    """
    with open(file_name, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(joint_angles)  # Write the joint angles as a new row

def generate_z_trajectory(start_z, end_z, num_points=500):
    """
    Generate a smooth trajectory for the z-axis motion of the box.
    
    Args:
        start_z (float): The starting z position of the box.
        end_z (float): The final z position of the box.
        num_points (int): The number of points to generate in the trajectory.
        
    Returns:
        numpy array: A trajectory of z positions from start_z to end_z.
    """
    return np.linspace(start_z, end_z, num_points)

def grad_manipulability(A, W, delta=1e-6):
    """
    Computes the gradient of the manipulability measure W with respect to the joint angles.
    
    Parameters:
    A (numpy array): The Jacobian or grasp matrix involved in the manipulability calculation.
    W (float): The manipulability measure (e.g., sqrt(det(AA^T))).
    delta (float): Small perturbation for finite differences (default is 1e-6).
    
    Returns:
    numpy array: Gradient of the manipulability w.r.t joint angles (phi).
    """
    
    # Number of joints (or degrees of freedom)
    num_joints = A.shape[1]
    # Initialize gradient vector for each joint
    grad_W = np.zeros(num_joints)
    # Compute the current value of W before perturbing any joints
    W_current = W
    
    # Loop over each joint and compute finite difference gradient
    for i in range(num_joints):
        # Perturb the ith joint by a small delta
        A_perturbed = A.copy()
        A_perturbed[:, i] += delta  # Add delta to column i (representing ith joint)
        
        # Recompute manipulability for the perturbed joint angles
        W_perturbed = np.sqrt(np.linalg.det(A_perturbed @ A_perturbed.T))
        
        # Compute the finite difference approximation of the gradient
        grad_W[i] = (W_perturbed - W_current) / delta
    
    return grad_W

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
def compute_phi_dot_opt(A, optimization_type, gain=0.00000001):
    """
    Computes the gradient of the manipulability measure with respect to joint angles.
    
    Parameters:
    A (numpy array): The Jacobian matrix or a related matrix involved in manipulability.
    optimization_type (str): Type of manipulability optimization ("velocity_manipulability", 
                             "force_manipulability", "directional_force_manipulability").
    gain (float): Gain to scale the gradient contribution (default is 1e-8).
    
    Returns:
    numpy array: Gradient of manipulability (phi_dot_opt), scaled by the gain.
    """
    
    # Compute manipulability measure W based on the selected optimization type
    if optimization_type == "velocity_manipulability":
        W = velocity_manipulability(A)
    elif optimization_type == "force_manipulability":
        W = force_manipulability(A)
    elif optimization_type == "directional_force_manipulability":
        W = directional_force_manipulability(A)
    else:
        raise ValueError("Invalid optimization type")

    # Compute the gradient of W w.r.t joint angles (phi)
    # Here we assume a 'grad_manipulability' function that calculates this gradient
    grad_W = grad_manipulability(A, W)

    # Apply the gain scaling to the gradient
    phi_dot_opt = gain * grad_W
    
    return phi_dot_opt

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

def compute_q_dot_d(current_index, delta_t, z_trajectory, orientations):
    """
    Compute the desired velocity q_dot_d for the box's z-axis motion and angular velocity for orientation.

    Parameters:
    - current_index: The current index of the trajectory.
    - delta_t: The time step.
    - z_trajectory: The z-axis trajectory of the box.
    - orientations: List of box orientations (quaternions).
    
    Returns:
    - q_dot_d: A 6-element array (3 for velocity, 3 for angular velocity).
    """
    # Check if the current index is valid
    if current_index >= len(z_trajectory) - 1:
        rospy.logwarn("Reached the end of the trajectory. Using zero velocity.")
        return np.zeros(6)  # Return zero for both position (3) and orientation (3) velocities

    # Compute the z-axis velocity using finite differences
    z_velocity = (z_trajectory[current_index + 1] - z_trajectory[current_index]) / delta_t
    q_dot_d_position = np.array([0, 0, z_velocity])  # Only z velocity is non-zero

    # Orientation difference between consecutive quaternions
    q1 = orientations[current_index]  # Current quaternion
    q2 = orientations[current_index + 1]  # Next quaternion

    # Compute the angular velocity using quaternion difference
    q_diff = tf.quaternion_multiply(tf.quaternion_inverse(q1), q2)
    angular_velocity = 2 * np.array([q_diff[0], q_diff[1], q_diff[2]]) / delta_t

    # Combine position velocity and angular velocity into a single array
    q_dot_d = np.concatenate([q_dot_d_position, angular_velocity])

    return 0.0 * q_dot_d  # Scale the velocity by a factor (e.g., 0.1)

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

# Function to compute joint velocities and manipulability
def compute_next_phi(A, current_index, delta_t=0.1, K_p=10):
    global current_joint_states

    if current_joint_states is None:
        rospy.logwarn("Joint states not available yet!")
        return None

    phi_t = current_joint_states
    q_dot_d = np.zeros(6)  # Since the box is stationary, desired velocity is zero
    e = np.zeros(6)  # Since the box is stationary, the error is zero

    A_pinv = np.linalg.pinv(A)
    phi_dot_opt = compute_phi_dot_opt(A, "velocity_manipulability")

    second_term = np.dot(A_pinv, (q_dot_d + K_p * e))
    third_term = np.dot(compute_null_space_component(A), phi_dot_opt)

    phi_next = phi_t + third_term

    return phi_next

# Command the robot with calculated joint states
def command_joint_states(joint_states):
    global left_arm_pub, right_arm_pub
    
    left_joint_states = joint_states[:7]
    right_joint_states = joint_states[7:]
    
    left_arm_traj = JointTrajectory()
    left_arm_traj.joint_names = [f'left_arm_joint{i+1}' for i in range(7)]
    left_arm_point = JointTrajectoryPoint()
    left_arm_point.positions = left_joint_states
    left_arm_point.time_from_start = rospy.Duration(0.1)
    left_arm_traj.points.append(left_arm_point)
    left_arm_pub.publish(left_arm_traj)

    right_arm_traj = JointTrajectory()
    right_arm_traj.joint_names = [f'right_arm_joint{i+1}' for i in range(7)]
    right_arm_point = JointTrajectoryPoint()
    right_arm_point.positions = right_joint_states
    right_arm_point.time_from_start = rospy.Duration(0.1)
    right_arm_traj.points.append(right_arm_point)
    right_arm_pub.publish(right_arm_traj)

# Function to move the box in Gazebo by updating its position
def set_box_state(position, orientation):
    rospy.wait_for_service('/gazebo/set_model_state')
    try:
        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        box_state = ModelState()
        box_state.model_name = 'compliant_box'  # Assuming your box model is named 'compliant_box'
        box_state.pose.position.x = position[0]
        box_state.pose.position.y = position[1]
        box_state.pose.position.z = position[2]
        box_state.pose.orientation.x = orientation[0]
        box_state.pose.orientation.y = orientation[1]
        box_state.pose.orientation.z = orientation[2]
        box_state.pose.orientation.w = orientation[3]
        
        set_model_state(box_state)
        rospy.loginfo(f"Moved box to position: {position}, orientation: {orientation}")
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


# Function to move the box and command robot end-effectors
def move_robot_and_save_joints(csv_file_name="joint_angles.csv"):
    global left_arm_pub, right_arm_pub

    current_index = 0
    delta_t = 0.1  # Time step

    with open(csv_file_name, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([f'Left_Arm_Joint_{i+1}' for i in range(7)] + [f'Right_Arm_Joint_{i+1}' for i in range(7)])

        while not rospy.is_shutdown():
            A = compute_A(G_matrix, Hand_Jacobian)

            if A is not None:
                next_joint_states = compute_next_phi(A, current_index, delta_t)

                if next_joint_states is not None:
                    command_joint_states(next_joint_states)
                    writer.writerow(next_joint_states)

                current_index += 1

            rospy.sleep(delta_t)


def main():
    global left_arm_pub, right_arm_pub

    rospy.init_node('manipulability_control_node')

    left_arm_pub = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
    right_arm_pub = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=10)

    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.Subscriber('/grasp_matrix', Float64MultiArray, grasp_matrix_callback)
    rospy.Subscriber('/hand_jacobian', Float64MultiArray, hand_jacobian_callback)

    rospy.loginfo("Waiting for data from topics...")

    move_robot_and_save_joints()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
