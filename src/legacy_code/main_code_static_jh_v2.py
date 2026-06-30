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

delta_t = 0.01  # Time step
# Global variables to store the G matrix and Hand Jacobian
G_matrix = None
Hand_Jacobian = None
current_joint_states = None
box_pose = None
trajectory_data = None

# Global variables to store desired orientations for left and right end-effectors
desired_left = None
desired_right = None
current_left = None
current_right = None

def save_joint_angles_to_csv(file_name, joint_angles):
    with open(file_name, mode='a', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(joint_angles)  # Write the joint angles as a new row

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

def compute_A(G_matrix, Hand_Jacobian):
    if G_matrix is None or Hand_Jacobian is None:
        rospy.logwarn("G_matrix or Hand_Jacobian not received yet!")
        return None
    
    G_inv_transpose = (np.linalg.pinv(G_matrix)).T
    A = np.dot(G_inv_transpose, Hand_Jacobian)
    return A

def compute_phi_dot_opt(A, optimization_type, gain=0.01):
    
    if optimization_type == "velocity_manipulability":
        W = velocity_manipulability(A)
    elif optimization_type == "force_manipulability":
        W = force_manipulability(A)
    elif optimization_type == "directional_force_manipulability":
        W = directional_force_manipulability(A)
    else:
        raise ValueError("Invalid optimization type")

    W_orientation = orientation_manipulability(desired_left, current_left, desired_right, current_right)
    W = 2*W + W_orientation

    grad_W = grad_manipulability(A, W)
    phi_dot_opt = gain * grad_W
    return phi_dot_opt

def compute_null_space_component(J_h):
    try:
        J_h_pinv = np.linalg.pinv(J_h)
        I = np.eye(J_h.shape[1])
        null_space_matrix = I - np.dot(J_h_pinv, J_h)
        return null_space_matrix

    except np.linalg.LinAlgError as e:
        rospy.logerr(f"Error computing pseudo-inverse or null space matrix: {e}")
        return None

def orientation_manipulability(desired_left, current_left, desired_right, current_right):
    # Convert quaternions to Euler angles
    desired_left_euler = tf.euler_from_quaternion(desired_left)
    current_left_euler = tf.euler_from_quaternion(current_left)
    desired_right_euler = tf.euler_from_quaternion(desired_right)
    current_right_euler = tf.euler_from_quaternion(current_right)

    # Hardcoded Lambda values
    lambda_1 = 0.000 # Example value for left arm
    lambda_2 = 0.000  # Example value for right arm

    # Compute the squared L2 norm error for the left arm
    left_error_squared = np.sum((np.array(desired_left_euler) - np.array(current_left_euler)) ** 2)
    left_orientation_error = lambda_1 * left_error_squared

    # Compute the squared L2 norm error for the right arm
    right_error_squared = np.sum((np.array(desired_right_euler) - np.array(current_right_euler)) ** 2)
    right_orientation_error = lambda_2 * right_error_squared

    return left_orientation_error + right_orientation_error


def velocity_manipulability(A):
    AA_T = np.dot(A, A.T)
    return np.sqrt(abs(np.linalg.det(AA_T)))

def force_manipulability(A):
    AA_T = np.dot(A, A.T)
    AA_T_pinv = np.linalg.pinv(AA_T)  #
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

def grasp_matrix_callback(msg):
    global G_matrix
    G_matrix = np.array(msg.data).reshape(6, 6)  # Assuming 6x6 Grasp Matrix

def hand_jacobian_callback(msg):
    global Hand_Jacobian
    try:
        Hand_Jacobian = np.array(msg.data).reshape(6, 14)  # 6 rows, 14 columns for 2x 7 DOF robots
    except Exception as e:
        rospy.logerr(f"Error in receiving or reshaping Hand Jacobian: {e}")
        Hand_Jacobian = None

def joint_state_callback(msg):
    global current_joint_states, desired_left, desired_right, current_left, current_right
    current_joint_states = np.array(msg.position)  # Assuming 14 joint states (2 arms with 7 DOF each)
    
    # For simplicity, assume we extract current left and right orientations from joint states
    current_left = current_joint_states[:7]  # Left arm joint angles
    current_right = current_joint_states[7:]  # Right arm joint angles
    
    # Initialize desired_left and desired_right with the first recorded orientations
    if desired_left is None and desired_right is None:
        desired_left = current_left
        desired_right = current_right
        rospy.loginfo("Desired orientations set for the first time.")

def compliant_box_pose_callback(msg):
    global box_pose
    position = [msg.position.x, msg.position.y, msg.position.z]
    orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    box_pose = position + orientation

def compute_next_phi(A, J_h, current_index, delta_t=0.01, K_p=10):
    global current_joint_states

    if current_joint_states is None:
        rospy.logwarn("Joint states not available yet!")
        return None

    if not isinstance(J_h, np.ndarray) or J_h.shape != (6, 14):
        rospy.logwarn(f"Invalid Hand Jacobian: {J_h}")
        return None
    
    phi_t = current_joint_states
    phi_dot_opt = compute_phi_dot_opt(A, "velocity_manipulability")
    null_space_matrix = compute_null_space_component(J_h)
    if null_space_matrix is None:
        rospy.logwarn("Null space matrix is None. Skipping iteration.")
        return None
    third_term = np.dot(null_space_matrix, phi_dot_opt)

    phi_next = phi_t + third_term

    return phi_next


def command_joint_states(joint_states):
    global left_arm_pub, right_arm_pub
    
    left_joint_states = joint_states[:7]
    right_joint_states = joint_states[7:]
    
    left_arm_traj = JointTrajectory()
    left_arm_traj.joint_names = [f'left_arm_joint{i+1}' for i in range(7)]
    left_arm_point = JointTrajectoryPoint()
    left_arm_point.positions = left_joint_states
    left_arm_point.time_from_start = rospy.Duration(delta_t)
    left_arm_traj.points.append(left_arm_point)
    left_arm_pub.publish(left_arm_traj)

    right_arm_traj = JointTrajectory()
    right_arm_traj.joint_names = [f'right_arm_joint{i+1}' for i in range(7)]
    right_arm_point = JointTrajectoryPoint()
    right_arm_point.positions = right_joint_states
    right_arm_point.time_from_start = rospy.Duration(delta_t)
    right_arm_traj.points.append(right_arm_point)
    right_arm_pub.publish(right_arm_traj)

def move_robot_and_save_joints(csv_file_name="joint_angles.csv"):
    global left_arm_pub, right_arm_pub

    current_index = 0
    delta_t = 0.01  # Time step

    with open(csv_file_name, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([f'Left_Arm_Joint_{i+1}' for i in range(7)] + [f'Right_Arm_Joint_{i+1}' for i in range(7)])

        while not rospy.is_shutdown():
            A = compute_A(G_matrix, Hand_Jacobian)

            # Ensure Hand_Jacobian is properly passed to compute_next_phi
            if A is not None and Hand_Jacobian is not None:
                next_joint_states = compute_next_phi(A, Hand_Jacobian, current_index, delta_t)

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
