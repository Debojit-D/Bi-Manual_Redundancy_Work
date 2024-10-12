### This code focusses on moving the box and the robot together. 

### This code is to debug the section why the det(A.A_Transpose) is not positive.
import rospy
import numpy as np
import csv
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import tf.transformations as tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

delta_t = 0.01  # Time step

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

def quaternion_to_angular_velocity(q1, q2, dt):
    """Computes angular velocity from the change in two quaternions."""
    q1_inv = tf.quaternion_inverse(q1)
    q_diff = tf.quaternion_multiply(q1_inv, q2)  # Relative quaternion
    angular_velocity = 2 * np.array(q_diff[:3]) / dt  # Divide by time step to get velocity
    return angular_velocity

def read_trajectory_and_compute_q_dot_d(csv_file, dt):
    trajectory_data = []

    # Reading the CSV file
    with open(csv_file, newline='') as csvfile:
        reader = csv.reader(csvfile)
        next(reader)  # Skip header row
        for row in reader:
            position_orientation = list(map(float, row))  # Convert all values to float
            trajectory_data.append(position_orientation)

    trajectory_data = np.array(trajectory_data)  # Convert to numpy array for easy manipulation

    # Initialize lists to store positions, orientations, and velocities
    positions = []
    orientations = []
    velocities = []

    # Compute q_dot_d (velocity) for each time step
    for i in range(1, len(trajectory_data)):
        # Store position (first 3 values)
        positions.append(trajectory_data[i][:3])  # Only positions (x, y, z)

        # Store orientation (last 4 values as quaternion)
        orientations.append(trajectory_data[i][3:])  # Only orientation (quaternion: x, y, z, w)

        # Difference in position (linear velocity)
        delta_position = trajectory_data[i][:3] - trajectory_data[i-1][:3]
        linear_velocity = delta_position / dt

        # Compute angular velocity from quaternion difference
        q_current = trajectory_data[i][3:]  # Current quaternion
        q_previous = trajectory_data[i-1][3:]  # Previous quaternion
        angular_velocity = quaternion_to_angular_velocity(q_previous, q_current, dt)

        # Combine linear and angular velocities
        q_dot_d = np.concatenate((linear_velocity, angular_velocity))
        velocities.append(q_dot_d)

    return positions, orientations, velocities


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
    
    G_inv_transpose = np.linalg.pinv(G_matrix).T
    A = np.dot(G_inv_transpose, Hand_Jacobian)
    return A

def compute_phi_dot_opt(A, optimization_type, gain=30000):
    
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

def compute_null_space_component(J_h):
    try:
        J_h_pinv = np.linalg.pinv(J_h)
        I = np.eye(J_h.shape[1])
        null_space_matrix = I - np.dot(J_h_pinv, J_h)
        return null_space_matrix

    except np.linalg.LinAlgError as e:
        rospy.logerr(f"Error computing pseudo-inverse or null space matrix: {e}")
        return None

def velocity_manipulability(A):
    AA_T = np.dot(A, A.T)
    return np.sqrt(np.linalg.det(AA_T))

def force_manipulability(A):
    AA_T = np.dot(A, A.T)
    AA_T_pinv = np.linalg.pinv(AA_T)  #
    return np.sqrt(np.linalg.det(AA_T_pinv))

def directional_force_manipulability(A, F):
    AA_T = np.dot(A, A.T)
    first_term = AA_T / np.trace(AA_T)
    second_term = F / np.trace(F)
    return np.sqrt(np.trace(first_term - second_term))

def q_forward_kinematics():
    global box_pose
    if box_pose is None:
        rospy.logwarn("Box pose not available yet!")
        return np.zeros(7)  # Return zeros until the box pose is available
    return np.array(box_pose)  # Return the current box position and orientation (6 components)

def grasp_matrix_callback(msg):
    global G_matrix
    G_matrix = np.array(msg.data).reshape(6, 12)  # Assuming 6x6 Grasp Matrix

def hand_jacobian_callback(msg):
    global Hand_Jacobian
    try:
        Hand_Jacobian = np.array(msg.data).reshape(12, 14)  # 6 rows, 14 columns for 2x 7 DOF robots
    except Exception as e:
        rospy.logerr(f"Error in receiving or reshaping Hand Jacobian: {e}")
        Hand_Jacobian = None

def joint_state_callback(msg):
    global current_joint_states
    current_joint_states = np.array(msg.position)  # Assuming 14 joint states (2 arms with 7 DOF each)

def compliant_box_pose_callback(msg):
    global box_pose
    position = [msg.position.x, msg.position.y, msg.position.z]
    orientation = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    box_pose = position + orientation

def compute_error_in_euler(q_d, q_fwd_kinematics):
    # Position error (same as before)
    position_error = q_d[:3] - q_fwd_kinematics[:3]

    # Extract quaternions
    q_d_quaternion = q_d[3:]  # Quaternion of desired orientation
    q_fwd_quaternion = q_fwd_kinematics[3:]  # Quaternion from forward kinematics

    # Convert both quaternions to Euler angles
    euler_d = tf.euler_from_quaternion(q_d_quaternion)
    euler_fwd = tf.euler_from_quaternion(q_fwd_quaternion)

    # Compute orientation error in Euler angles
    orientation_error = np.array(euler_d) - np.array(euler_fwd)

    # Combine position and orientation errors
    error = np.concatenate([position_error, orientation_error])
    
    return error

def compute_next_phi(A, J_h, current_index, positions, orientations, q_dot_d, delta_t=0.01, K_p=0.5):
    global current_joint_states

    if current_joint_states is None:
        rospy.logwarn("Joint states not available yet!")
        return None

    if not isinstance(J_h, np.ndarray) or J_h.shape != (12, 14):
        rospy.logwarn(f"Invalid Hand Jacobian: {J_h}")
        return None

    ## First Term Calculation
    phi_t = current_joint_states
    
    ## Third Term Calculation
    phi_dot_opt = compute_phi_dot_opt(A, "velocity_manipulability")
    null_space_matrix = compute_null_space_component(J_h)

    if null_space_matrix is None:
        rospy.logwarn("Null space matrix is None. Skipping iteration.")
        return None
    third_term = np.dot(null_space_matrix, phi_dot_opt)
    
    ## Second Term Calculation
    try:
        q_d = np.concatenate([positions, orientations])  # Combine position and orientation
    except ValueError as e:
        print(f"Error concatenating: {e}")
        return None
    A_pinv = np.linalg.pinv(A)
    error = compute_error_in_euler(q_d, q_forward_kinematics())
    second_term = np.dot(A_pinv, (abs(q_dot_d) + K_p * error)) * delta_t

    # Compute the next joint states
    phi_next = phi_t + second_term + third_term

    return phi_next


def command_joint_states(joint_states):
    global left_arm_pub, right_arm_pub

    # Extract joint states for both arms
    left_joint_states = joint_states[:7]  # First 7 for the left arm
    right_joint_states = joint_states[7:]  # Next 7 for the right arm

    # Create JointTrajectory message for the left arm
    left_arm_traj = JointTrajectory()
    left_arm_traj.joint_names = [f'left_arm_joint{i+1}' for i in range(7)]
    left_arm_point = JointTrajectoryPoint()
    left_arm_point.positions = left_joint_states
    left_arm_point.time_from_start = rospy.Duration(delta_t)
    left_arm_traj.points.append(left_arm_point)

    # Create JointTrajectory message for the right arm
    right_arm_traj = JointTrajectory()
    right_arm_traj.joint_names = [f'right_arm_joint{i+1}' for i in range(7)]
    right_arm_point = JointTrajectoryPoint()
    right_arm_point.positions = right_joint_states
    right_arm_point.time_from_start = rospy.Duration(delta_t)
    right_arm_traj.points.append(right_arm_point)

    # Publish both arm commands together
    left_arm_pub.publish(left_arm_traj)
    right_arm_pub.publish(right_arm_traj)

def move_robot_and_save_joints(csv_file_name="joint_angles.csv"):
    global left_arm_pub, right_arm_pub
    current_index = 0
    delta_t = 0.01  # Time step
    q_d_full_pos, q_d_full_ort, q_ddot_full = read_trajectory_and_compute_q_dot_d("box_trajectory.csv", delta_t)

    with open(csv_file_name, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([f'Left_Arm_Joint_{i+1}' for i in range(7)] + [f'Right_Arm_Joint_{i+1}' for i in range(7)])

        while not rospy.is_shutdown():
            A = compute_A(G_matrix, Hand_Jacobian)

            # Ensure Hand_Jacobian is properly passed to compute_next_phi
            if A is not None and Hand_Jacobian is not None and current_index < len(q_d_full_pos):
                q_d_pos = q_d_full_pos[current_index]
                q_d_ort = q_d_full_ort[current_index] # Desired box position for the current time step
                q_dot_d = q_ddot_full[current_index]  # Desired velocity for the current time step
                next_joint_states = compute_next_phi(
                    A, 
                    Hand_Jacobian, 
                    current_index, 
                    q_d_pos,
                    q_d_ort, 
                    q_dot_d, 
                    delta_t
                )

                if next_joint_states is not None:
                    command_joint_states(next_joint_states)
                    writer.writerow(next_joint_states)

                current_index += 1
                print(f"Current Index: {current_index}")
            

            rospy.sleep(delta_t)

def main():
    global left_arm_pub, right_arm_pub

    rospy.init_node('manipulability_control_node')

    left_arm_pub = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=10)
    right_arm_pub = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=10)

    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.Subscriber('/grasp_matrix', Float64MultiArray, grasp_matrix_callback)
    rospy.Subscriber('/hand_jacobian', Float64MultiArray, hand_jacobian_callback)
    rospy.Subscriber('/compliant_box_pose', Pose, compliant_box_pose_callback)

    rospy.loginfo("Waiting for data from topics...")

    move_robot_and_save_joints()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
