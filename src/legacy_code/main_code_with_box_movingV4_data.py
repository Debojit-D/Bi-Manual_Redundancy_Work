#### This is working for velocity manipulability

import rospy
import numpy as np
import csv
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Pose
import tf.transformations as tf
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

delta_t = 0.01  # Time step

G_matrix = None
Hand_Jacobian = None
current_joint_states = None
box_pose = None

def quaternion_to_angular_velocity(q1, q2, dt):
    """Computes angular velocity from the change in two quaternions."""
    q1_inv = tf.quaternion_inverse(q1)
    q_diff = tf.quaternion_multiply(q1_inv, q2)  # Relative quaternion
    angular_velocity = 2 * np.array(q_diff[:3]) / dt  # Divide by time step to get velocity
    return angular_velocity

def read_trajectory_and_compute_q_dot_d(csv_file, dt):
    trajectory_data = []

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
    """Computes the gradient of the manipulability measure W with respect to joint angles."""
    num_joints = A.shape[1]
    grad_W = np.zeros(num_joints)
    W_current = W
    
    for i in range(num_joints):
        A_perturbed = A.copy()
        A_perturbed[:, i] += delta  # Add delta to column i (representing ith joint)
        W_perturbed = np.sqrt(np.linalg.det(A_perturbed @ A_perturbed.T))
        grad_W[i] = (W_perturbed - W_current) / delta
    
    return grad_W

def compute_A(G_matrix, Hand_Jacobian):
    """Compute matrix A."""
    if G_matrix is None or Hand_Jacobian is None:
        rospy.logwarn("G_matrix or Hand_Jacobian not received yet!")
        return None
    
    G_inv_transpose = np.linalg.pinv(G_matrix).T
    A = np.dot(G_inv_transpose, Hand_Jacobian)
    return A

def compute_phi_dot_opt(A, optimization_type, gain=45000):
    """Compute optimal joint velocities for optimization."""
    if optimization_type == "velocity_manipulability":
        W = velocity_manipulability(A)
    elif optimization_type == "force_manipulability":
        W = force_manipulability(A)
    elif optimization_type == "directional_force_manipulability":
        #F = np.array([0, 0, 0, 0, 0, 0])
        W = directional_force_manipulability(A)
    else:
        raise ValueError("Invalid optimization type")

    grad_W = grad_manipulability(A, W)
    phi_dot_opt = gain * grad_W
    return phi_dot_opt

def compute_null_space_component(J_h):
    """Compute null space matrix."""
    try:
        J_h_pinv = np.linalg.pinv(J_h)
        I = np.eye(J_h.shape[1])
        null_space_matrix = I - np.dot(J_h_pinv, J_h)
        return null_space_matrix
    except np.linalg.LinAlgError as e:
        rospy.logerr(f"Error computing pseudo-inverse or null space matrix: {e}")
        return None

def velocity_manipulability(A):
    """Compute velocity manipulability."""
    try:
        AA_T = np.dot(A, A.T)
        return np.sqrt(np.linalg.det(AA_T))
    except np.linalg.LinAlgError:
        return np.nan  # Return NaN if the matrix is not positive definite

def force_manipulability(A):
    """Compute force manipulability."""
    try:
        AA_T = np.dot(A, A.T)
        AA_T_pinv = np.linalg.pinv(AA_T)
        return np.sqrt(np.linalg.det(AA_T_pinv))
    except np.linalg.LinAlgError:
        return np.nan

def directional_force_manipulability(A, F):
    """Compute directional force manipulability."""
    AA_T = np.dot(A, A.T)
    first_term = AA_T / np.trace(AA_T)
    second_term = F / np.trace(F)
    return np.sqrt(np.trace(first_term - second_term))

def q_forward_kinematics():
    """Get the forward kinematics from box pose."""
    global box_pose
    if box_pose is None:
        rospy.logwarn("Box pose not available yet!")
        return np.zeros(7)  # Return zeros until the box pose is available
    return np.array(box_pose)

def grasp_matrix_callback(msg):
    global G_matrix
    G_matrix = np.array(msg.data).reshape(6, 12)  # Assuming 6x12 Grasp Matrix

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
    """Compute position and orientation error between desired and forward kinematics in Euler angles."""
    position_error = q_d[:3] - q_fwd_kinematics[:3]
    q_d_quaternion = q_d[3:]
    q_fwd_quaternion = q_fwd_kinematics[3:]
    euler_d = tf.euler_from_quaternion(q_d_quaternion)
    euler_fwd = tf.euler_from_quaternion(q_fwd_quaternion)
    orientation_error = np.array(euler_d) - np.array(euler_fwd)
    error = np.concatenate([position_error, orientation_error])
    return error

def compute_next_phi(A, J_h, current_index, positions, orientations, q_dot_d, delta_t=0.01, Kp=1.2):
    """Compute the next joint state."""
    global current_joint_states
    if current_joint_states is None or not isinstance(J_h, np.ndarray) or J_h.shape != (12, 14):
        rospy.logwarn("Joint states or Hand Jacobian not available!")
        return None

    phi_t = current_joint_states
    phi_dot_opt = compute_phi_dot_opt(A, "velocity_manipulability")
    null_space_matrix = compute_null_space_component(J_h)
    if null_space_matrix is None:
        rospy.logwarn("Null space matrix is None. Skipping iteration.")
        return None
    third_term = np.dot(null_space_matrix, phi_dot_opt)

    try:
        #positions = np.array([0.29999993624392374, 3.4717179166135275e-08, 1.1])
        #orientations = np.array([2.007355688987178e-08, -5.531757113221283e-09, -8.205284718919182e-08, 0.9999999999999964])
        q_d = np.concatenate([positions, orientations])
    except ValueError as e:
        rospy.logerr(f"Error concatenating: {e}")
        return None
    A_pinv = np.linalg.pinv(A)
    error = compute_error_in_euler(q_d, q_forward_kinematics())
    K_p_matrix = np.diag([Kp, Kp, Kp, Kp, Kp, Kp])
    second_term = np.dot(A_pinv, ((q_dot_d) + K_p_matrix @ error)) * delta_t
    phi_next = phi_t + second_term + third_term
    return phi_next

def command_joint_states(joint_states):
    """Command joint states to both arms."""
    global left_arm_pub, right_arm_pub
    left_joint_states = joint_states[:7]
    right_joint_states = joint_states[7:]
    left_arm_traj = JointTrajectory()
    left_arm_traj.joint_names = [f'left_arm_joint{i+1}' for i in range(7)]
    left_arm_point = JointTrajectoryPoint()
    left_arm_point.positions = left_joint_states
    left_arm_point.time_from_start = rospy.Duration(delta_t)
    left_arm_traj.points.append(left_arm_point)

    right_arm_traj = JointTrajectory()
    right_arm_traj.joint_names = [f'right_arm_joint{i+1}' for i in range(7)]
    right_arm_point = JointTrajectoryPoint()
    right_arm_point.positions = right_joint_states
    right_arm_point.time_from_start = rospy.Duration(delta_t)
    right_arm_traj.points.append(right_arm_point)

    left_arm_pub.publish(left_arm_traj)
    right_arm_pub.publish(right_arm_traj)

def move_robot_and_save_joints(csv_file_name="joint_angles.csv"):
    """Move the robot along the trajectory and save joint angles."""
    global left_arm_pub, right_arm_pub
    current_index = 0
    q_d_full_pos, q_d_full_ort, q_ddot_full = read_trajectory_and_compute_q_dot_d("box_trajectory.csv", delta_t)

    with open(csv_file_name, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow([f'Left_Arm_Joint_{i+1}' for i in range(7)] + 
                        [f'Right_Arm_Joint_{i+1}' for i in range(7)] + 
                        ['Velocity_Manipulability', 'Error_Position_X', 'Error_Position_Y', 'Error_Position_Z',
                         'Error_Orientation_Roll', 'Error_Orientation_Pitch', 'Error_Orientation_Yaw',
                         'Box_Position_X', 'Box_Position_Y', 'Box_Position_Z', 
                         'Box_Orientation_X', 'Box_Orientation_Y', 'Box_Orientation_Z', 'Box_Orientation_W'])
        while not rospy.is_shutdown() and current_index < len(q_d_full_pos):
            A = compute_A(G_matrix, Hand_Jacobian)
            if A is not None and Hand_Jacobian is not None:
                q_d_pos = q_d_full_pos[current_index]
                q_d_ort = q_d_full_ort[current_index]
                q_dot_d = q_ddot_full[current_index]
                next_joint_states = compute_next_phi(A, Hand_Jacobian, current_index, q_d_pos, q_d_ort, q_dot_d)
                if next_joint_states is not None:
                    command_joint_states(next_joint_states)
                    
                    # Log manipulability, joint states, and error
                    velocity_manip = velocity_manipulability(A)
                    q_d = np.concatenate([q_d_pos, q_d_ort])
                    error = compute_error_in_euler(q_d, q_forward_kinematics())
                    
                     # Get the current box pose
                    if box_pose is not None:
                        box_position = box_pose[:3]
                        box_orientation = box_pose[3:]
                    else:
                        box_position = [0, 0, 0]  # Default values if box pose not available
                        box_orientation = [0, 0, 0, 1]  # Default quaternion
                    
                    # Write joint states, velocity manipulability, error, and box pose to CSV
                    writer.writerow(list(next_joint_states) + 
                                    [velocity_manip] + 
                                    list(error[:3]) +  # Position error
                                    list(error[3:]) +  # Orientation error
                                    list(box_position) + list(box_orientation))  # Box pose (6-DoF)
                current_index += 1
                rospy.loginfo(f"Current Index: {current_index}")
            rospy.sleep(delta_t)

def main():
    global left_arm_pub, right_arm_pub
    rospy.init_node('manipulability_control_node')
    left_arm_pub = rospy.Publisher('/left_arm_trajectory_controller/command', JointTrajectory, queue_size=100)
    right_arm_pub = rospy.Publisher('/right_arm_trajectory_controller/command', JointTrajectory, queue_size=100)

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
