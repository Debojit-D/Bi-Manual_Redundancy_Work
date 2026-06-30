# redundancy_optimization.py
import numpy as np
import grasp_matrix_and_hand_jacobian_cal

def velocity_manipulability(A):
    """Compute velocity manipulability."""
    try:
        AA_T = np.dot(A, A.T)
        #print(np.sqrt(np.linalg.det(AA_T)))
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
    F_diag = np.diag(F)
    #print(AA_T)
    first_term = AA_T / np.trace(AA_T)
    second_term = F_diag / np.trace(F_diag)
    return np.sqrt(np.trace(np.dot((first_term-second_term), (first_term-second_term).T)))

def compute_null_space_component(J_h):
    """Compute null space matrix."""
    try:
        J_h_pinv = np.linalg.pinv(J_h)
        I = np.eye(J_h.shape[1])
        null_space_matrix = I - np.dot(J_h_pinv, J_h)
        return null_space_matrix
    except np.linalg.LinAlgError as e:
        return None
    
def compute_phi_dot_opt(A, optimization_type, gain=100000):
    """Compute optimal joint velocities for optimization."""
    
    if optimization_type == "velocity_manipulability":
        W = velocity_manipulability(A)
    elif optimization_type == "force_manipulability":
        W = force_manipulability(A)
    elif optimization_type == "directional_force_manipulability":
        F = np.array([0.0, 0.0, -9.81])  # Assuming 6D force vector
        W = directional_force_manipulability(A,F)
    else:
        raise ValueError("Invalid optimization type")
    
    grad_W = grad_manipulability(A, W)
    phi_dot_opt = gain * grad_W
    return phi_dot_opt

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

def compute_next_phi(
    model,
    data,       
    current_joint_states,
    object_desired_position,  
    object_current_position,     
    object_desired_velocity,       
    object_current_velocity,    
    delta_t=0.01, 
    K_p=0.01
    ):
    
    # Compute grasp matrix and hand Jacobian
    G = grasp_matrix_and_hand_jacobian_cal.grasp_matrix_calculator(model, data)
    J_h = grasp_matrix_and_hand_jacobian_cal.hand_jacobian_calculator(model, data)
    

    # Compute A matrix
    G_inv_transpose = np.linalg.pinv(G).T
    A = np.dot(G_inv_transpose, J_h)
    A_pinv = np.linalg.pinv(A)

    # Compute trajectory tracking error
    trajectory_tracking_pos_error = object_desired_position - object_current_position

    # Compute null-space component
    null_space_matrix = compute_null_space_component(J_h)
    optimization_term = compute_phi_dot_opt(A, "velocity_manipulability")

    # **Fix: Ensure velocity_error has shape (6,1)**
    velocity_error = (object_desired_velocity + K_p * trajectory_tracking_pos_error).reshape(-1, 1)

    # Compute first, second, and third terms
    first_term = current_joint_states
    second_term = (A_pinv @ velocity_error).flatten() * delta_t  # Flatten ensures correct shape
    third_term = (null_space_matrix @ optimization_term)

    # Compute next joint state
    next_joint_angles = first_term + second_term*0 + third_term*0
    
    #print(third_term)

    return next_joint_angles[0:7], next_joint_angles[7:]


    

