import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

def quintic_polynomial_trajectory(q0, qf, v0=0, vf=0, a0=0, af=0,
                                  total_time=2.0, time_step=0.01):
    """
    Generates a 1D quintic polynomial trajectory.
    Returns position, velocity, acceleration, time array.
    """
    t_values = np.arange(0, total_time, time_step)
    T = total_time

    # Build matrix for quintic boundary conditions
    M = np.array([
        [0,    0,    0,    0,  0, 1],
        [T**5, T**4, T**3, T**2, T, 1],
        [0,    0,    0,    0,  1, 0],
        [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
        [0,    0,    0,    2,  0, 0],
        [20*T**3, 12*T**2, 6*T, 2,  0, 0]
    ])
    b = np.array([q0, qf, v0, vf, a0, af])
    coeffs = np.linalg.solve(M, b)

    # Evaluate polynomials
    q_traj = np.polyval(coeffs, t_values)
    v_traj = np.polyval(np.polyder(coeffs), t_values)
    a_traj = np.polyval(np.polyder(coeffs, 2), t_values)

    return q_traj, v_traj, a_traj, t_values


def quat_conjugate(q):
    """
    Conjugate of quaternion q = [x, y, z, w].
    Returns q* = [-x, -y, -z, w].
    """
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=float)


def quat_multiply(q1, q2):
    """
    Hamilton product of two quaternions in [x, y, z, w] format.
    Returns q = q1 * q2.
    """
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2
    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    return np.array([x, y, z, w], dtype=float)


def generate_smooth_quintic_trajectory(
    initial_position, final_position,
    initial_quat, final_quat,
    total_time=2.0, time_step=0.01
):
    """
    Generates a smooth trajectory in Cartesian position + orientation.
    Returns:
      - position_trajectory (N, 3)
      - orientation_trajectory (N, 4)  # quaternions (x, y, z, w)
      - linear_velocity (N, 3)
      - linear_acceleration (N, 3)
      - angular_velocity (N, 3) [approx via direct quaternion derivative]
      - time_values (N,)
    """
    # Time discretization
    num_steps = int(total_time / time_step)
    time_values = np.linspace(0, total_time, num_steps)

    # --- 3D Position Trajectory ---
    position_trajectory = np.zeros((num_steps, 3))
    linear_velocity_trajectory = np.zeros((num_steps, 3))
    linear_acceleration_trajectory = np.zeros((num_steps, 3))

    for i in range(3):
        q_traj, v_traj, a_traj, _ = quintic_polynomial_trajectory(
            q0=initial_position[i],
            qf=final_position[i],
            v0=0, vf=0, a0=0, af=0,
            total_time=total_time,
            time_step=time_step
        )
        position_trajectory[:, i] = q_traj
        linear_velocity_trajectory[:, i] = v_traj
        linear_acceleration_trajectory[:, i] = a_traj

    # --- Orientation via SLERP ---
    key_times = np.array([0, total_time])
    key_rots = R.from_quat([initial_quat, final_quat])  # shape (2, 4)
    slerp = Slerp(key_times, key_rots)
    orientation_trajectory = np.array([slerp(t).as_quat() for t in time_values])
    # Note: orientation_trajectory[i] is [x, y, z, w]

    # --- Direct Quaternion Derivative => Angular Velocity ---
    # Compute finite difference of orientation in time, quat_dot[i] ~ d(q)/dt at i
    quat_dot = np.gradient(orientation_trajectory, time_step, axis=0)

    angular_velocity_trajectory = np.zeros((num_steps, 3))
    for i in range(num_steps):
        # q(t)
        q = orientation_trajectory[i]  # [x, y, z, w]
        # dq(t)/dt
        dq = quat_dot[i]

        # Use the formula: ω = 2 * ( dq ⊗ q* )_vec
        # where q* = conjugate of q, and (...)_vec is the x,y,z part.
        q_conj = quat_conjugate(q)
        product = quat_multiply(dq, q_conj)  # [x, y, z, w]
        # The vector part is product[:3]
        angular_velocity_trajectory[i] = 2.0 * product[:3]

    return (
        position_trajectory,
        orientation_trajectory,
        linear_velocity_trajectory,
        linear_acceleration_trajectory,
        angular_velocity_trajectory,
        time_values
    )
