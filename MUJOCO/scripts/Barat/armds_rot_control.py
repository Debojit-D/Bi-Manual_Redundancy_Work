import numpy as np
import scipy.spatial.transform as transform
from scipy.spatial.transform import Rotation, Slerp

def logistic_tau(t, t0, k):
    """
    Compute the logistic blending parameter tau and its derivative.
    
    tau(t) = 1 / (1 + exp(-k*(t - t0)))
    d_tau(t) = k * exp(-k*(t - t0)) / (1 + exp(-k*(t - t0)))^2
    """
    tau = 1.0 / (1.0 + np.exp(-k * (t - t0)))
    d_tau = k * np.exp(-k * (t - t0)) / ((1.0 + np.exp(-k * (t - t0)))**2)
    return tau, d_tau

class DSController:
    """
    DS Controller for both position and rotation.
    Supports quaternion-based orientation control.
    """
    def __init__(self, K_pos, K_rot):
        self.K_pos = K_pos  # Position control gain
        self.K_rot = K_rot  # Rotation control gain

    def compute_command(self, x_R, x_d, x_V, q_R, q_d, q_V, tau, d_tau, dot_x_V=None, dot_q_V=None):
        """
        Compute DS-based command for position and rotation.
        
        Parameters:
            x_R: Current position (R^3)
            x_d: Asynchronous target position
            x_V: Synchronous target position
            q_R: Current orientation (quaternion)
            q_d: Asynchronous target orientation
            q_V: Synchronous target orientation
            tau: Blending parameter
            d_tau: Time derivative of tau
            dot_x_V: Optional velocity of virtual target (default zeros)
            dot_q_V: Optional angular velocity of virtual target (default zeros)
        
        Returns:
            Tuple: (linear velocity, angular velocity)
        """
        if dot_x_V is None:
            dot_x_V = np.zeros_like(x_V)
        if dot_q_V is None:
            dot_q_V = np.zeros(3)

        # --- Position Blending ---
        x_target = tau * x_V + (1 - tau) * x_d
        v_command = tau * dot_x_V + d_tau * (x_V - x_d) - self.K_pos * (x_R - x_target)

        # --- Orientation Blending using SLERP ---
        key_times = [0, 1]  # Define the interpolation range
        key_rots = Rotation.from_quat([q_d, q_V])  # Create Rotation objects from quaternions
        slerp = Slerp(key_times, key_rots)  # Initialize SLERP with keyframes
        q_target = slerp([tau]).as_quat()[0]  # Interpolate at the given tau

        # --- Quaternion Error Computation ---
        q_error = Rotation.from_quat(q_R) * Rotation.from_quat(q_target).inv()
        e_rot = q_error.as_rotvec()  # Map error to a rotation vector

        # --- Compute Angular Velocity ---
        omega_command = -self.K_rot * e_rot + tau * dot_q_V

        return v_command, omega_command

class DualArmController:
    """
    DualArmController for bimanual position and orientation control.
    Incorporates both arms in one method.
    """
    def __init__(self, K_pos_left, K_rot_left, K_pos_right, K_rot_right):
        self.left_controller = DSController(K_pos_left, K_rot_left)
        self.right_controller = DSController(K_pos_right, K_rot_right)
        self.mode = "asynchronous"  # Modes: "asynchronous", "synchronous", "blending"
        self.blend_start_time = None
        self.blend_duration = 1.0
        self.blend_k = 10.0

    def set_mode(self, mode, current_time=None, blend_duration=None):
        """
        Set the control mode.
        """
        self.mode = mode
        if mode == "blending" and current_time is not None:
            self.blend_start_time = current_time
            if blend_duration is not None:
                self.blend_duration = blend_duration

    def get_tau(self, current_time):
        """
        Get the blending parameter tau and its derivative based on mode.
        """
        if self.mode == "synchronous":
            return 1.0, 0.0
        elif self.mode == "asynchronous":
            return 0.0, 0.0
        elif self.mode == "blending":
            if self.blend_start_time is None:
                self.blend_start_time = current_time
            t0 = self.blend_start_time + self.blend_duration / 2.0
            tau, d_tau = logistic_tau(current_time, t0, self.blend_k)
            return tau, d_tau
        else:
            return 0.0, 0.0

    def compute_commands(self,
                         x_R_left, x_R_right,
                         x_d_left, x_d_right,
                         x_V_left, x_V_right,
                         q_R_left, q_R_right,
                         q_d_left, q_d_right,
                         q_V_left, q_V_right,
                         current_time,
                         dot_x_V_left=None, dot_x_V_right=None,
                         dot_q_V_left=None, dot_q_V_right=None):
        """
        Compute control commands for both arms.
        
        Parameters:
            x_R_left, x_R_right: Current positions of left and right arms.
            x_d_left, x_d_right: Asynchronous target positions.
            x_V_left, x_V_right: Synchronous target positions.
            q_R_left, q_R_right: Current orientations (quaternions).
            q_d_left, q_d_right: Asynchronous target orientations.
            q_V_left, q_V_right: Synchronous target orientations.
            current_time: Current time to compute blending.
            dot_x_V_left, dot_x_V_right: Optional linear velocities for virtual targets.
            dot_q_V_left, dot_q_V_right: Optional angular velocities for virtual targets.
            
        Returns:
            Tuple: (v_left, omega_left, v_right, omega_right)
        """
        tau, d_tau = self.get_tau(current_time)

        # Compute command for left arm.
        v_left, omega_left = self.left_controller.compute_command(
            x_R_left, x_d_left, x_V_left,
            q_R_left, q_d_left, q_V_left,
            tau, d_tau, dot_x_V_left, dot_q_V_left)

        # Compute command for right arm.
        v_right, omega_right = self.right_controller.compute_command(
            x_R_right, x_d_right, x_V_right,
            q_R_right, q_d_right, q_V_right,
            tau, d_tau, dot_x_V_right, dot_q_V_right)

        return v_left, omega_left, v_right, omega_right

# --- Example usage in the main loop ---
if __name__ == "__main__":
    import time

    # Initialize the dual arm controller with gains for each arm.
    dual_arm = DualArmController(K_pos_left=2.0, K_rot_left=2.0,
                                 K_pos_right=2.0, K_rot_right=2.0)
    dual_arm.set_mode("asynchronous")

    # Define initial positions and orientations (quaternions)
    x_R_left = np.array([0.0, 0.0, 0.0])
    x_R_right = np.array([0.5, 0.0, 0.0])
    q_R_left = np.array([1, 0, 0, 0])  # Identity quaternion
    q_R_right = np.array([1, 0, 0, 0])

    # Asynchronous targets for each arm
    x_d_left = np.array([0.1, 0.2, 0.3])
    x_d_right = np.array([0.6, 0.2, 0.3])
    q_d_left = np.array([0.707, 0, 0.707, 0])  # 90-degree rotation about X
    q_d_right = np.array([0.707, 0, 0.707, 0])

    # Synchronous (virtual) targets for coordinated tasks
    x_V_left = np.array([0.2, 0.3, 0.4])
    x_V_right = np.array([0.7, 0.3, 0.4])
    q_V_left = np.array([0, 0, 1, 0])  # 180-degree rotation about Z
    q_V_right = np.array([0, 0, 1, 0])

    dt = 0.01  # time step
    start_time = time.time()

    for step in range(500):
        current_time = time.time() - start_time

        # Switch mode example (if desired, based on time or condition)
        if current_time > 2.0 and dual_arm.mode != "synchronous":
            print("Switching to synchronous mode.")
            dual_arm.set_mode("synchronous")

        # Compute commands for both arms
        v_left, omega_left, v_right, omega_right = dual_arm.compute_commands(
            x_R_left, x_R_right,
            x_d_left, x_d_right,
            x_V_left, x_V_right,
            q_R_left, q_R_right,
            q_d_left, q_d_right,
            q_V_left, q_V_right,
            current_time)

        # Integrate the position commands (Euler integration for demonstration)
        x_R_left += dt * v_left
        x_R_right += dt * v_right

        # Integrate quaternion orientation using exponential map
        q_R_left = (Rotation.from_quat(q_R_left) * Rotation.from_rotvec(0.5 * dt * omega_left)).as_quat()
        q_R_right = (Rotation.from_quat(q_R_right) * Rotation.from_rotvec(0.5 * dt * omega_right)).as_quat()

        # Print results every 50 steps
        if step % 50 == 0:
            print(f"Time {current_time:.2f}s: "
                f"Left pos: {x_R_left}, Right pos: {x_R_right}, "
                f"Left quat: {q_R_left}, Right quat: {q_R_right}")

        time.sleep(dt)

