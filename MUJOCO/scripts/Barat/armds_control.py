"""
armds_control.py

A library for DS-based control of robotic arms supporting both synchronous and asynchronous modes,
with optional dynamic blending for smooth switching. Suitable for bimanual tasks such as snap fitting.
Library Name: armds_control (Arm Dynamical System Control)
"""

import numpy as np
import math

def logistic_tau(t, t0, k):
    """
    Compute the logistic blending parameter tau and its derivative.
    
    The logistic function is given by:
        tau(t) = 1 / (1 + exp(-k*(t - t0)))
    Its time derivative is:
        d_tau(t) = k * exp(-k*(t - t0)) / (1 + exp(-k*(t - t0)))^2

    Parameters:
        t (float): Current time.
        t0 (float): Midpoint of the blend (transition center).
        k (float): Steepness parameter.
    
    Returns:
        tuple: (tau, d_tau)
    """
    tau = 1.0 / (1.0 + np.exp(-k * (t - t0)))
    d_tau = k * np.exp(-k * (t - t0)) / ((1.0 + np.exp(-k * (t - t0)))**2)
    return tau, d_tau

class DSController:
    """
    Dynamical System (DS) Controller for a single robotic arm.
    
    The control law is derived as follows:
    
      Define:
        - x_R: current state (e.g., end-effector position, in R^n)
        - x_d: independent (asynchronous) target state.
        - x_V: synchronous (virtual) target state.
        - tau: blending parameter (scalar in [0,1]) such that:
             tau = 0   -> fully asynchronous (track x_d)
             tau = 1   -> fully synchronous (track x_V)
        - The blended target is:
             x_target = tau*x_V + (1-tau)*x_d.
      
      We desire to have the error e = x_R - x_target converge to zero.
      A DS of the form:
      
          dot{x_R} = tau * dot{x_V} + dot{tau}*(x_V - x_d) - K*(x_R - x_target)
      
      is proposed. (Here, we assume dot{x_V} is provided externally; if unknown, it is taken as zero.)
      
      Then the error dynamics become:
      
          dot{e} = dot{x_R} - dot{x_target} 
                 = -K * e.
      
      With K positive definite, e converges to zero exponentially.
    
    Attributes:
        K (float or np.ndarray): Gain (positive scalar or matrix).
    """
    def __init__(self, K):
        self.K = K  # Gain parameter; must be positive.
    
    def compute_command(self, x_R, x_d, x_V, tau, d_tau, dot_x_V=None):
        """
        Compute the DS-based control command (desired velocity) for the arm.
        
        Equation:
          v = tau * dot{x_V} + d_tau*(x_V - x_d) - K*(x_R - (tau*x_V + (1-tau)*x_d))
        
        Parameters:
            x_R (np.array): Current state (position) of the arm.
            x_d (np.array): Asynchronous target state.
            x_V (np.array): Synchronous (virtual) target state.
            tau (float): Blending parameter (0 for async, 1 for sync).
            d_tau (float): Time derivative of tau.
            dot_x_V (np.array, optional): Time derivative of x_V. Defaults to zero.
        
        Returns:
            np.array: Desired velocity command.
        """
        if dot_x_V is None:
            dot_x_V = np.zeros_like(x_V)
        # Blended target:
        x_target = tau * x_V + (1 - tau) * x_d
        # DS command:
        command = tau * dot_x_V + d_tau * (x_V - x_d) - self.K * (x_R - x_target)
        return command

class DualArmController:
    """
    DualArmController manages two DSControllers (one per arm) to generate 
    commands for a dual-arm system in both synchronous and asynchronous modes.
    
    It supports:
      - Synchronous mode: Both arms track a common virtual target.
      - Asynchronous mode: Each arm tracks its own independent target.
      - Blending: A dynamic blending mode that smoothly interpolates between modes.
      
    Mode switching can be done via a simple state-machine (discrete switch)
    or by enabling dynamic blending.
    """
    def __init__(self, K_left, K_right):
        self.left_controller = DSController(K_left)
        self.right_controller = DSController(K_right)
        # Default control mode: asynchronous.
        self.mode = "asynchronous"  # Options: "synchronous", "asynchronous", "blending"
        self.blend_start_time = None
        self.blend_duration = 1.0  # Duration for blending transition (seconds).
        self.blend_k = 10.0      # Steepness parameter for logistic blending.
    
    def set_mode(self, mode, current_time=None, blend_duration=None):
        """
        Set the control mode.
        
        Parameters:
            mode (str): "synchronous", "asynchronous", or "blending".
            current_time (float, optional): Current time (for initiating blend).
            blend_duration (float, optional): Blending duration (if mode is "blending").
        """
        self.mode = mode
        if mode == "blending" and current_time is not None:
            self.blend_start_time = current_time
            if blend_duration is not None:
                self.blend_duration = blend_duration
    
    def get_tau(self, current_time):
        """
        Determine the blending parameter tau and its derivative based on the current mode.
        
        Returns:
            tuple: (tau, d_tau)
        """
        if self.mode == "synchronous":
            return 1.0, 0.0
        elif self.mode == "asynchronous":
            return 0.0, 0.0
        elif self.mode == "blending":
            if self.blend_start_time is None:
                self.blend_start_time = current_time
            # Logistic blending: midpoint t0 and steepness self.blend_k.
            t0 = self.blend_start_time + self.blend_duration / 2.0
            tau, d_tau = logistic_tau(current_time, t0, self.blend_k)
            return tau, d_tau
        else:
            return 0.0, 0.0
    
    def compute_commands(self, x_R_left, x_R_right, x_d_left, x_d_right, x_V_left, x_V_right, current_time, dot_x_V_left=None, dot_x_V_right=None):
        """
        Compute the control commands for both arms based on the current state and mode.
        
        Parameters:
            x_R_left (np.array): Current state of left arm.
            x_R_right (np.array): Current state of right arm.
            x_d_left (np.array): Asynchronous target for left arm.
            x_d_right (np.array): Asynchronous target for right arm.
            x_V_left (np.array): Synchronous target for left arm.
            x_V_right (np.array): Synchronous target for right arm.
            current_time (float): Current time.
            dot_x_V_left (np.array, optional): Time derivative of left synchronous target.
            dot_x_V_right (np.array, optional): Time derivative of right synchronous target.
        
        Returns:
            tuple: (v_left, v_right) velocity commands.
        """
        tau, d_tau = self.get_tau(current_time)
        v_left = self.left_controller.compute_command(x_R_left, x_d_left, x_V_left, tau, d_tau, dot_x_V_left)
        v_right = self.right_controller.compute_command(x_R_right, x_d_right, x_V_right, tau, d_tau, dot_x_V_right)
        return v_left, v_right

# --- Example: Using the Library in a Simulation Loop ---
if __name__ == "__main__":
    # This example simulates two arms in a simple loop.
    # In your actual application, integrate this with MuJoCo and Mink's IK.
    
    import time
    
    # Initialize DualArmController with gain parameters.
    dual_arm = DualArmController(K_left=2.0, K_right=2.0)
    
    # Initial mode is asynchronous.
    dual_arm.set_mode("asynchronous")
    
    # Dummy initial states for left and right arms (positions in R^3)
    x_R_left = np.array([0.0, 0.0, 0.0])
    x_R_right = np.array([0.5, 0.0, 0.0])
    
    # Asynchronous targets for individual tasks.
    x_d_left = np.array([0.1, 0.2, 0.3])
    x_d_right = np.array([0.6, 0.2, 0.3])
    
    # Synchronous (virtual) targets for coordinated tasks.
    x_V_left = np.array([0.2, 0.3, 0.4])
    x_V_right = np.array([0.7, 0.3, 0.4])
    
    dt = 0.01  # time step (s)
    start_time = time.time()
    
    # Simulation loop for demonstration (replace with your MuJoCo loop)
    for step in range(500):
        current_time = time.time() - start_time
        
        # Example: switch mode at 2 seconds (discrete switch)
        if current_time > 2.0 and dual_arm.mode != "synchronous":
            print("Switching to synchronous mode (discrete).")
            dual_arm.set_mode("synchronous")
        # Alternatively, to test blending, you can use:
        # if current_time > 2.0 and dual_arm.mode != "blending":
        #     print("Switching to blending mode.")
        #     dual_arm.set_mode("blending", current_time, blend_duration=1.0)
        
        # Compute DS commands for both arms.
        v_left, v_right = dual_arm.compute_commands(x_R_left, x_R_right,
                                                     x_d_left, x_d_right,
                                                     x_V_left, x_V_right,
                                                     current_time)
        # Integrate the DS commands (Euler integration for demonstration)
        x_R_left = x_R_left + dt * v_left
        x_R_right = x_R_right + dt * v_right
        
        # Log states every 50 steps.
        if step % 50 == 0:
            print(f"Time {current_time:.2f}s: Left pos: {x_R_left}, Right pos: {x_R_right}")
        
        time.sleep(dt)
