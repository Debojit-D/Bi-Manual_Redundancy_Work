import mujoco
import mujoco.viewer
import os
import numpy as np
from loop_rate_limiters import RateLimiter
import time
from armds_control import DualArmController

MODEL_PATH = "/home/barat/snap/mujoco_simu/robot_descriptions/franka_emika_panda/dual_panda_scene.xml"

def initialize_model():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    return model, data

def define_waypoints(data, site_ref_left, site_ref_right):
    target_position_left_A = np.copy(data.site_xpos[site_ref_left]) + np.array([0, 0.2, 0.0])
    target_position_right_B = np.copy(data.site_xpos[site_ref_right]) + np.array([0.2, 0, 0.0])

    target_position_left_C = target_position_left_A + np.array([0.0, 0.3, 0.0])
    target_position_right_D = (target_position_left_C + target_position_right_B) / 2  

    return target_position_left_A, target_position_right_B, target_position_left_C, target_position_right_D

def has_reached_target(current_pos, target_pos, threshold=0.005):
    """
    Check if the current position is within a small threshold of the target position.
    """
    return np.linalg.norm(current_pos - target_pos) < threshold

def update_ds_control(dual_arm, left_pos, right_pos, x_A, x_B, x_C, x_D):
    """
    Dynamically update control mode based on task completion.
    """
    mode = dual_arm.mode

    if mode == "asynchronous":
        target_left, target_right = x_A, x_B
        if has_reached_target(left_pos, x_A) and has_reached_target(right_pos, x_B):
            dual_arm.set_mode("asynchronous")
            return x_C, x_B  # Left moves to C, right stays at B

    elif mode == "asynchronous" and has_reached_target(left_pos, x_C):
        dual_arm.set_mode("synchronous")
        return x_D, x_D  # Both arms move together

    elif mode == "synchronous" and has_reached_target(left_pos, x_D) and has_reached_target(right_pos, x_D):
        dual_arm.set_mode("blending")
        return left_pos, right_pos

    return target_left, target_right

def print_simulation_status(current_time, x_R_left, x_R_right, target_left, target_right, mode):
    """
    Prints the simulation status.
    """
    print(f"\n[Time: {current_time:.2f}s] Controller Mode: {mode}")
    print(f"Left Arm -> Current: {x_R_left}, Commanded: {target_left}")
    print(f"Right Arm -> Current: {x_R_right}, Commanded: {target_right}")

if __name__ == "__main__":
    model, data = initialize_model()
    
    site_ref_left, site_ref_right = model.site("attachment_site_left").id, model.site("attachment_site_right").id
    x_A, x_B, x_C, x_D = define_waypoints(data, site_ref_left, site_ref_right)
    
    x_R_left = np.copy(data.site_xpos[site_ref_left])
    x_R_right = np.copy(data.site_xpos[site_ref_right])

    dual_arm = DualArmController(K_left=2.0, K_right=2.0)

    with mujoco.viewer.launch_passive(model=model, data=data, show_left_ui=False, show_right_ui=False) as viewer:
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)
        mujoco.mj_resetDataKeyframe(model, data, model.key("home1").id)
        mujoco.mj_forward(model, data)
        
        rate = RateLimiter(frequency=40.0, warn=False)
        start_time = time.time()
        
        while viewer.is_running():
            current_time = time.time() - start_time

            # Step the simulation
            mujoco.mj_step(model, data)
            
            # Force update of site positions
            mujoco.mj_forward(model, data)
            x_R_left = np.copy(data.site_xpos[site_ref_left])
            x_R_right = np.copy(data.site_xpos[site_ref_right])
            print(x_R_left)


            # Get DS control targets
            target_left, target_right = update_ds_control(dual_arm, x_R_left, x_R_right, x_A, x_B, x_C, x_D)
            print_simulation_status(current_time, x_R_left, x_R_right, x_A, target_right, dual_arm.mode)

            # Compute end-effector velocity commands
            v_ee_left, v_ee_right = dual_arm.compute_commands(x_R_left, x_R_right, target_left, target_right, x_C, x_D, current_time)

            # Compute Jacobians using Mujoco
            Jp_left = np.zeros((3, model.nv))
            Jr_left = np.zeros((3, model.nv))
            Jp_right = np.zeros((3, model.nv))
            Jr_right = np.zeros((3, model.nv))

            mujoco.mj_jacBody(model, data, Jp_left, Jr_left, model.body("hand_l").id)
            mujoco.mj_jacBody(model, data, Jp_right, Jr_right, model.body("hand_r").id)

            J_left = np.vstack((Jp_left[:, :7], Jr_left[:, :7]))  # 6x7 Jacobian for left arm
            J_right = np.vstack((Jp_right[:, 7:14], Jr_right[:, 7:14]))  # 6x7 Jacobian for right arm

            # Solve for joint velocities using the pseudo-inverse
            q_dot_left = np.linalg.pinv(J_left) @ np.hstack([v_ee_left, np.zeros(3)])  
            q_dot_right = np.linalg.pinv(J_right) @ np.hstack([v_ee_right, np.zeros(3)])

            # print(q_dot_left)
            # Apply joint velocity commands
            data.ctrl[0:7] = q_dot_left[:7]  
            data.ctrl[7:14] = q_dot_right[:7]  

            mujoco.mj_step(model, data)
            viewer.sync()
            rate.sleep()
