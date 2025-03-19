from pathlib import Path
import time
import numpy as np
from scipy.spatial.transform import Rotation

# Import your libraries.
from dual_arm_control import DualArmCollisionController
from armds_rot_control import DualArmController

# --- Setup Paths ---
_HERE = Path(__file__).parent
_XML = _HERE / "/home/barat/Debojit_WS/Bi-Manual_Redundancy_Work/MUJOCO/robot_descriptions/franka_emika_panda/dual_panda_scene.xml"


# Initialize the collision controller (which resets the robot to home1 and aligns the mocap targets)
collision_controller = DualArmCollisionController(model_path=_XML.as_posix())

# Print the initial mocap positions (should reflect the end-effector poses at home1)
# print(f"Init_pose_left: {collision_controller.initial_target_position_left}")
# print(f"Init_pose_right: {collision_controller.initial_target_position_right}")

# Then proceed with your DS control loop...

# # Reset mocap bodies before simulation starts
# collision_controller.reset_mocap_targets()

# # Update the stored initial positions after resetting the mocap targets.
# collision_controller.initial_target_position_left = collision_controller.data.mocap_pos[0].copy()
# collision_controller.initial_target_position_right = collision_controller.data.mocap_pos[1].copy()

# print(f"Init_pose_left: {collision_controller.initial_target_position_left}")


# (This loads the model, sets up tasks & collision limits, and initializes mocap targets.)
init_pos_left = collision_controller.initial_target_position_left.copy()
init_pos_right = collision_controller.initial_target_position_right.copy()
init_quat_left = collision_controller.data.mocap_quat[0].copy()
init_quat_right = collision_controller.data.mocap_quat[1].copy()

# --- Initialize the DS Controller for Dual-Arm ---
ds_controller = DualArmController(K_pos_left=2.0, K_rot_left=2.0,
                                  K_pos_right=20.0, K_rot_right=2.0)
ds_controller.set_mode("asynchronous")

print(f"Init_pose_left: {init_pos_left}")


# --- Define Asynchronous Target Poses ---
# Pose A for left arm, Pose B for right arm (relative to initial positions).
pose_A_position = init_pos_left + np.array([-0.1, -0.2, 0.1])
pose_B_position = init_pos_right + np.array([-0.1, 0.2, 0.1])
pose_A_orientation = np.array([1, 0, 0, 0])
pose_B_orientation = np.array([1, 0, 0, 0])

# For DS control, define asynchronous targets (x_d) and synchronous targets (x_V).
x_d_left = pose_A_position.copy()
x_d_right = pose_B_position.copy()
x_V_left = init_pos_left.copy()
x_V_right = init_pos_right.copy()
q_d_left = pose_A_orientation.copy()
q_d_right = pose_B_orientation.copy()
q_V_left = init_quat_left.copy()
q_V_right = init_quat_right.copy()

# Initialize DS state (start from current simulation state).
ds_state_pos_left = init_pos_left.copy()
ds_state_pos_right = init_pos_right.copy()
ds_state_quat_left = init_quat_left.copy()
ds_state_quat_right = init_quat_right.copy()

dt = 0.005
total_time = 30.0
start_time = time.time()

print("Starting dual-arm DS control with collision avoidance...")

# print("Available Mocap Bodies:", [collision_controller.model.body(i).name for i in range(collision_controller.model.nbody)])

# Instead of calling collision_controller.run() directly,
# we open the viewer and embed our DS control update in its simulation loop.
try:
    from mujoco.viewer import launch_passive
    with launch_passive(model=collision_controller.model, data=collision_controller.data) as viewer:
        while viewer.is_running() and (time.time() - start_time < total_time):
            current_time = time.time() - start_time

            # --- DS Control Update ---
            # Compute DS commands for both arms.
            v_left, omega_left, v_right, omega_right = ds_controller.compute_commands(
                ds_state_pos_left, ds_state_pos_right,
                x_d_left, x_d_right,
                x_V_left, x_V_right,
                ds_state_quat_left, ds_state_quat_right,
                q_d_left, q_d_right,
                q_V_left, q_V_right,
                current_time
            )

            print(f"Step {current_time:.2f}s | v_left: {v_left}, omega_left: {omega_left}, v_right: {v_right}, omega_right: {omega_right}")
            # print("External force on left arm:", collision_controller.data.cfrc_ext[:7])
            # print("External force on right arm:", collision_controller.data.cfrc_ext[7:14])
            # print(f"Step {current_time:.2f}s | Mocap Left Pos: {collision_controller.data.mocap_pos[0]}, Mocap Right Pos: {collision_controller.data.mocap_pos[1]}")

            # Integrate DS commands (Euler integration).
            ds_state_pos_left += dt * v_left
            ds_state_pos_right += dt * v_right
            ds_state_quat_left = (Rotation.from_quat(ds_state_quat_left) *
                                  Rotation.from_rotvec(0.5 * dt * omega_left)).as_quat()
            ds_state_quat_right = (Rotation.from_quat(ds_state_quat_right) *
                                   Rotation.from_rotvec(0.5 * dt * omega_right)).as_quat()

            # Form new target poses for the collision controller.
            target_left = {
                "position": ds_state_pos_left,
                "orientation": ds_state_quat_left
            }
            target_right = {
                "position": ds_state_pos_right,
                "orientation": ds_state_quat_right
            }

            print(f"🔄 Mocap Pos Left (before update): {collision_controller.data.mocap_pos[0]}")
            print(f"🔄 Mocap Pos Right (before update): {collision_controller.data.mocap_pos[1]}")

            # Apply DS control update
            collision_controller.data.mocap_pos[0] = ds_state_pos_left.copy()
            collision_controller.data.mocap_pos[1] = ds_state_pos_right.copy()
            collision_controller.data.mocap_quat[0] = ds_state_quat_left.copy()
            collision_controller.data.mocap_quat[1] = ds_state_quat_right.copy()

            print(f"✅ Mocap Pos Left (after update): {collision_controller.data.mocap_pos[0]}")
            print(f"✅ Mocap Pos Right (after update): {collision_controller.data.mocap_pos[1]}")

            
            
            # Compute the error
            error_left = ds_state_pos_left - x_d_left
            error_right = ds_state_pos_right - x_d_right
            error_rot_left = Rotation.from_quat(ds_state_quat_left).inv() * Rotation.from_quat(q_d_left)
            error_rot_right = Rotation.from_quat(ds_state_quat_right).inv() * Rotation.from_quat(q_d_right)

            # print(f"🛑 Error Left Pos: {error_left}, Rot: {error_rot_left.as_rotvec()}")
            # print(f"🛑 Error Right Pos: {error_right}, Rot: {error_rot_right.as_rotvec()}")

            # print(f"FORCED Mocap Left: {collision_controller.data.mocap_pos[0]}")
            # print(f"FORCED Mocap Right: {collision_controller.data.mocap_pos[1]}")

            # print(f"DS State Left Pos: {ds_state_pos_left}, Target: {x_d_left}")
            # print(f"DS State Right Pos: {ds_state_pos_right}, Target: {x_d_right}")
            # print(f"💡 Mocap Left Pos (before update): {collision_controller.data.mocap_pos[0]}")
            # print(f"💡 Mocap Right Pos (before update): {collision_controller.data.mocap_pos[1]}")

            collision_controller.update_targets(target_left, target_right)

            # --- Step the collision avoidance simulation (includes IK solving) ---
            collision_controller.step_simulation(dt)

            viewer.sync()
            time.sleep(dt)
except (AttributeError, ImportError):
    print("Viewer not available, running headless simulation.")
    while time.time() - start_time < total_time:
        # (You can replicate the above loop without viewer.sync() in headless mode.)
        current_time = time.time() - start_time

        v_left, omega_left, v_right, omega_right = ds_controller.compute_commands(
            ds_state_pos_left, ds_state_pos_right,
            x_d_left, x_d_right,
            x_V_left, x_V_right,
            ds_state_quat_left, ds_state_quat_right,
            q_d_left, q_d_right,
            q_V_left, q_V_right,
            current_time
        )
        
        ds_state_pos_left += dt * v_left
        ds_state_pos_right += dt * v_right
        ds_state_quat_left = (Rotation.from_quat(ds_state_quat_left) *
                              Rotation.from_rotvec(0.5 * dt * omega_left)).as_quat()
        ds_state_quat_right = (Rotation.from_quat(ds_state_quat_right) *
                               Rotation.from_rotvec(0.5 * dt * omega_right)).as_quat()

        target_left = {
            "position": ds_state_pos_left,
            "orientation": ds_state_quat_left
        }
        target_right = {
            "position": ds_state_pos_right,
            "orientation": ds_state_quat_right
        }
        collision_controller.update_targets(target_left, target_right)
        collision_controller.step_simulation(dt)
        time.sleep(dt)
        

print("Simulation complete.")
