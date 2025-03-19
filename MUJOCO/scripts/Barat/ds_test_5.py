import numpy as np
import time
import mujoco
import mujoco.viewer
import mink
from scipy.spatial.transform import Rotation
from armds_rot_control import DSController  # Import DS Controller

# --- Load MuJoCo Model ---
xml_path = "/home/barat/snap/mujoco_simu/robot_descriptions/franka_emika_panda/mjx_scene.xml"
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Reset to a known valid configuration
if model.key("home1") is not None:
    mujoco.mj_resetDataKeyframe(model, data, model.key("home1").id)
else:
    print("[WARNING] No home keyframe found; please ensure a valid initial pose.")

# --- Update Mink's configuration with the valid qpos ---
configuration = mink.Configuration(model)
configuration.update(data.qpos)

# --- Find End-Effector and Mocap Body IDs ---
end_effector_name = "left_finger"
mocap_body_name = "target"

end_effector_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, end_effector_name)
mocap_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, mocap_body_name)

if end_effector_id == -1:
    raise ValueError(f"[ERROR] End-effector '{end_effector_name}' not found in model.")

if mocap_body_id == -1:
    raise ValueError(f"[ERROR] Mocap body '{mocap_body_name}' not found in model.")

# --- Extract Initial Pose ---
mujoco.mj_forward(model, data)
initial_position = data.xpos[end_effector_id].copy()
initial_quat = data.xquat[end_effector_id].copy()  # SciPy format [x, y, z, w]

# Convert quaternion to Mink's format [w, x, y, z]
initial_quat = np.roll(initial_quat, shift=1)

# --- Define Target Pose Relative to Initial Pose ---
relative_translation = np.array([0.1, 0.1, 0.0])  # Move 0.1m in X & Y
relative_rotation = Rotation.from_euler('z', 0, degrees=True).as_quat()
relative_rotation = np.roll(relative_rotation, shift=1)  # Convert to [w, x, y, z]

target_position = initial_position + relative_translation
target_quat = (Rotation.from_quat(np.roll(initial_quat, -1)) * Rotation.from_quat(np.roll(relative_rotation, -1))).as_quat()
target_quat = np.roll(target_quat, shift=1)  # Convert back to [w, x, y, z]

print(f"[DEBUG] Computed Target Pose: Position = {target_position}, Quaternion = {target_quat}")

# --- Simulation Parameters ---
total_time = 5.0  # Move in 5 seconds
dt = 0.01  # 10ms step

# --- Initialize DS Controller ---
K_pos, K_rot = 2.0, 2.0
ds_controller = DSController(K_pos, K_rot)

# --- Initialize Current Pose ---
x_R = initial_position.copy()
q_R = initial_quat.copy()

# --- Initialize Mink for Inverse Kinematics on the Mocap Body ---
ik_task = mink.FrameTask(mocap_body_name, "body", position_cost=1.0, orientation_cost=1.0, lm_damping=1.0)
tasks = [ik_task]
solver = "osqp"

def set_manual_target_pose(data, model, target_position, target_quaternion):
    """
    Manually sets the 'target' mocap body's position and orientation.

    Args:
        data (mujoco.MjData): MuJoCo simulation data object.
        model (mujoco.MjModel): MuJoCo model object.
        target_position (np.array): Desired target XYZ position [x, y, z].
        target_quaternion (np.array): Desired target orientation as a quaternion [w, x, y, z].
    """

    # Get mocap ID for the 'target' body
    mocap_body_id = model.body("target").mocapid

    if mocap_body_id == -1:
        raise ValueError("[ERROR] Mocap body 'target' not assigned a valid mocap ID!")

    # Set the target position and orientation
    data.mocap_pos[mocap_body_id] = target_position
    data.mocap_quat[mocap_body_id] = target_quaternion
    
    # Force an update to apply the changes
    mujoco.mj_forward(model, data)

    # Debugging Information
    print(f"[DEBUG] Manually Set 'target' Position: {data.mocap_pos[mocap_body_id]}")
    print(f"[DEBUG] Manually Set 'target' Quaternion: {data.mocap_quat[mocap_body_id]}")


def update_mocap_target(data, model, target_pos, target_quat):
    """Moves the mocap body to the target pose for IK"""
    mocap_id = model.body(mocap_body_name).mocapid  # Correct ID for mocap bodies

    if mocap_id == -1:
        raise ValueError(f"[ERROR] Mocap body '{mocap_body_name}' not assigned a mocap ID.")

    data.mocap_pos[mocap_id] = target_pos
    data.mocap_quat[mocap_id] = target_quat


# --- Simulation Loop ---
start_time = time.time()
current_time = 0

# Set target for arms to reach
manual_target_position = np.array([0.5, 0.0, 0.5])  # Set desired XYZ position
manual_target_quaternion = np.array([1.0, 0.0, 0.0, 0.0])  # Identity quaternion [w, x, y, z]

set_manual_target_pose(data, model, manual_target_position, manual_target_quaternion)



with mujoco.viewer.launch_passive(model, data) as viewer:
    while current_time < total_time:
        current_time = time.time() - start_time
        tau = current_time / total_time  # Linear interpolation for blending

        # --- Compute DS-based Velocity Commands ---
        v_command, omega_command = ds_controller.compute_command(
            x_R, x_R, target_position,  # Current & target position
            q_R, q_R, target_quat,  # Current & target quaternion
            tau, 0.0  # No time derivative
        )

        # --- Euler Integration for Position & Orientation ---
        x_R += v_command * dt
        q_R = (Rotation.from_quat(np.roll(q_R, -1)) * Rotation.from_rotvec(0.5 * dt * omega_command)).as_quat()
        q_R = np.roll(q_R, shift=1)  # Convert back to [w, x, y, z]

        # 🔍 Debug: Print updated position and quaternion
        print(f"[DEBUG] Updated Pose: Position = {x_R}, Quaternion = {q_R}")

        # --- Move the Mocap Body to this Pose for IK ---
        update_mocap_target(data, model, x_R, q_R)

        # --- Compute IK for Joint Positions ---
        T_target = mink.SE3(q_R, x_R)  # Define target pose
        ik_task.set_target(T_target)  # Update IK target

        # 🔍 Debug: Print the target pose before solving IK
        print(f"[DEBUG] Target Pose for IK: Position = {x_R}, Quaternion = {q_R}")

        try:
            # Solve IK to get joint positions (FIXED FUNCTION CALL)
            try:
                joint_positions = mink.solve_ik(configuration, tasks,dt, solver=solver, damping=5e-3)
            except Exception as e:
                print(f"[ERROR] Mink IK Solver Failed: {e}")
                break

            configuration.update(joint_positions)  # Ensure the configuration updates properly
        except Exception as e:
            print(f"[ERROR] Mink IK Solver Failed: {e}")
            break

        # --- Apply Joint Positions to MuJoCo Model ---
        data.qpos[:] = joint_positions

        # --- Step MuJoCo Simulation ---
        mujoco.mj_step(model, data)
        viewer.sync()

        time.sleep(dt)

print("Motion completed successfully!")
