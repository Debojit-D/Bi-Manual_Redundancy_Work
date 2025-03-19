import numpy as np
import time
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation
from armds_rot_control import DualArmController  # Importing from your stored library

# --- Define Poses ---
# Pose A (Initial for Robot 1)
pose_A_pos = np.array([0.0, 0.0, 0.0])
pose_A_quat = np.array([0, 0, 0, 1])  # Identity quaternion

# Pose B (Initial for Robot 2)
pose_B_pos = np.array([1.0, 0.0, 0.0])
pose_B_quat = np.array([0, 0, 0, 1])  # Identity quaternion

# Pose C (Target for Robot 1 in Phase 1)
pose_C_pos = np.array([0.5, 0.5, 0.0])
pose_C_quat = np.array([0, 0, 0.7071, 0.7071])  # 90° rotation about Z

# Pose D (Target for Robot 2 in Phase 1)
pose_D_pos = np.array([1.5, 0.5, 0.0])
pose_D_quat = np.array([0, 0.7071, 0, 0.7071])  # 90° rotation about Y

# Pose E (Final target for both robots in Phase 2)
pose_E_pos = np.array([1.0, 1.0, 0.0])
pose_E_quat_left = np.array([0, 0, 0, 1])  # Robot 1 faces +X
pose_E_quat_right = np.array([0, 0, 1, 0])  # Robot 2 faces -X

# --- Phase Durations ---
phase1_duration = 5.0  # Move A → C and B → D asynchronously
phase2_duration = 10.0  # Move to E synchronously with opposite orientations
total_time = phase1_duration + phase2_duration

# --- Initial States ---
x_R_left = pose_A_pos.copy()
q_R_left = pose_A_quat.copy()
x_R_right = pose_B_pos.copy()
q_R_right = pose_B_quat.copy()

# --- Controller Gains ---
K_pos = 5.0
K_rot = 15.0

# Create the dual-arm controller
dual_arm = DualArmController(K_pos, K_rot, K_pos, K_rot)
dual_arm.set_mode("asynchronous")  # Start asynchronously

# --- Visualization Setup ---
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim(-0.5, 2.0)
ax.set_ylim(-0.5, 2.0)
ax.set_zlim(-0.5, 2.0)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title("Dual-Arm DS Control Simulation")

def get_forward(quat):
    """Returns the forward direction [1, 0, 0] transformed by the quaternion."""
    forward = np.array([1, 0, 0])
    rot = Rotation.from_quat(quat)
    return rot.apply(forward)

# Initialize visualization arrows
left_arrow = ax.quiver(x_R_left[0], x_R_left[1], x_R_left[2],
                       get_forward(q_R_left)[0], get_forward(q_R_left)[1], get_forward(q_R_left)[2],
                       color='blue', length=0.2, normalize=True)
right_arrow = ax.quiver(x_R_right[0], x_R_right[1], x_R_right[2],
                        get_forward(q_R_right)[0], get_forward(q_R_right)[1], get_forward(q_R_right)[2],
                        color='red', length=0.2, normalize=True)

plt.ion()
plt.show()

# --- Simulation Loop ---
dt = 0.01  # Time step
start_time = time.time()
current_sim_time = 0.0
previous_phase = None

while current_sim_time < total_time:
    current_sim_time = time.time() - start_time

    # --- Determine Current Phase ---
    if current_sim_time < phase1_duration:
        current_phase = "phase1"
    else:
        current_phase = "phase2"

    # --- Add a Wait Time Between Phases ---
    if current_phase != previous_phase:  # Detect phase transition
        print(f"Transitioning to {current_phase}... Waiting for 2 seconds.")
        time.sleep(2)  # Pause before next phase
        previous_phase = current_phase

    # --- Set Targets Based on Phase ---
    if current_phase == "phase1":
        # Move asynchronously: A → C (Robot 1) and B → D (Robot 2)
        x_d_left, x_V_left, q_d_left, q_V_left = pose_C_pos, pose_C_pos, pose_C_quat, pose_C_quat
        x_d_right, x_V_right, q_d_right, q_V_right = pose_D_pos, pose_D_pos, pose_D_quat, pose_D_quat
        dual_arm.set_mode("asynchronous")

    elif current_phase == "phase2":
        # Move synchronously: Both go to E, but face opposite directions
        x_d_left, x_V_left, q_d_left, q_V_left = pose_E_pos, pose_E_pos, pose_E_quat_left, pose_E_quat_left
        x_d_right, x_V_right, q_d_right, q_V_right = pose_E_pos, pose_E_pos, pose_E_quat_right, pose_E_quat_right
        dual_arm.set_mode("synchronous")

    # --- Compute DS Commands for Both Arms ---
    v_left, omega_left, v_right, omega_right = dual_arm.compute_commands(
        x_R_left, x_R_right,
        x_d_left, x_d_right,
        x_V_left, x_V_right,
        q_R_left, q_R_right,
        q_d_left, q_d_right,
        q_V_left, q_V_right,
        current_sim_time
    )

    # --- Integrate Commands ---
    x_R_left += dt * v_left
    x_R_right += dt * v_right

    # Update orientations using quaternion exponential map
    delta_q_left = Rotation.from_rotvec(0.5 * dt * omega_left).as_quat()
    delta_q_right = Rotation.from_rotvec(0.5 * dt * omega_right).as_quat()
    q_R_left = (Rotation.from_quat(q_R_left) * Rotation.from_quat(delta_q_left)).as_quat()
    q_R_right = (Rotation.from_quat(q_R_right) * Rotation.from_quat(delta_q_right)).as_quat()

    # --- Update 3D Visualization ---
    for coll in ax.collections:
        coll.remove()

    left_arrow = ax.quiver(x_R_left[0], x_R_left[1], x_R_left[2],
                           get_forward(q_R_left)[0], get_forward(q_R_left)[1], get_forward(q_R_left)[2],
                           color='blue', length=0.2, normalize=True)
    right_arrow = ax.quiver(x_R_right[0], x_R_right[1], x_R_right[2],
                            get_forward(q_R_right)[0], get_forward(q_R_right)[1], get_forward(q_R_right)[2],
                            color='red', length=0.2, normalize=True)

    plt.draw()
    plt.pause(0.001)

plt.ioff()
plt.show()
