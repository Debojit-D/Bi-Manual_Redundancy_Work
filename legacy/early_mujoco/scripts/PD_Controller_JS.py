from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

# Resolve the model relative to this script so the code is machine-independent.
xml_file_path = (
    Path(__file__).resolve().parent.parent
    / "robot_descriptions"
    / "heal"
    / "dual_heal_reconfigured_home.xml"
)

# Load the MuJoCo model
model = mujoco.MjModel.from_xml_path(xml_file_path.as_posix())
data = mujoco.MjData(model)

# Define the target positions for the joints
target_joint_position1 = np.zeros(model.nq)  # Target position 1
target_joint_position1[:6] = [-1.10, -1.169, 1.757, 1.54, -1.52, 0.00]  # Left arm
target_joint_position1[6:12] = [-2.00, -1.169, 1.757, 1.54, 1.60, 0.00]  # Right arm

target_joint_position2 = np.zeros(model.nq)  # Target position 2
target_joint_position2[:6] = [-1.54, -1.169, 1.757, 1.64, -1.52, 0.00]  # Left arm
target_joint_position2[6:12] = [-1.54, -1.169, 1.757, 1.54, 1.60, 0.00]  # Right arm

# PD controller gains
Kp = 1  # Proportional gain
Kd = 1.5  # Derivative gain

# Define a tolerance for stopping movement
tolerance = 1e-1

# State machine for tracking which target position to move to
current_target_idx = 0
target_positions = [target_joint_position1, target_joint_position2]

# Flag to indicate when all targets have been reached
all_targets_reached = False

# Compute PD torques
def compute_pd_torques(data, target_positions, Kp, Kd):
    """Calculate PD control torques for each joint."""
    pd_torques = np.zeros(model.nu)
    for i in range(model.nu):
        # Get joint position and velocity indices
        joint_id = model.actuator_trnid[i][0]
        qpos_idx = model.jnt_qposadr[joint_id]
        dof_idx = model.jnt_dofadr[joint_id]
        position_error = target_positions[qpos_idx] - data.qpos[qpos_idx]
        velocity_error = -data.qvel[dof_idx]
        pd_torques[i] = Kp * position_error + Kd * velocity_error
    return pd_torques

# Define the control callback function
def control_callback(model, data):
    global current_target_idx, all_targets_reached

    # If all targets are reached, apply gravity compensation only
    if all_targets_reached:
        actuator_joint_ids = model.actuator_trnid[:, 0]
        dof_indices = model.jnt_dofadr[actuator_joint_ids]
        data.ctrl[:] = data.qfrc_bias[dof_indices]  # Apply gravity compensation to actuated DOFs
        return

    # Get the current target position
    target_joint_positions = target_positions[current_target_idx]

    # Compute PD torques for moving to the target joint positions
    pd_torques = compute_pd_torques(data, target_joint_positions, Kp, Kd)

    # Get actuator joint IDs and DOF indices
    actuator_joint_ids = model.actuator_trnid[:, 0]
    dof_indices = model.jnt_dofadr[actuator_joint_ids]

    # Apply PD control torques combined with gravity compensation
    data.ctrl[:] = data.qfrc_bias[dof_indices] + pd_torques

    # Calculate position error over all actuated joints. A zero-valued target is
    # still an active target and must not be omitted from the convergence check.
    actuator_joint_ids = model.actuator_trnid[:, 0]
    qpos_indices = model.jnt_qposadr[actuator_joint_ids]
    position_error = np.linalg.norm(
        target_joint_positions[qpos_indices] - data.qpos[qpos_indices]
    )

    if position_error < tolerance:
        print(f"Target position {current_target_idx + 1} reached.")

        # Move to the next target position, if available
        current_target_idx += 1
        if current_target_idx >= len(target_positions):
            print("All target positions reached. Holding position.")
            # Set the flag to apply gravity compensation only
            all_targets_reached = True
        else:
            print(f"Moving to target position {current_target_idx + 1}.")

def main():
    """Run the PD-controlled simulation in MuJoCo's passive viewer."""
    mujoco.set_mjcb_control(control_callback)
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
    finally:
        # Avoid leaving a process-global callback installed during interpreter
        # shutdown or when this module is reused.
        mujoco.set_mjcb_control(None)


if __name__ == "__main__":
    main()
