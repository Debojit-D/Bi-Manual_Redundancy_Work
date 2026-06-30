import os
import numpy as np
import mujoco
import mujoco.viewer
import trajecotory_generation


# Path to your dual-panda MuJoCo XML
MODEL_PATH = os.path.join(
    os.path.dirname(__file__),
    "../robot_descriptions/franka_emika_panda/dual_panda_scene.xml"
)

# -- Load the model and data --
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"Model file not found: {MODEL_PATH}")
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# If the model has keyframes, optionally apply a "home" keyframe
if model.nkey > 0:
    keyframe_id = 0
    mujoco.mj_resetDataKeyframe(model, data, keyframe_id)
    print(f"Applied keyframe {keyframe_id} (e.g. 'home1')")
else:
    print("⚠ No keyframes found in the model!")

# -------------
# Find the free joint ID for your table (you must know the actual joint name)
# -------------
table_joint_name = "table_joint"  # Example name; adjust to match your model
jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, table_joint_name)
if jnt_id < 0:
    raise ValueError(f"Joint '{table_joint_name}' not found in model!")
jnt_adr = model.jnt_qposadr[jnt_id]

# MuJoCo expects: qpos for free joint = [x, y, z, qw, qx, qy, qz]
initial_pos_mj = data.qpos[jnt_adr : jnt_adr + 3]
initial_quat_mj = data.qpos[jnt_adr + 3 : jnt_adr + 7]  # [qw, qx, qy, qz]

# Convert MuJoCo's [qw,qx,qy,qz] to the more common [x,y,z,w] if your
# generate_smooth_quintic_trajectory expects [x, y, z, w]. 
# If your function expects the scalar-last format, let's reorder:
qw, qx, qy, qz = initial_quat_mj
initial_quat = np.array([qx, qy, qz, qw])  # (x, y, z, w)

# Define final position (move +0.2m in the z direction)
final_pos = np.array(initial_pos_mj)
final_pos[2] += 0.2

# Keep orientation the same for demonstration
final_quat = np.array([qx, qy, qz, qw])  # no change

# Generate the trajectory
pos_traj, quat_traj, lin_vel, lin_acc, ang_vel, t_vals = \
    trajecotory_generation.generate_smooth_quintic_trajectory(
        initial_position=initial_pos_mj,  # same as above
        final_position=final_pos,
        initial_quat=initial_quat,
        final_quat=final_quat,
        total_time=2.0,
        time_step=0.001
    )

# -------------
# Launch the viewer and execute the motion
# -------------
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Press 'ESC' in the viewer to quit.")
    
    # Step through the trajectory
    for i, t in enumerate(t_vals):
        # If the viewer was closed, stop early
        if not viewer.is_running():
            break

        # 1) Write the position into data.qpos
        data.qpos[jnt_adr : jnt_adr + 3] = pos_traj[i]

        # 2) Write the orientation (remember to reorder to [qw,qx,qy,qz] for MuJoCo)
        x, y, z, w = quat_traj[i]  # from function
        data.qpos[jnt_adr + 3 : jnt_adr + 7] = [w, x, y, z]

        # 3) Step physics and update viewer
        mujoco.mj_step(model, data)
        viewer.sync()

print("Done moving the table!")
