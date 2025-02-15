import mujoco
import mujoco.viewer
import os

# Define the path to the MuJoCo XML model
MODEL_PATH = os.path.join(
    os.path.dirname(__file__), 
    "../robot_descriptions/franka_emika_panda/dual_panda_scene.xml"
)

# Load the MuJoCo model
if not os.path.exists(MODEL_PATH):
    raise FileNotFoundError(f"Model file not found: {MODEL_PATH}")

# Load the model and create a data object
model = mujoco.MjModel.from_xml_path(MODEL_PATH)
data = mujoco.MjData(model)

# Check if keyframes exist
if model.nkey > 0:
    keyframe_id = 0  # Assuming "home1" is the first keyframe
    mujoco.mj_resetDataKeyframe(model, data, keyframe_id)
    print(f"Applied keyframe {keyframe_id} ('home1')")
else:
    print("⚠ No keyframes found in the model!")

# Launch the MuJoCo viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("Press 'ESC' to close the viewer.")
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()
