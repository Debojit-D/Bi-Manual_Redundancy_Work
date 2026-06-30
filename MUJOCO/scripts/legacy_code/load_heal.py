import mujoco
import mujoco.viewer
import sys

# Path to your MuJoCo XML file
XML_PATH = "/home/iitgn-robotics-1/Debojit_WS/Bi-Manual_Redundancy_Work/MUJOCO/robot_descriptions/heal/dual_heal_reconfigured_home.xml"  # Update this with your actual XML file path

# Load the model
try:
    model = mujoco.MjModel.from_xml_path(XML_PATH)
    data = mujoco.MjData(model)
except Exception as e:
    print(f"Error loading the MuJoCo XML file: {e}")
    sys.exit(1)

# Start the simulation viewer
with mujoco.viewer.launch_passive(model, data) as viewer:
    print("MuJoCo simulation loaded successfully. Close the viewer to exit.")
    
    while viewer.is_running():  # Keep running until manually closed
        mujoco.mj_step(model, data)  # Step the simulation
        viewer.sync()  # Render and update the viewer

