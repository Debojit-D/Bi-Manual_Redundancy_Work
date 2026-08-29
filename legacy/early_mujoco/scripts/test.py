import mujoco
import mujoco.viewer
import os
import numpy as np

MODEL_PATH = os.path.join(
    os.path.dirname(__file__), 
    "../robot_descriptions/franka_emika_panda/dual_panda_scene.xml"
)

if __name__ == "__main__":
    # Load the MuJoCo model and data
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    # Reset the model to a predefined home pose (if home pose exists in the XML)
    mujoco.mj_resetDataKeyframe(model, data, 0)  # Assuming the home pose is the first keyframe in the XML

    # Initialize the viewer (no simulation or control, just load the model)
    with mujoco.viewer.launch_passive(model=model, data=data) as viewer:
        # Set default camera view
        mujoco.mjv_defaultFreeCamera(model, viewer.cam)

        # Time-related variables
        t = 0.0  # Time in seconds
        frequency = 0.01  # Frequency of the sine wave (1 Hz for 1 cycle per second)
        amplitude = -0.8  # Amplitude of the sine wave

        # Run the viewer loop
        while viewer.is_running():
            # Apply a sine wave motion to the control input (base control input as an example)
            data.ctrl[0:2] = amplitude * np.sin(2 * np.pi * frequency * t)  # Sine wave for joint 0

            # Increment time (adjust to control the speed of the sine wave)
            t += 0.01  # Step size for time increment (can adjust to modify the speed of motion)

            # Step the simulation (no change in simulation data)
            mujoco.mj_step(model, data)
            viewer.sync()
