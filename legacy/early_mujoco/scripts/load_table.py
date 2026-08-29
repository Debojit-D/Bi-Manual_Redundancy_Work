import mujoco
import mujoco.viewer
import os

def main():
    # Path to the new scene
    # Adjust if you placed vention_table_scene.xml in another folder
    MODEL_PATH = os.path.join(
        os.path.dirname(__file__),
        "..",  # go up one level to 'MUJOCO/'
        "assets",
        "furniture",
        "ventionTable.xml"
    )


    # Make sure the file exists
    if not os.path.exists(MODEL_PATH):
        raise FileNotFoundError(f"Scene file not found: {MODEL_PATH}")

    # Load the model and create a data object
    model = mujoco.MjModel.from_xml_path(MODEL_PATH)
    data = mujoco.MjData(model)

    # Launch the MuJoCo viewer
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Press 'ESC' to close the viewer.")
        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()

if __name__ == "__main__":
    main()
