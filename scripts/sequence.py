import subprocess
import os

# List of file names to run
file_names = [
    "going_home_async.py",
    "z_attach_left.py",
    "z_attach_right.py",
    "services.py",
    "box_trajectory_cub_spl.py",
]

# Directory where the files are located
script_directory = "/home/iitgn-robotics-2/franka_ros1_ws/src/scripts/"

# Loop through the file names and run each file sequentially
for file_name in file_names:
    file_path = os.path.join(script_directory, file_name)
    print(f"Running {file_name}...")
    result = subprocess.run(["python3", file_path])
    
    # Check if the process was successful
    if result.returncode != 0:
        print(f"Error: {file_name} failed with exit code {result.returncode}.")
        break  # Stop if any script fails
    print(f"{file_name} completed successfully.\n")
