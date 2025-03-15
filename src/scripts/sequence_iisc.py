import subprocess
import os

# List of file names to run
file_names = [
    "going_home_async.py",
    "z_attach_joint_angles_real_hw_v3.py",
    # "z_attach_right.py",
    # "services2.py",
    #"2_reset.py",
    #"main_code_static_jh_debug.py",
    
    # "bad_main_code.py"
    #"main_code_static.py",2
    #"main_code copy.py"
    #"traj_direct copy 3.py"
    #"box_trajectory_cub_spl.py"
]

# Directory where the files are located
script_directory = "/home/barat/Debojit_WS/Bi-Manual_Redundancy_Work/src/scripts"

# Loop through the file names and run each file sequentially
for file_name in file_names:
    file_path = os.path.join(script_directory, file_name)

    if not os.path.exists(file_path):
        print(f"Error: {file_path} does not exist! Skipping...")
        continue

    print(f"Running {file_name}...")

    try:
        result = subprocess.run(["python3", file_path], check=True, timeout=300)  # Timeout after 5 minutes

    except subprocess.TimeoutExpired:
        print(f"Error: {file_name} took too long to run! Skipping...")
        continue

    except subprocess.CalledProcessError as e:
        print(f"Error: {file_name} failed with exit code {e.returncode}.")
        break  # Stop execution if a script fails

    except KeyboardInterrupt:
        print("\nProcess interrupted. Exiting...")
        break  # Handle Ctrl+C interruption safely

    print(f"{file_name} completed successfully.\n")