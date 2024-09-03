#!/usr/bin/env python3

import subprocess
import os
import time

def run_script(script_name):
    # Use Python3 to run the script
    return subprocess.Popen(['python3', script_name])

def main():
    # Get the current directory of this script
    current_dir = os.path.dirname(os.path.realpath(__file__))

    # Paths to the scripts for the left and right arms
    left_arm_script = os.path.join(current_dir, 'z_check_left.py')
    right_arm_script = os.path.join(current_dir, 'z_check_right.py')

    # Start both scripts almost simultaneously
    left_process = run_script(left_arm_script)
    right_process = run_script(right_arm_script)

    print("Started both left and right arm scripts at the same time.")

    # Optionally, wait for both scripts to complete (remove if not needed)
    left_process.wait()
    right_process.wait()

if __name__ == '__main__':
    main()
