# Bi-Manual Redundancy Optimization

## Description
This repository is under development and includes the work currently being carried out in Bi-Manual Redundancy Optimization.

## Features
- Bi-Manual Redundancy Optimization
- Integration with Franka Emika Panda robots
- MoveIt! configuration for multiple arms

## Installation
1. Create a workspace:
    ```bash
    mkdir -p ~/franka_ros1_ws/src
    cd ~/franka_ros1_ws/src
    ```

2. Clone the `franka_ros` repository from the Franka Emika website:
    ```bash
    git clone https://github.com/frankaemika/franka_ros.git
    ```

3. Clone the following packages and folders:
    ```bash
    git clone https://github.com/Debojit-D/without_gripper_panda_multiple_arms.git
    git clone https://github.com/Debojit-D/without_gripper_panda_multiple_arms_moveit_config.git
    ```

4. Navigate back to the workspace root and build the workspace:
    ```bash
    cd ~/franka_ros1_ws
    catkin_make
    ```

Given below is the image of how the differet reference frames have been considered.

![Alt text](Frames.jpg)


## Usage
```bash
# Source the workspace
source devel/setup.bash

# Launch the project
roslaunch without_gripper_panda_multiple_arms bringup_moveit.launch.
