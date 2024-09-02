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
```

## Custom ROS Topics

This project includes several custom ROS topics that provide essential information for bi-manual manipulation and redundancy optimization in the simulation environment with two Franka robots.

### /compliant_box_pose
This topic provides the pose information of the box being grasped by the two Franka robots in the simulation. The data is published as position coordinates (x, y, z) and quaternions (x, y, z, w) representing the orientation of the box.

### /left_contact_frame_basis
This topic provides the basis information of the left contact point in the simulation. The basis is represented in terms of the rotation matrix components [si, ti, ni] with respect to the world frame.

### /right_contact_frame_basis
This topic provides the basis information of the right contact point in the simulation. Similar to the left contact frame, it is represented as a rotation matrix [si, ti, ni] with respect to the world frame.

### /left_manipulator_jacobian
This topic publishes the Jacobian matrix for the left manipulator, which includes the information necessary to understand the relationship between joint velocities and end-effector velocities for the left arm.

### /right_manipulator_jacobian
This topic publishes the Jacobian matrix for the right manipulator, providing similar information for the right arm, crucial for controlling the end-effector's movements based on joint velocities.

### /left_end_effector_pose
This topic provides the pose of the left end effector, including both the position (x, y, z) and orientation (quaternion) of the left arm's end effector.

### /right_end_effector_pose
This topic provides the pose of the right end effector, including both the position (x, y, z) and orientation (quaternion) of the right arm's end effector.

