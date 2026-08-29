# Legacy code

Everything under this directory is retained **only for historical
reference**. None of it is imported by the active `bimanual_redundancy`
package, none of it is required to reproduce the paper's results, and none of
it is covered by the active test suite.

## Contents

- `ros1/` — Earlier ROS 1 Python control code (`legacy_code/`) and the ROS 1
  catkin tracking package (`ros_track/`), including a leftover
  `COLCON_IGNORE` build marker (`log/`). Depends on `rospy`, `moveit_commander`,
  and other ROS 1 packages that are not part of this project's Python
  environment.
- `gazebo/` — The archived Gazebo/MoveIt simulation stack
  (`legacy_gazebo_stack/`): URDF/xacro robot descriptions, Gazebo world
  assets, and `moveit_config` packages used before the project moved to
  MuJoCo.
- `early_mujoco/` — Superseded MuJoCo scripts (`scripts/`) that predate the
  current `bimanual_redundancy.experiments` implementations and are no longer
  part of the published reproduction pipeline.
- `data/` — Earlier recorded results, plots, and images
  (`old_data/`, `legacy_data/`, `misc/`) collected before the current
  `outputs/` data-recording convention.

## Why this code is kept

These directories document the project's history — the original ROS 1/Gazebo
implementation, and earlier MuJoCo experiments — before the codebase
consolidated around the Equation (8) controller and the current MuJoCo
simulation backend. They are not maintained, are not guaranteed to run, and
should not be used as a starting point for new work.

If you are trying to reproduce the paper's results, start from the top-level
`README.md` and `src/bimanual_redundancy/` instead.
