# Legacy code

Everything under this directory is retained for historical reference only.
None of it is imported by the active `bimanual_redundancy` package, required
to reproduce the paper's results, or covered by the test suite.

## Contents

- `ros1/`: earlier ROS 1 Python control code (`legacy_code/`) and the ROS 1
  catkin tracking package (`ros_track/`). Depends on `rospy`,
  `moveit_commander`, and other ROS 1 packages not part of this project's
  Python environment.
- `gazebo/`: the archived Gazebo/MoveIt simulation stack
  (`legacy_gazebo_stack/`), URDF/xacro descriptions and `moveit_config`
  packages used before the project moved to MuJoCo.
- `early_mujoco/`: superseded MuJoCo scripts that predate the current
  `bimanual_redundancy.experiments` implementations.
- `data/`: earlier recorded results, plots, and images, from before the
  current `outputs/` recording convention.

These directories are research snapshots from before the codebase
consolidated around the Equation (8) controller and MuJoCo backend. They are
not maintained and not guaranteed to run. To reproduce the paper's results,
use the top-level `README.md` and `src/bimanual_redundancy/` instead.
