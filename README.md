# Bi-Manual Redundancy Optimization

This repository studies task-specific redundancy optimization for cooperative
dual-arm manipulation. The active implementation uses two 7-DoF Franka Panda
arms in MuJoCo and follows the closed-loop inverse-kinematics structure in
Equation (8) of *Task-Specific Manipulability Metrics for Redundancy
Optimization in Cooperative Manipulation*.

The current work compares a trajectory-tracking baseline against null-space
optimization using three objectives:

- velocity manipulability;
- force manipulability; and
- directional-force manipulability.

The older ROS 1, Gazebo, and MoveIt experiments are retained under legacy
directories for reference. They are not part of the active Python environment.

## Current MuJoCo workflow

The implemented controller is

```text
phi[k+1] = phi[k]
         + A_control^dagger (q_dot_d + K_p e) dt
         + (I - J_H^dagger J_H) phi_dot_opt dt
```

with

```text
phi_dot_opt = Lambda * dW/dphi.
```

The baseline sets `phi_dot_opt = 0`. The static optimization experiment holds
the measured object pose while activating the projected null-space term.

The paper-faithful map

```text
A = (G.T)^dagger J_H
```

is used to evaluate the manipulability objectives. A separate
compatibility-preserving tracking map is used by the primary controller for
stable physical frictional grasping.

## Repository layout

```text
MUJOCO/
├── assets/                         Furniture and scene assets
├── robot_descriptions/             Franka and object MJCF models
├── scripts/
│   ├── dual_franka_eq8_baseline_pick_place.py
│   ├── dual_franka_eq8_optimized_6d_pick_place.py
│   ├── dual_franka_eq8_optimized_pick_place.py
│   ├── dual_franka_eq8_static_optimization.py
│   └── legacy_code/                Earlier MuJoCo experiments
└── utils/
    ├── grasping_kinematics/
    │   └── cooperative_manipulation_kinematics.py
    ├── redundancy_optimization/
    │   ├── equation_8_controller.py
    │   └── manipulability_optimization.py
    └── scene_builder/
        └── dual_franka_mujoco_scene.py

src/legacy_code/                    Earlier ROS Python experiments
src/legacy_gazebo_stack/            Archived ROS/Gazebo packages
data/legacy_data/                   Earlier recorded results
```

## Quick environment setup

The active code is tested with Python 3.12 on Linux. From the repository root:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -e .
```

The last command installs the repository and the pinned dependencies from
`pyproject.toml`, including MuJoCo, Mink, DAQP, NumPy, SciPy, and the loop-rate
limiter. The environment is local to this repository; the former workaround

```bash
source /home/debojit/debojit/Touch2Screw/.venv/bin/activate
```

is no longer required.

Verify the installation with:

```bash
python -c "import importlib.metadata as m, mujoco; from qpsolvers import available_solvers; print(mujoco.__version__, m.version('mink'), available_solvers)"
```

`daqp` must appear in the solver list.

Whenever opening a new terminal, activate the environment with:

```bash
source .venv/bin/activate
```

Avoid launching these scripts with `/usr/bin/python3`, because that bypasses
the activated virtual environment.

## Running the active experiments

Run commands from the repository root after activating `.venv`.

### Scene preview

```bash
python MUJOCO/utils/scene_builder/dual_franka_mujoco_scene.py
```

This opens the shared scene at the home configuration, keeps the arm targets
active, and applies arm bias compensation until the viewer is closed.

### Equation (8) baseline

```bash
python -m MUJOCO.scripts.dual_franka_eq8_baseline_pick_place
```

Sequence:

1. smoothly approach both grasp points using Mink;
2. close both grippers through physical MuJoCo contact;
3. execute a smooth vertical lift using the primary Equation (8) term;
4. continuously track the final object pose; and
5. keep null-space optimization disabled.

### Optimized Equation (8) lift

```bash
python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place
```

This runs the same grasp, 0.26 m quintic lift, and final-pose hold as the
baseline while evaluating `phi_dot_opt` and applying the projected null-space
term every control cycle. Optimization remains active during the final hold.
Velocity manipulability is the default objective; the force and
directional-force objectives can be selected in the runner settings.

### Optimized 6D pick-and-place trajectory

```bash
python -m MUJOCO.scripts.dual_franka_eq8_optimized_6d_pick_place
```

This example initializes the physical table at an editable start pose, grasps
it, follows one smooth `start -> intermediate -> goal` trajectory in full
SE(3), and then keeps the table grasped at the goal until the viewer closes.
The intermediate pose is a pass-through waypoint rather than a stop: linear
and angular velocity stay continuous there. The trajectory starts and finishes
at zero twist, with Equation (8), null-space optimization, and the soft
inter-arm collision penalty active throughout the motion and final hold.

After the grippers close, the controller records both hand poses relative to
the table. A position-level grasp correction then removes the small drift that
an instantaneous Jacobian null-space projection alone cannot prevent under
finite integration, actuator lag, and compliant contact.

The following world-frame pose settings are editable near the top of the file:

- `TABLE_START_POSITION`, `TABLE_START_EULER_XYZ`;
- `TABLE_INTERMEDIATE_POSITION`, `TABLE_INTERMEDIATE_EULER_XYZ`; and
- `TABLE_GOAL_POSITION`, `TABLE_GOAL_EULER_XYZ`.

Positions are metres. Euler angles are extrinsic XYZ radians and describe the
controlled `site_top_middle` table frame.

### Static null-space optimization

```bash
python -m MUJOCO.scripts.dual_franka_eq8_static_optimization
```

Sequence:

1. approach and physically grasp the object;
2. record the current object pose without lifting it;
3. maintain zero desired object velocity; and
4. continuously apply `(I - J_H^dagger J_H) phi_dot_opt`.

Both module commands can also be run as file paths after the editable install:

```bash
python MUJOCO/scripts/dual_franka_eq8_baseline_pick_place.py
python MUJOCO/scripts/dual_franka_eq8_optimized_6d_pick_place.py
python MUJOCO/scripts/dual_franka_eq8_optimized_pick_place.py
python MUJOCO/scripts/dual_franka_eq8_static_optimization.py
```

## Main experiment settings

The runner files intentionally keep experiment-level parameters near the top.

### Shared settings

- `LEFT_ARM_SPAWN_POSITION`, `RIGHT_ARM_SPAWN_POSITION`: robot base positions
  in world-frame `[x, y, z]` coordinates.
- `SHOW_MOCAP_TARGETS`: display or hide Cartesian target bodies.
- `ENABLE_ARM_BIAS_COMPENSATION`: apply MuJoCo model-bias compensation to the
  14 arm DoFs.
- `K_P`: closed-loop object position/orientation feedback gains.

Changing an arm spawn position changes only the corresponding fixed base-body
position; joint states, base orientations, object pose, and other scene
properties remain unchanged.

### Lift settings

- `LIFT_HEIGHT`
- `LIFT_DURATION`
- `CONTROL_HZ`

### Optimization settings

- `OBJECTIVE`: `VELOCITY`, `FORCE`, or `DIRECTIONAL_FORCE`;
- `OPTIMIZATION_GAIN`: the Equation (4) gain `Lambda`;
- `MAXIMUM_OPTIMIZATION_JOINT_SPEED`: final safety limit for `phi_dot_opt`;
- `FINITE_DIFFERENCE_STEP`: joint perturbation used to evaluate `dW/dphi`;
- `DESIRED_WRENCH_DIRECTION`: `[Fx, Fy, Fz, Mx, My, Mz]`; and
- `CHARACTERISTIC_LENGTH`: converts moment entries to force-equivalent units
  for the spatial directional-force objective.

The directional-force cost is minimized automatically; the velocity and force
objectives are maximized.

## Manipulability objectives

The optimization utilities implement the three paper costs:

```text
Velocity:          W_v = sqrt(det(A A.T))
Force:             W_f = sqrt(det((A A.T)^dagger))
Directional force: normalized Frobenius distance between A A.T and F
```

Joint gradients are computed with central finite differences and then passed
to the Equation (8) controller for null-space projection.

## Actuation and gravity compensation

The Franka arm actuators are affine `general` actuators configured as
joint-position PD servos:

```text
tau = K_p (q_command - q) - K_d q_dot.
```

Therefore arm commands in `data.ctrl` are joint-position targets, not direct
torques. Gravity and velocity-dependent model bias are applied only to the arm
DoFs through `qfrc_applied` at every MuJoCo substep.

The gripper DoFs do not receive this compensation. Their 0–255 actuator mapping
is calibrated so `255` corresponds to the fully open tendon length and produces
zero closing preload at startup.

## Implementation notes and limitations

- The grasp is generated by physical fingertip contact; there is no weld,
  teleport, or hidden object attachment.
- Object pose is measured from the live MuJoCo state and fed back continuously.
- The current spatial dual-Franka study is most mature for velocity
  manipulability. Force and directional-force objectives are implemented for
  comparison, but their spatial experimental interpretation remains ongoing.
- The mathematical model assumes fixed full contacts, while the simulation
  uses frictional contacts. This distinction should be reported when comparing
  results with the paper.
- Legacy ROS/Gazebo code requires a separate ROS 1 workspace and is not
  installed by the repository-local Python environment.

## Troubleshooting

### `SolverNotFound: osqp`

The active scripts use DAQP. Confirm the local environment is active and DAQP
is available:

```bash
python -c "from qpsolvers import available_solvers; print(available_solvers)"
```

### `ModuleNotFoundError: MUJOCO`

Install the repository editable from its root:

```bash
source .venv/bin/activate
python -m pip install -e .
```

### Viewer does not open

The interactive viewer requires a working desktop/OpenGL display. Headless
simulation can still use MuJoCo with an appropriate EGL or OSMesa setup, but
the provided entry points launch the interactive passive viewer by default.
