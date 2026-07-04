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

The active code is tested with Python 3.12 on Linux. Confirm that Python 3.12
is installed before creating the environment:

```bash
python3.12 --version
```

From the repository root, create the environment explicitly with Python 3.12:

```bash
python3.12 -m venv .venv
source .venv/bin/activate
python --version  # Must report Python 3.12.x
python -m pip install --upgrade pip
python -m pip install -e .
```

If `python3.12` is not installed (for example, Ubuntu 20.04 does not provide it
in its standard repositories), install a user-local Python with
[uv](https://docs.astral.sh/uv/). This does not replace the operating system's
Python:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
source "$HOME/.local/bin/env"
uv python install 3.12
uv venv --python 3.12 .venv
source .venv/bin/activate
python --version  # Must report Python 3.12.x
uv pip install -e .
```

`uv venv` creates a minimal environment without the `pip` Python module by
default, so `python -m pip` will report `No module named pip` in this setup.
Use `uv pip` as shown above. Alternatively, `uv venv --seed --python 3.12
.venv` creates an environment containing `pip`.

If creating `.venv` fails, stop there: activation and installation cannot work
until the environment exists. Outside an activated environment, `python` may
refer to Python 2 on older Linux installations.

Do not substitute an older system `python3` when creating the environment. The
project requires Python 3.10 or newer, and its pinned dependencies are tested
with Python 3.12. On Python 3.8, for example, pip may misleadingly report that
it cannot find `mink==1.1.1`; the release is being hidden because it is not
compatible with that interpreter.

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

This grasps the table, executes a 0.26 m quintic lift, smoothly lowers it back
to its measured starting pose, and then holds it there. It evaluates
`phi_dot_opt` and applies the projected null-space term during lifting,
lowering, and the final hold. The force, velocity, and directional-force
objectives can be selected in the runner settings.

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

The standalone runner remains independently usable. Select an objective with
`--objective velocity`, `--objective force`, or
`--objective directional_force`; use `--baseline` to suppress the null-space
term while monitoring the selected metric. It starts immediately after the
grasp is established and can either run until the viewer closes or for a fixed
`--duration`.

### Four-mode static comparison

Run baseline and all three paper objectives as four independent viewer
sessions:

```bash
python -m MUJOCO.scripts.dual_franka_eq8_static_comparison
```

The order is baseline, velocity manipulability, force manipulability, and
directional-force manipulability. Each mode starts immediately after the
grasp is established. It advances automatically once the maximum applied
null-space joint speed remains at or below `0.005 rad/s` for `0.5 s` (after a
minimum run time of `1 s`). Closing a viewer early also advances to the next
mode. Customize the convergence rule or record the sequence with:

```bash
python -m MUJOCO.scripts.dual_franka_eq8_static_comparison \
  --convergence-speed 0.005 --convergence-hold 0.5 --record-data
```

Convergence stopping is the default. For matched, fixed-duration recordings,
pass `--duration`; this disables convergence stopping and gives every mode the
same requested recording interval:

```bash
python -m MUJOCO.scripts.dual_franka_eq8_static_comparison \
  --duration 10 --record-data
```

Recording creates a separately named, timestamped CSV for each mode. The
`optimization_mode` column distinguishes `baseline`, `velocity`, `force`, and
`directional_force` samples.

Plot the newest four-mode dataset with:

```bash
python -m MUJOCO.plotting_scripts.plot_eq8_static_comparison
```

This produces five paper-oriented figures in
`outputs/mujoco_data/static_comparison_figures/`: separate velocity, force,
and directional-force metric plots; one combined `[0, 1]` min-max normalized
optimization-progress plot; and actuator effort. For the normalized plot,
higher-is-better metrics are mapped toward `1`, while the lower-is-better
directional cost is inverted so progress has the same meaning. Effort is
computed directly from the 14 recorded joint torques as `sqrt(tau @ tau.T)`.

### CSV data recording

The static and optimized lift-and-lower experiments can record every Equation
(8) control step:

```bash
python -m MUJOCO.scripts.dual_franka_eq8_static_optimization --record-data
python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place --record-data
```

Timestamped files are written to `outputs/mujoco_data/`. To choose a path:

```bash
python -m MUJOCO.scripts.dual_franka_eq8_static_optimization \
  --output-csv outputs/my_static_run.csv
```

Supplying `--output-csv` enables recording without requiring `--record-data`.
Existing paths are not overwritten; a numeric suffix is added. Each row is
flushed immediately, and the file is closed in a `finally` block, so closing
the viewer early or pressing `Ctrl+C` preserves every completed sample. The
CSV contains object pose/error, grasp error, Equation (8) velocity terms,
optimization gradient and objective, collision diagnostics, 14 arm joint
positions, and actuator/bias/constraint/applied torque diagnostics.
`tau_total_est_norm` uses
`qfrc_actuator + qfrc_applied + qfrc_constraint - qfrc_bias` over the 14 arm
DoFs.

Create publication-oriented plots for a recorded static run with:

```bash
python -m MUJOCO.plotting_scripts.plot_eq8_static_optimization \
  outputs/mujoco_data/<recording>.csv
```

The CSV argument is optional. Running the module without it uses the
`DEFAULT_INPUT_CSV` path near the top of the plotting script, which can be
edited for IDE Run/Debug use:

```bash
python -m MUJOCO.plotting_scripts.plot_eq8_static_optimization
```

By default, the script creates a sibling `<recording>_figures/` directory with
PDF and 300 dpi PNG versions of six figure groups: normalized objective
improvement, pose tracking, controller behavior, collision/joint-limit safety,
joint-posture evolution, and actuator effort. It also writes `run_summary.txt`
with headline metrics. Use `--output-dir`, `--format`, `--dpi`, and `--show` to
customize the output; run the command with `--help` for details. These plots
describe one representative run, so baseline comparisons and variability
claims require additional matched recordings.

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
  in world-frame `[x, y, z]` coordinates. Both arms share one fixed-footprint
  mounting base from the ground to their common positive `z` height; at
  `z <= 0` that base is hidden and non-colliding. The two `z` values must be
  equal because both robots sit on the same top surface.
- `LEFT_ARM_SPAWN_EULER_XYZ_DEGREES`,
  `RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES`: robot-base orientations as extrinsic
  XYZ Euler angles in degrees. The shared scene API also retains optional
  `*_euler_xyz` radian arguments for programmatic callers.
- `TABLE_SPAWN_POSITION` in the static and optimized lift-and-lower runners:
  world-frame `[x, y, z]` position of the table's `site_top_middle` reference
  frame.
- `SHOW_MOCAP_TARGETS`: display or hide Cartesian target bodies.
- `ENABLE_ARM_BIAS_COMPENSATION`: apply MuJoCo model-bias compensation to the
  14 arm DoFs.
- `K_P`: closed-loop object position/orientation feedback gains.

Changing an arm spawn position changes the corresponding fixed base-body and
recenters the shared mounting platform between the two robot bases. Joint
states, base orientations, object pose, and other scene properties remain
unchanged. Its footprint is configured once in
`DualFrankaMuJoCoScene.MOUNTING_PLATFORM_HALF_SIZE_XY`.

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
