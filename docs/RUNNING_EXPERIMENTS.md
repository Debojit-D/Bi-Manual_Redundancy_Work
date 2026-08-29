# Running the experiment scripts

This is a reference for the individual, ad hoc `bimanual_redundancy.experiments`
and `bimanual_redundancy.plotting` scripts — interactive viewers, per-script
CLI flags, output paths, and the in-file constants each runner exposes.

This is **not** the paper-reproduction pipeline. For the config-driven
commands that reproduce the manuscript's campaigns
(`bimanual-redopt run` / `bimanual-redopt reproduce-paper`), see
[`REPRODUCING_THE_PAPER.md`](REPRODUCING_THE_PAPER.md) instead. The scripts
documented here are the underlying runners that both the interactive
workflow below and the config-driven pipeline call into.

Run commands from the repository root after activating `.venv`.

## Scene preview

```bash
python -m bimanual_redundancy.simulation.scene
```

This opens the shared scene at the home configuration, keeps the arm targets
active, and applies arm bias compensation until the viewer is closed.
Use `--top-view` for the directly overhead camera. Interactive experiment
runners support the same flag. The shared scene opens viewers maximized and
uses separate configurable look-at targets for the perspective and top views.

## Equation (8) baseline

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_baseline_pick_place
```

Sequence:

1. smoothly approach both grasp points using Mink;
2. close both grippers through physical MuJoCo contact;
3. execute a smooth vertical lift using the primary Equation (8) term;
4. continuously track the final object pose; and
5. keep null-space optimization disabled.

## Optimized Equation (8) lift

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_pick_place
```

This grasps the table, executes a 0.26 m quintic lift, smoothly lowers it back
to its measured starting pose, and then holds it there. It evaluates
`phi_dot_opt` and applies the projected null-space term during lifting,
lowering, and the final hold. The force, velocity, and directional-force
objectives can be selected in the runner settings.

## Optimized 6D pick-and-place trajectory

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place
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

## Four-mode pick-and-place comparisons

Run the baseline, velocity, force, and directional-force modes through the
same lift-and-return trajectory:

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_pick_place_comparison
```

Run the same four modes through the configurable full-SE(3) trajectory:

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_6d_pick_place_comparison
```

Both runners use four independent MuJoCo scenes and accept `--record-data`,
`--hold-duration`, `--top-view`, collision options, and optimizer limits. The
6D runner additionally exposes all start, intermediate, and goal poses and
both trajectory-segment durations through its CLI. Run either command with
`--help` for the complete option list.

## Static null-space optimization

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_static_optimization
```

Sequence:

1. approach and physically grasp the object;
2. record the current object pose without lifting it;
3. maintain zero desired object velocity; and
4. continuously apply `(I - J_H^dagger J_H) phi_dot_opt`.

The standalone runner remains independently usable. Select an objective with
`--objective velocity`, `--objective force`, `--objective
directional_force` (Eq. 16, direct), or `--objective
directional_force_indirect` (Eq. 17, indirect); use `--baseline` to
suppress the null-space term while monitoring the selected metric. It
starts immediately after the
grasp is established and can either run until the viewer closes or for a fixed
`--duration`.

## Static comparison (baseline plus four objectives)

Run baseline and all four paper objectives as five independent viewer
sessions:

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_static_comparison
```

The order is baseline, velocity manipulability, force manipulability,
direct directional-force manipulability (Eq. 16), and indirect
directional-force manipulability (Eq. 17) -- the manuscript's matched
static comparison of both directional-force formulations. Each mode starts
immediately after the
grasp is established. It advances automatically once the maximum applied
null-space joint speed remains at or below `0.005 rad/s` for `0.5 s` (after a
minimum run time of `1 s`). Closing a viewer early also advances to the next
mode. Customize the convergence rule or record the sequence with:

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_static_comparison \
  --convergence-speed 0.005 --convergence-hold 0.5 --record-data
```

Convergence stopping is the default. For matched, fixed-duration recordings,
pass `--duration`; this disables convergence stopping and gives every mode the
same requested recording interval:

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_static_comparison \
  --duration 10 --record-data
```

Recording creates a separately named, timestamped CSV for each mode. The
`optimization_mode` column distinguishes `baseline`, `velocity`, `force`,
`directional_force`, and `directional_force_indirect` samples.

## Headless dual-view video recording

Every four-mode comparison can run without opening the interactive viewer and
record both the tuned perspective camera and the overhead camera:

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_static_comparison --record-video
python -m bimanual_redundancy.experiments.dual_franka_eq8_pick_place_comparison --record-video
python -m bimanual_redundancy.experiments.dual_franka_eq8_6d_pick_place_comparison --record-video
```

Headless recording skips real-time sleeping and displays simulation progress
with `tqdm`. Each invocation creates a timestamped directory under
`outputs/mujoco_videos/`, with one directory per mode and two files inside:

```text
outputs/mujoco_videos/<experiment_timestamp>/
├── baseline/{perspective.mp4,top_view.mp4}
├── velocity/{perspective.mp4,top_view.mp4}
├── force/{perspective.mp4,top_view.mp4}
└── directional_force/{perspective.mp4,top_view.mp4}
```

The defaults are 1280x720 at 30 fps. Override them with `--video-width`,
`--video-height`, and `--video-fps`, or choose another root with
`--video-output-dir`. Width and height must be positive even numbers. Video
recording can be combined with `--record-data`, but fitted collision-sphere
overlays are currently available only in the interactive viewer.

## Directional-distance 2x2 permutation test

The experimental directional-distance optimizer has a separate four-case
runner; it does not replace the paper's static optimizer:

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_directional_distance_comparison
```

It runs force-capability minimize/maximize followed by velocity-capability
minimize/maximize in independent viewer sessions. Convergence stopping is the
default. Use a fixed interval and record all four cases with:

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_directional_distance_comparison \
  --duration 10 --record-data
```

Plot the newest four permutation recordings with:

```bash
python -m bimanual_redundancy.plotting.plot_eq8_directional_distance_comparison
```

The plotter creates four separate raw-distance figures, one combined `[0, 1]`
normalized-progress figure, one actuator-effort figure, and a text summary in
`outputs/mujoco_data/directional_distance_comparison_figures/`.

Plot the newest four-mode dataset with:

```bash
python -m bimanual_redundancy.plotting.plot_eq8_static_comparison
```

This produces five paper-oriented figures in
`outputs/mujoco_data/static_comparison_figures/`: separate velocity, force,
and directional-force metric plots; one combined `[0, 1]` min-max normalized
optimization-progress plot; and actuator effort. For the normalized plot,
higher-is-better metrics are mapped toward `1`, while the lower-is-better
directional cost is inverted so progress has the same meaning. Effort is
computed directly from the 14 recorded joint torques as `sqrt(tau @ tau.T)`.

Plot the newest simple pick-and-place four-mode dataset with:

```bash
python -m bimanual_redundancy.plotting.plot_eq8_pick_place_comparison
```

Use `--experiment 6d` for the newest 6D comparison dataset. This plotter
creates the measured 3D object path, all three manipulability metrics,
tracking and grasp errors, null-space speed, actuator effort, inter-arm
clearance, and joint-limit margin. Use `--format`, `--output-dir`, and `--show`
to control figure output.

## CSV data recording

The static and optimized lift-and-lower experiments can record every Equation
(8) control step:

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_static_optimization --record-data
python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_pick_place --record-data
```

Timestamped files are written to `outputs/mujoco_data/`. To choose a path:

```bash
python -m bimanual_redundancy.experiments.dual_franka_eq8_static_optimization \
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
python -m bimanual_redundancy.plotting.plot_eq8_static_optimization \
  outputs/mujoco_data/<recording>.csv
```

The CSV argument is optional. Running the module without it uses the
`DEFAULT_INPUT_CSV` path near the top of the plotting script, which can be
edited for IDE Run/Debug use:

```bash
python -m bimanual_redundancy.plotting.plot_eq8_static_optimization
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
python src/bimanual_redundancy/experiments/dual_franka_eq8_baseline_pick_place.py
python src/bimanual_redundancy/experiments/dual_franka_eq8_optimized_6d_pick_place.py
python src/bimanual_redundancy/experiments/dual_franka_eq8_optimized_pick_place.py
python src/bimanual_redundancy/experiments/dual_franka_eq8_static_optimization.py
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

- `OBJECTIVE`: `VELOCITY`, `FORCE`, `DIRECTIONAL_FORCE` (Eq. 16, direct), or
  `DIRECTIONAL_FORCE_INDIRECT` (Eq. 17, indirect);
- `OPTIMIZATION_GAIN`: the Equation (4) gain `Lambda`;
- `MAXIMUM_OPTIMIZATION_JOINT_SPEED`: final safety limit for `phi_dot_opt`;
- `FINITE_DIFFERENCE_STEP`: joint perturbation used to evaluate `dW/dphi`;
- `DESIRED_WRENCH_DIRECTION`: `[Fx, Fy, Fz, Mx, My, Mz]`; and
- `CHARACTERISTIC_LENGTH`: converts moment entries to force-equivalent units
  for the spatial directional-force objective.

The direct directional-force cost (Eq. 16) is minimized automatically; the
velocity, force, and indirect directional-force (Eq. 17) objectives are
maximized.
