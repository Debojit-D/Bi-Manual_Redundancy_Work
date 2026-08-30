# Architecture

Package layout and dependency direction for `bimanual_redundancy`. For the
physics/control description of the Equation (8) controller and the
manipulability objectives, see the root `README.md`; for the equation-by-
equation code map, see `docs/PAPER_CODE_MAP.md`.

## Package layout

```
src/bimanual_redundancy/
├── paths.py         Repository-relative resource locations (models/, outputs/, configs/)
├── core/             Mathematical core: Eq. (8) controller, cooperative
│                     kinematics, manipulability objectives (Eq. 12-17),
│                     shared finite-difference gradients, collision penalties
├── systems/          CooperativeSystemSpec registry: the source of truth for
│                     which robot embodiment a run targets (arms, grasp
│                     object, collision geometry)
├── simulation/        MuJoCo backend: scene/model construction, viewer,
│                     cameras, video/CSV recording, run-safety and timing
├── experiments/       Runnable baseline/optimized/comparison scripts
└── plotting/          Publication and data plotting for recorded CSVs
```

Every module under `src/bimanual_redundancy/` is both an importable library
and a runnable script (`python -m bimanual_redundancy.experiments.<name>`);
CLI behavior is guarded behind `if __name__ == "__main__":`, so importing a
module for tests never launches a simulation.

## Dependency direction

```
experiments/, plotting/  -->  simulation/  -->  core/  -->  paths.py
                                                    ^
                                              systems/
```

`core/` never imports `simulation/`: the controller and manipulability
objectives operate on a `mujoco.MjModel`/`mujoco.MjData` pair handed to
them, not on `DualFrankaMuJoCoScene` or the viewer. `simulation/` is the
only layer that owns the MuJoCo viewer, ffmpeg recording, and CLI parsing.
`experiments/` and `plotting/` compose `core/` and `simulation/` into
runnable scripts.

## Mathematical core vs. MuJoCo backend

**`core/`** encodes the paper's equations: the Equation (8) controller
update, cooperative kinematics (grasp matrix, hand Jacobian), and the
velocity/force/directional-force manipulability objectives (Eq. 12-17) with
their finite-difference gradients. Collision/safety penalty shaping
(`collision_penalties.py`) is not part of the paper formulation (see
`docs/PAPER_CODE_MAP.md`) but lives in `core/` because, like
`objectives.py`, it only reads an already-stepped `mujoco.MjData` and never
drives the simulation loop.

**`simulation/`** is everything needed to run those equations inside MuJoCo:
model/scene construction, the viewer, camera presets, recording, and
run-safety/timing instrumentation. Swapping simulators means rewriting this
layer, not `core/`.

## Robot and model ownership

`bimanual_redundancy.systems.SYSTEM_SPECS` (a `CooperativeSystemSpec` per
robot) is the runtime source of truth for a robot embodiment: joint/DoF
indices, grasp geometry, and collision configuration. The reference
embodiment is `dual_franka_panda`, validated with `bimanual-redopt
validate-robot --robot dual_franka_panda`. MJCF models live under
`models/robots/`; `configs/robots/*.json` are sphere-fitting profiles
consumed by the `mujoco_curobo_bridge` submodule, not runtime robot
definitions. See `docs/ADDING_A_ROBOT.md` to add an embodiment.

## Active vs. legacy code

- **Active**: `src/bimanual_redundancy/`, `models/`, `configs/`, `tests/`;
  this is what `pip install -e .` installs and the test suite exercises.
- **Legacy**: `legacy/` (ROS 1, Gazebo, early MuJoCo scripts, old data),
  historical reference only, not imported by the active package. See
  `legacy/README.md`.
- **Deprecated shim**: `src/MUJOCO/` re-exports the historical `MUJOCO.*`
  import paths as thin wrappers around `bimanual_redundancy`, each emitting
  a `DeprecationWarning`. It contains no independent logic and is scheduled
  for removal once downstream callers migrate.
