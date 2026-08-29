# Architecture

This document describes the package layout produced by Refactor 1
(`refactor/01-package-layout`): a reorganization of the research codebase
into a conventional src-layout Python package. It changes where code lives,
not what it computes — see the root `README.md` for the physics/control
description of the Equation (8) controller and the manipulability
objectives.

## Package ownership

```
src/bimanual_redundancy/
├── paths.py          Repository-relative resource locations (models/, outputs/, configs/)
├── core/              Mathematical core — engine-independent in spirit, though
│                       objectives.py and cooperative_kinematics.py use MuJoCo's
│                       forward-kinematics/Jacobian queries against an already
│                       up-to-date mujoco.MjData, not the simulation loop itself.
│   ├── cooperative_kinematics.py    Grasp matrix G, hand Jacobian J_H, Eq.(8) projector
│   ├── controller.py                 Equation (8) closed-loop update
│   ├── objectives.py                 Manipulability objectives (Eq. 13-15), collision
│   │                                   costs, and their finite-difference gradients
│   └── directional_distance_optimization.py
│                                      Experimental 2x2 directional-distance permutation
│                                      study (sits beside objectives.py by design — see
│                                      "Known deviations" below)
├── simulation/        MuJoCo backend — scene construction, the simulation loop,
│                       recording, and viewer-only concerns
│   ├── scene.py                      DualFrankaMuJoCoScene: model/data ownership,
│   │                                   actuators, viewer camera setup
│   ├── collisions.py                 Viewer overlay drawing for collision spheres
│   │                                   computed by core.objectives
│   ├── cameras.py                    Camera distance/view presets
│   ├── cli.py                        Shared argparse helpers for interactive scripts
│   ├── control_timing.py             Computation-only timing around controller updates
│   ├── comparison_run_safety.py      Failure isolation for batch comparison sweeps
│   ├── grasp_safety.py               Object-pose-drift grasp-loss detection
│   └── recording/
│       ├── video.py                  Headless dual-view MP4 recording (ffmpeg)
│       └── csv_recorder.py           Step-wise Equation (8) CSV recording
├── experiments/       Runnable experiments: baselines, optimized runs, and
│                       four-mode/six-mode comparison sweeps for static, translational
│                       pick-and-place, and 6D pick-and-place tasks (see "Known
│                       deviations" — these are not yet split into static.py /
│                       translational.py / six_d.py files)
└── plotting/          Publication and data plotting for recorded CSV runs
```

Everything under `src/bimanual_redundancy/` is a runnable module
(`python -m bimanual_redundancy.experiments.<name>`) as well as an importable
library; scripts guard their CLI behind `if __name__ == "__main__":` so
importing them (e.g. from tests) never launches a simulation.

## Dependency direction

```
experiments/ , plotting/  ──depends on──>  simulation/  ──depends on──>  core/
                                                 │
                                                 └──depends on──> paths.py
core/ ──depends on──> paths.py  (not on simulation/)
```

`core/` never imports from `simulation/`: the controller and the
manipulability objectives operate on a `mujoco.MjModel`/`mujoco.MjData` pair
handed to them, not on `DualFrankaMuJoCoScene` or the viewer. `simulation/`
is the only layer that owns the MuJoCo viewer, ffmpeg recording, and
argparse/CLI concerns. `experiments/` and `plotting/` compose `core/` and
`simulation/` into runnable scripts and never import each other except where
a comparison script explicitly reuses a baseline/optimized experiment module
as a library (e.g. `dual_franka_eq8_pick_place_comparison.py` importing
`dual_franka_eq8_optimized_pick_place` for its `main()`/parameters).

`bimanual_redundancy/paths.py` is a single source of truth for
repository-relative locations (`models/`, `configs/`, `outputs/`,
`mujoco_curobo_bridge/`), replacing the historical
`Path(__file__).resolve().parents[N]` pattern that broke silently whenever a
module moved to a different directory depth (which this refactor did, by
construction, for every active module).

## Mathematical core vs. MuJoCo backend

- **Mathematical core** (`core/`): Equation (8) controller update,
  cooperative kinematics (grasp matrix, hand Jacobian), and the
  velocity/force/directional-force manipulability objectives with their
  finite-difference gradients and collision-cost terms. This is the part of
  the codebase that encodes the paper's equations.
- **MuJoCo backend** (`simulation/`): everything needed to actually run those
  equations inside a MuJoCo simulation — model/scene construction, the
  viewer, camera presets, video/CSV recording, and run-safety/timing
  instrumentation. Swapping simulators would mean rewriting this layer, not
  `core/`.

## Active vs. legacy code

- **Active**: `src/bimanual_redundancy/`, `models/`, `configs/`, `tests/`.
  This is what `pip install -e .` installs and what the test suite exercises.
- **Legacy**: `legacy/ros1/`, `legacy/gazebo/`, `legacy/early_mujoco/`,
  `legacy/data/` — retained for historical reference only, not imported by
  the active package, not covered by tests. See `legacy/README.md`.
- **Deprecated compatibility shim**: `src/MUJOCO/` mirrors the historical
  `MUJOCO.*` import paths (`MUJOCO.scripts.*`, `MUJOCO.utils.*`,
  `MUJOCO.plotting_scripts.*`) as thin re-export wrappers around
  `bimanual_redundancy`, each emitting a `DeprecationWarning`. It contains no
  independent implementation — only imports from the active package — so it
  cannot drift out of sync with `core`/`simulation`/`experiments`/`plotting`
  logic. It is intended to be deleted in a later refactor once downstream
  callers have migrated.

## Known deviations from the target layout

This refactor followed the requested `core/{cooperative_kinematics,
controller, objectives, gradients}.py` and `experiments/{static,
translational, six_d}.py` shape as closely as possible without merging or
splitting any function/class bodies (the refactor's explicit constraint).
Where the real codebase didn't collapse cleanly into that shape, it was kept
as-is and documented here rather than forced:

- **No standalone `gradients.py`.** Finite-difference gradient evaluation is
  a method (`ManipulabilityOptimizer.gradient` /
  `DirectionalDistancePermutationOptimizer.gradient`) embedded in
  `objectives.py` and `directional_distance_optimization.py`, not a
  free-standing module. Extracting it would mean splitting those classes,
  which this refactor was explicitly told not to do.
- **`core/directional_distance_optimization.py` is an extra file** beyond the
  four named in the target tree. It is an experimental permutation study
  that intentionally sits beside `objectives.py` rather than inside it (see
  its own module docstring); it belongs conceptually to `core/` because it
  has the same MuJoCo-independent-in-spirit, objective/gradient shape as
  `objectives.py`.
- **`simulation/recording/` is a subpackage (`video.py` + `csv_recorder.py`),
  not a single `recording.py`.** The two files were separate, independently
  used modules (`utils/video_recording.py` and
  `utils/data_recording/equation8_csv_recorder.py`); merging them into one
  file would have meant concatenating unrelated implementations for the sake
  of matching a filename.
- **`experiments/` is flat, not split into `static.py` / `translational.py`
  / `six_d.py`.** The real experiment surface is ~14 scripts (baseline,
  optimized, and comparison-sweep variants per task, plus batch/orchestration
  scripts such as `comparison_main.py` and
  `table_spawn_comparison_positions.py`) — collapsing each task family into
  one file would require merging distinct `main()` entry points and
  argument-parsing logic, which is out of scope for a behavior-preserving
  move. Splitting `experiments/` into `static/`, `translational/`, `six_d/`,
  and `batch/` subpackages of existing files (no code changes, just more
  `mkdir`) is a reasonable follow-up for Refactor 2.
- **`simulation/` also holds `cli.py`, `control_timing.py`,
  `comparison_run_safety.py`, and `grasp_safety.py`**, which aren't named in
  the target tree's `simulation/` bullet list (scene/collisions/cameras/
  recording). They're cross-cutting helpers used only by simulation-running
  scripts, so they were kept there rather than introducing a fifth
  `common/`-style package for four small files.

## Unresolved architectural debt

- The `src/MUJOCO/` compatibility shim should be removed once downstream
  scripts/notebooks that still `import MUJOCO...` have migrated to
  `bimanual_redundancy`.
- `experiments/` would benefit from the `static/` / `translational/` /
  `six_d/` / `batch/` subdivision described above.
- Collision-cost *computation* (as opposed to its viewer *visualization* in
  `simulation/collisions.py`) still lives inside
  `core.objectives.ManipulabilityOptimizer` alongside the manipulability
  objectives themselves. Separating "objective value" from "collision
  penalty" into distinct classes would clarify `core/` further but is a
  behavior-risking change, not a move, so it was left untouched.
- `models/objects/furniture/ventionTable.xml` (as opposed to the
  `ventionTable/` subdirectory actually included by the active scene) embeds
  a hardcoded absolute path from a different machine and appears unused by
  any active script or MJCF include. It was relocated as-is; whether it's
  dead weight worth deleting is a decision for the repository owner, not
  this refactor.
