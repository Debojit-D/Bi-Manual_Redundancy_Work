# Bimanual Redundancy Optimization

**Paper:** *Task-Specific Manipulability Metrics for Redundancy Optimization
in Cooperative Manipulation*
**Authors:** Debojit Das, Barat S., Harish J. Palanthandalam-Madapusi
**Status:** Provisionally accepted, *Industrial Robot: The International
Journal of Robotics Research and Application*
**Project website:** https://debojit-d.github.io/Bimanual-Redundancy-Optimization/
**Repository:** https://github.com/Debojit-D/Bimanual-Redundancy-Optimization

## Overview

This repository is the reference implementation for task-specific
redundancy optimization in cooperative dual-arm manipulation. It implements
the closed-loop Equation (8) controller from the manuscript for two 7-DoF
Franka Panda arms cooperatively manipulating a shared object in MuJoCo, and
compares a trajectory-tracking baseline against null-space optimization
under several manipulability objectives. Everything needed to reproduce the
manuscript's spatial simulation campaigns is config-driven and runs through
a single CLI.

The older ROS 1, Gazebo, and MoveIt experiments that predate the MuJoCo
implementation are retained under `legacy/` for historical reference only;
they are not part of the active Python environment.

## Implemented objectives

The optimization utilities implement four manuscript costs. "Directional
force" is not one formula: the manuscript defines two distinct,
non-equivalent objectives (Appendix A), and this repository implements both.

```text
Velocity (Eq. 13, maximized):
    W_v = sqrt(det(A A.T))

Force (Eq. 14, maximized):
    W_f = sqrt(det((A A.T)^dagger))

Directional force, direct (Eq. 16, minimized):
    normalized Frobenius distance between (A A.T)^dagger and F

Directional force, indirect (Eq. 17, maximized):
    normalized Frobenius distance between A A.T and F
```

The direct formulation (`--objective directional_force`) compares in
force-capability space and is the default/primary directional-force mode in
this spatial study. The indirect formulation
(`--objective directional_force_indirect`) compares in velocity-capability
space and is additionally evaluated in the static and six-dimensional
comparisons; it corresponds to the formulation used by the manuscript's
separate planar hardware experiments, which this repository does not
implement (see [Hardware implementation](#hardware-implementation) below).
Full equation-by-function detail is in
[`docs/PAPER_CODE_MAP.md`](docs/PAPER_CODE_MAP.md).

## Quick start

```bash
git clone --recurse-submodules \
  https://github.com/Debojit-D/Bimanual-Redundancy-Optimization.git
cd Bimanual-Redundancy-Optimization

uv venv --python 3.12 .venv
source .venv/bin/activate

uv pip install -e .
```

Headless video recording additionally requires `ffmpeg`, and the interactive
viewer requires a working OpenGL/display environment. If any step above
doesn't work as shown (no `python3.12`, `uv venv` missing `pip`, no display),
see [`docs/TROUBLESHOOTING.md`](docs/TROUBLESHOOTING.md).

## 5-minute example

Validate that the reference robot embodiment loads correctly:

```bash
bimanual-redopt validate-robot --robot dual_franka_panda
```

This prints the controlled joint count and the resolved `qpos`/DoF indices —
a quick sanity check that the MuJoCo model, submodule, and environment are
all wired up correctly.

Then run a very short, headless smoke pass of one paper campaign end to end
(scene construction, controller, and CSV recording, at a tiny simulated
duration — this checks plumbing, not publication statistics):

```bash
bimanual-redopt run --config configs/paper/static.toml --smoke
```

The command prints the output directory it wrote to under
`outputs/paper_reproduction/`.

## Reproduce the paper

Every manuscript campaign is driven by a TOML config under `configs/paper/`
through the same CLI:

| Paper config | Command | Output |
|---|---|---|
| `static.toml` | `bimanual-redopt run --config configs/paper/static.toml` | Static optimization campaign (Figures 10, 12) |
| `six_d.toml` | `bimanual-redopt run --config configs/paper/six_d.toml` | Representative 6D trajectories (Figure 11, part of Figure 14/Table I) |
| `translational.toml` | `bimanual-redopt run --config configs/paper/translational.toml` | Translational pick-and-place (Figure 13, part of Table I) |
| `directional_direct_vs_indirect.toml` | `bimanual-redopt run --config configs/paper/directional_direct_vs_indirect.toml` | Direct vs. indirect directional-force comparison (Figure 16) |

Run every campaign in one call with `bimanual-redopt reproduce-paper`, or
`bimanual-redopt reproduce-paper --smoke` for the short plumbing check used
above.

For the full manuscript-figure-level breakdown, output provenance, and exact
reproduction caveats, see
[`docs/REPRODUCING_THE_PAPER.md`](docs/REPRODUCING_THE_PAPER.md). For
per-script interactive/ad hoc experiment usage (outside the config-driven
pipeline), see [`docs/RUNNING_EXPERIMENTS.md`](docs/RUNNING_EXPERIMENTS.md).

## Paper ↔ Code

[`docs/PAPER_CODE_MAP.md`](docs/PAPER_CODE_MAP.md) is the authoritative
equation index — every manuscript equation number maps to its implementing
function, file, and test, for example:

| Eq. | What | Function | File |
|---|---|---|---|
| 8 | Closed-loop redundancy update law | `Equation8Controller.update` | `core/controller.py` |
| 13 | Velocity manipulability | `velocity_cost` | `core/objectives.py` |
| 16 | Directional force (direct) | `directional_force_direct_cost` | `core/objectives.py` |

## Repository structure

```text
src/bimanual_redundancy/   Active package: core/ (math), simulation/ (MuJoCo
                            backend), experiments/, plotting/
models/                     Robot and object MJCF models
configs/                    Paper reproduction configs, robot profiles
tests/                      Top-level pytest suite
docs/                       Architecture, equation map, reproduction guides
legacy/                     Archived ROS 1/Gazebo/MoveIt code (reference only)
mujoco_curobo_bridge/       Pinned external bridge submodule
```

See [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) for the full package
breakdown and dependency direction.

## Hardware implementation

The manuscript also reports planar dual-arm hardware experiments. **That
hardware implementation is not distributed in this public repository**, and
the hardware results are not reproducible from this codebase — only the
spatial MuJoCo simulation study is.

For research inquiries regarding access to the planar hardware
implementation, please contact Barat S.:
https://www.linkedin.com/in/baratsuresh2811/

## Adding another robot

The redundancy-optimization framework is designed to support additional
cooperative robot embodiments through `CooperativeSystemSpec`, a minimal
specification interface that separates the generic manipulability
mathematics from any one robot's kinematics, actuation, or MuJoCo model. See
[`docs/ADDING_A_ROBOT.md`](docs/ADDING_A_ROBOT.md) for the extension
interface and checklist.

Contributions are welcome for:

- additional robot embodiments;
- cooperative grasp configurations;
- new task-specific objectives;
- analytical/autodiff gradient implementations; and
- benchmarking/reproduction tooling.

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for guidelines, and
[`docs/UPSTREAMING.md`](docs/UPSTREAMING.md) for which parts of the codebase
are generic enough to eventually be proposed to other projects.

## Citation

If you use this code, please cite the associated manuscript. Machine-readable
metadata is also available in [`CITATION.cff`](CITATION.cff).

```bibtex
@article{das2026taskspecific,
  title   = {Task-Specific Manipulability Metrics for Redundancy Optimization in Cooperative Manipulation},
  author  = {Das, Debojit and S., Barat and Palanthandalam-Madapusi, Harish J.},
  journal = {Industrial Robot: The International Journal of Robotics Research and Application},
  year    = {2026},
  note    = {Provisionally accepted}
}
```

## Development

After the [Quick start](#quick-start) setup, install the test dependencies
and run the suite from the repository root:

```bash
uv pip install -e ".[dev]"
pytest
```

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for environment setup details,
branch naming, and guidelines for adding robots, objectives, or new
mathematics.

## Limitations

- The grasp is generated by physical fingertip contact; there is no weld,
  teleport, or hidden object attachment.
- The mathematical model assumes fixed full contacts, while the simulation
  uses frictional contacts. This distinction should be reported when
  comparing results with the manuscript.
- The current spatial dual-Franka study is most mature for velocity
  manipulability. Force and directional-force objectives are implemented for
  comparison, but their spatial experimental interpretation remains ongoing.
- The manuscript's planar hardware experiments are not distributed or
  reproducible from this repository (see
  [Hardware implementation](#hardware-implementation)).
- Legacy ROS/Gazebo code requires a separate ROS 1 workspace and is not
  installed by the repository-local Python environment.

## License

Original code in this repository is released under the Apache License 2.0.
Third-party models, assets, dependencies, and submodules remain subject to
their respective licenses. See [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).
