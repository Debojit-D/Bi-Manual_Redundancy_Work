# Upstreaming guide

This repository is licensed under Apache-2.0 for its original code (see
[`LICENSE`](../LICENSE), [`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md)).
That makes some of its components reusable outside this project, but not all
of it — most of the code is intentionally specific to this manuscript's
experiments. This document describes the four layers the codebase is
organized into, and which of them are realistic candidates for future
contribution to other open-source projects (for example [Mink](
https://github.com/kevinzakka/mink), which this repository already depends
on).

## The four layers

1. **Paper-specific experiment/reproduction code** —
   `src/bimanual_redundancy/experiments/`, `src/bimanual_redundancy/paper_config.py`,
   `src/bimanual_redundancy/paper_reproduction.py`, `configs/paper/`. This
   implements the manuscript's specific trajectories, comparison sweeps, and
   CSV/plot outputs. It exists to reproduce one paper's results and is not
   intended for reuse outside that purpose.

2. **MuJoCo/backend-specific infrastructure** —
   `src/bimanual_redundancy/simulation/` (scene construction, cameras,
   collision-sphere overlays, video/CSV recording, control timing). This
   wires the mathematical core to MuJoCo specifically. It is reusable across
   different robots and experiments *within this project*, but it is
   MuJoCo-specific by design, not backend-agnostic.

3. **Robot-specific `CooperativeSystemSpec` integrations** —
   the per-robot specification/registration described in
   [`ADDING_A_ROBOT.md`](ADDING_A_ROBOT.md) (see
   `src/bimanual_redundancy/systems/`). Each entry adapts one robot's MJCF,
   joint layout, and grasp geometry to the generic interface the math layer
   expects. These are inherently tied to one robot's kinematic details.

4. **Mathematically generic reusable primitives** —
   `src/bimanual_redundancy/core/` (`cooperative_kinematics.py`,
   `controller.py`'s Equation (8) update, `objectives.py`'s manipulability
   costs, `directional_distance_optimization.py`). This layer is written
   against the `CooperativeSystemSpec` abstraction, not against MuJoCo or any
   one robot directly (see [`ARCHITECTURE.md`](ARCHITECTURE.md) for the
   enforced dependency direction).

**Layer 4 is the main candidate for possible future upstream contributions.**
Potential examples, if and when there is a fitting target library:

- manipulability objective primitives (velocity/force/directional-force
  costs, Eq. 13–17);
- null-space projection utilities;
- task-compatible capability metrics generalized beyond one robot pair;
- generic cooperative-task abstractions (grasp matrix / hand Jacobian
  composition for an arbitrary number of cooperating arms);
- analytical/autodiff gradient tooling to replace the current finite-difference
  gradients in `core/gradients.py`.

None of this is a claim that layer 4 is already packaged, tested, or
documented to the standard an upstream project like Mink would require. It
would need to be extracted, generalized beyond this manuscript's assumptions,
and adapted to the target project's own APIs, testing conventions, and
contribution process before a PR would make sense there.

## Relationship to Mink

Mink is, and will remain, an external dependency of this repository, not
something this repository forks or reimplements:

- this repository owns the paper-specific cooperative redundancy
  experiments — that scope stays here, not upstream;
- Mink remains an external IK dependency, used as-is via `pip`/`uv`, not
  vendored or modified in this repository (see
  [`THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md));
- generic functionality developed in layer 4 above may, in the future, be
  proposed upstream to Mink or a comparable project where it is a genuine
  conceptual fit — but only after being generalized and adapted to that
  project's own testing standards and contribution policy, and only with
  that project's maintainers' agreement;
- this repository does not fork or duplicate Mink's functionality, and the
  mathematical core (`core/`) does not require a Mink-specific interface —
  it is written against `CooperativeSystemSpec`, which is independent of any
  one IK library.
