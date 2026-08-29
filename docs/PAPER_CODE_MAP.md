# Paper -> Code Map

Authoritative source: *Task-Specific Manipulability Metrics for Redundancy
Optimization in Cooperative Manipulation* (Das, Barat S., Palanthandalam-
Madapusi; IITGN Robotics Laboratory; *Industrial Robot*, final revision).
Equation numbers below are the manuscript's own numbers.

Goal: from any equation number here, jump straight to the implementing
function and its test.

## Quick index

| Eq. | What | Function | File | Test |
|---|---|---|---|---|
| 1 | `J_H phi_dot = G^T q_dot` | `grasp_matrix`, `hand_jacobian` | `core/cooperative_kinematics.py` | `tests/test_paper_equations.py` (Eq. 8 null-space tests exercise both) |
| 2 | `q_dot = A phi_dot` | `paper_object_velocity_map` | `core/cooperative_kinematics.py` | `tests/test_spatial_manipulability_scaling.py::test_scaling_matrix_and_velocity_map` |
| 4 | `phi_dot_opt = Lambda (dW/dphi)^T` | `ManipulabilityOptimizer.optimization_velocity`, `core.gradients.central_difference_gradient` | `core/objectives.py`, `core/gradients.py` | `tests/test_paper_equations.py::test_direct_is_minimized_indirect_is_maximized` |
| 8 | closed-loop tracking + `(I-J_H^dagger J_H)phi_dot_opt` | `Equation8Controller.update`, `CooperativeManipulationKinematics.null_space_projector` | `core/controller.py`, `core/cooperative_kinematics.py` | `tests/test_paper_equations.py::Equation8NullSpaceTests`, `tests/test_control_compute_timing.py` |
| 9 | grasp/contact compatibility constraint | `grasp_matrix`, `hand_jacobian` (same relation as Eq. 1) | `core/cooperative_kinematics.py` | — (see Eq. 1) |
| 10 | implemented object-velocity map `A` | `paper_object_velocity_map` | `core/cooperative_kinematics.py` | `tests/test_spatial_manipulability_scaling.py::test_velocity_and_force_capability_matrices` |
| 11 | unit joint-velocity bound `phi_dot^T phi_dot <= 1` | not separately implemented (see note below) | — | — |
| 12 | velocity manipulability ellipsoid `q_dot^T(AA^T)^dagger q_dot <= 1` | `ManipulabilityOptimizer.velocity_capability_matrices` (builds `AA^T`) | `core/objectives.py` | `tests/test_spatial_manipulability_scaling.py::test_velocity_and_force_capability_matrices` |
| 13 | `W_v = sqrt(det(A A^T))` | `ManipulabilityOptimizer.velocity_manipulability` (+ `_raw`/`_scaled`) | `core/objectives.py` | `tests/test_paper_equations.py::test_eq13_velocity_manipulability` |
| 14 | `W_f = sqrt(det((A A^T)^dagger))` | `ManipulabilityOptimizer.force_manipulability` (+ `_raw`/`_scaled`) | `core/objectives.py` | `tests/test_paper_equations.py::test_eq14_force_manipulability` |
| 15 | `F = diag(|Fx|,|Fy|,|Fz|,|Mx|,|My|,|Mz|)` | `ManipulabilityOptimizer._make_direction_matrices` (-> `desired_force_matrix_raw`) | `core/objectives.py` | `tests/test_paper_equations.py::test_eq15_desired_wrench_matrix_construction` |
| 16 | direct directional-force objective, **minimized** | `ManipulabilityOptimizer.directional_force_direct_cost` (+ `_raw`/`_scaled`) | `core/objectives.py` | `tests/test_paper_equations.py::test_eq16_direct_directional_force_cost`, `::test_direct_objective_uses_force_capability_not_velocity` |
| 17 | indirect directional-force objective, **maximized** | `ManipulabilityOptimizer.directional_force_indirect_cost` (+ `_raw`/`_scaled`) | `core/objectives.py` | `tests/test_paper_equations.py::test_eq17_indirect_directional_force_cost`, `::test_direct_objective_uses_force_capability_not_velocity` |
| App. A.1 | direct force-space derivation, trace normalization | `ManipulabilityOptimizer._normalized_frobenius_distance` | `core/objectives.py` | `tests/test_paper_equations.py::test_eq16_*` |
| App. A.2 | indirect velocity-dual derivation | same, applied to `AA^T` instead of `(AA^T)^dagger` | `core/objectives.py` | `tests/test_paper_equations.py::test_eq17_*` |

## Per-equation detail

### Eq. (1) `J_H phi_dot = G^T q_dot` and Eq. (9) (restated in Sec. 2.4)

Eq. (9) is the same relation as Eq. (1), restated in the paper's "Objective
Functions" section as "the contact compatibility constraint in object
grasping and cooperative manipulation tasks." Both are realized by the same
two functions, not a standalone "constraint" method:

- `CooperativeManipulationKinematics.grasp_matrix(data)` -> `G`, `R^(6x12)`.
- `CooperativeManipulationKinematics.hand_jacobian(data)` -> `J_H`,
  `R^(12x14)`.

Two different least-squares solutions of this one constraint are used
elsewhere in the code (see the Eq. (8) and Eq. (10) entries below) — this is
documented, not accidental.

### Eq. (2) `q_dot = A phi_dot` and Eq. (10) (restated in Sec. 2.4)

`CooperativeManipulationKinematics.paper_object_velocity_map(data)` returns
the raw paper quantity `A = (G^T)^dagger J_H`, `R^(6x14)`. This is the map
used to build every manipulability objective (Eq. 12-17) via
`ManipulabilityOptimizer.object_velocity_maps(data)`.

### Eq. (4) `phi_dot_opt = Lambda (dW/dphi)^T`

- `ManipulabilityOptimizer.gradient(data)` evaluates `dW/dphi` via
  `core.gradients.central_difference_gradient` (central joint-space finite
  differences, shared with `DirectionalDistancePermutationOptimizer`).
- `ManipulabilityOptimizer.optimization_velocity(data)` applies the sign
  (`Lambda`'s ascent/descent) and speed limit, returning
  `OptimizationResult.phi_dot_opt`.

### Eq. (8) `phi(t+dt) = phi(t) + A^dagger(q_dot_d + K_p e)dt + (I-J_H^dagger J_H)phi_dot_opt dt`

- Null-space projector `(I - J_H^dagger J_H)`:
  `CooperativeManipulationKinematics.null_space_projector(data)`,
  `R^(14x14)`.
- Full discrete update: `Equation8Controller.update(...)` in
  `core/controller.py`.
- **Implementation notes** (both documented in `update`'s docstring, not
  silently absorbed into "Eq. 8"):
  1. the tracking pseudoinverse uses `A_control =
     kinematics.tracking_object_velocity_map(data)`, a different
     least-squares solution of the same Eq. (1)/(9) constraint than the
     manipulability objectives' `A` (Eq. 10). Both solve
     `J_H phi_dot = G^T q_dot`; they are not the same matrix.
  2. an additional `q_dot_grasp` grasp-feedback correction term (active only
     when `grasp_feedback_gain` is set) compensates for position-level
     hand/object drift that Eq. (8)'s instantaneous-velocity form does not
     model. It is zero by default.
- Null-space property test: `J_H @ [(I - J_H^dagger J_H) phi_dot_opt] ~= 0`,
  verified numerically in
  `tests/test_paper_equations.py::Equation8NullSpaceTests` on a real grasped
  scene, and exercised as a live diagnostic
  (`Equation8Diagnostics.unscaled_null_space_leakage`) every controller step.

### Eq. (11) `phi_dot^T phi_dot <= 1`

The manuscript states this as the unit joint-velocity input used to derive
the velocity-manipulability ellipsoid (Eq. 12) from the object-velocity map
(Eq. 10): substituting `phi_dot = 1` direction vectors through `A` traces
out the ellipsoid `q_dot^T(AA^T)^dagger q_dot <= 1`. There is no standalone
function for Eq. (11) itself — it is a derivation step, not an implemented
quantity — and the codebase does not clamp `phi_dot` to the unit ball
anywhere. The actual joint-speed limiting is
`ManipulabilityOptimizer.maximum_joint_speed`, an independent
implementation-only bound on `phi_dot_opt` (not `phi_dot`, and not `<=1`
normalized), applied in `optimization_velocity`.

### Eq. (12) `q_dot^T (AA^T)^dagger q_dot <= 1`

The velocity-manipulability ellipsoid used as "the numerical manipulability
model in the controller." Implemented implicitly:
`ManipulabilityOptimizer.velocity_capability_matrices(data)` returns
`AA^T` (raw and scaled), and `force_capability_matrices` returns
`(AA^T)^dagger`; nothing evaluates the quadratic form itself since the
optimizer only needs the ellipsoid's *shape* (via Eq. 13's `sqrt(det(...))`
and Eq. 16/17's directional distance), not pointwise ellipsoid membership.

### Eq. (13) `W_v = sqrt(det(A A^T))` — velocity manipulability, **maximized**

`ManipulabilityOptimizer.velocity_manipulability(data)` (raw paper
quantity) / `velocity_manipulability_raw` (same) /
`velocity_manipulability_scaled` (spatial diagnostic on `A_scaled`, **not**
part of Eq. 13). `A A^T` is `R^(6x6)`.

### Eq. (14) `W_f = sqrt(det((A A^T)^dagger))` — force manipulability, **maximized**

`ManipulabilityOptimizer.force_manipulability(data)` /
`force_manipulability_raw` / `force_manipulability_scaled` (diagnostic).
`(AA^T)^dagger` is `R^(6x6)`, equal to `(AA^T)^-1` for full-row-rank `A`.

### Eq. (15) `F = diag(|Fx|,|Fy|,|Fz|,|Mx|,|My|,|Mz|)`

`ManipulabilityOptimizer._make_direction_matrices(wrench_direction)`, called
from `__init__` with the `desired_wrench_direction` constructor argument,
producing `self.desired_force_matrix_raw` (Eq. 15 itself) and
`self.desired_force_matrix_scaled` (diagnostic: moment entries additionally
divided by the characteristic length `l`, not part of Eq. 15).

### Eq. (16) direct directional-force objective — **minimized**

```
W_df^direct = || (AA^T)^dagger/tr((AA^T)^dagger) - F/tr(F) ||_F
```

`ManipulabilityOptimizer.directional_force_direct_cost(data)` (+ `_raw` /
`_scaled`). Operates on the **force-capability matrix**
`(AA^T)^dagger` (Eq. 14's matrix), not the velocity-capability matrix.
`ManipulabilityObjective.DIRECTIONAL_FORCE` selects it. This is the
formulation used by the spatial simulations' primary comparisons (see
"Planar vs. spatial usage" below).

**Naming note**: the old method name `directional_force_cost` did not say
*which* directional-force formulation it computed — see "Naming
correction" below.

### Eq. (17) indirect directional-force objective — **maximized**

```
W_df^indirect = || AA^T/tr(AA^T) - F/tr(F) ||_F
```

`ManipulabilityOptimizer.directional_force_indirect_cost(data)` (+ `_raw` /
`_scaled`). Operates on the **velocity-capability matrix** `AA^T` (Eq. 13's
matrix), not the force-capability matrix.
`ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT` selects it. This is the
formulation used by the planar hardware experiments (see "Planar vs.
spatial usage" below). Per Appendix A.2: "The direct and indirect
formulations share the same task-level motivation but are not mathematically
equivalent" — they generally produce different objective gradients,
null-space motions, and stationary configurations even under the same
prescribed load direction.

## Planar vs. spatial usage (paper implementation notes)

Quoting the manuscript directly, since this is exactly the distinction the
codebase must preserve and previously did not document:

> "The planar hardware experiments employ the indirect velocity-dual
> formulation in equation (17), whereas the spatial directional-force
> simulations employ the direct force-space formulation in equation (16).
> Both formulations are additionally evaluated under matched conditions in
> the static spatial comparison."

> "The direct directional-force formulation is used in the primary
> comparisons, while the indirect formulation is additionally evaluated in
> the static and six-dimensional cases."

In this repository (the spatial MuJoCo study):

- `ManipulabilityObjective.DIRECTIONAL_FORCE` (Eq. 16, direct) is the
  default/primary directional-force mode across the spatial experiment
  scripts (`dual_franka_eq8_*`).
- `ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT` (Eq. 17, indirect) is
  additionally exercised in the static and six-dimensional comparisons —
  e.g. `experiments/dual_franka_eq8_static_comparison.py`,
  `experiments/dual_franka_eq8_6d_pick_place_comparison.py`,
  `experiments/comparison_main.py`, and
  `experiments/add_directional_force_indirect_to_batch.py` — matching the
  manuscript's "additionally evaluated in the static and six-dimensional
  cases."
- Do not add or modify which cases use which formulation; the above mirrors
  what the manuscript already specifies and what the code already does.

## Naming correction

**Before this refactor**, `ManipulabilityOptimizer` exposed
`directional_force_cost(data)` with no indication of which manuscript
equation it computed. The top-level `README.md` correspondingly described
"Directional force" as a single "normalized Frobenius distance between
`A A.T` and `F`" — which is actually Eq. (17)'s *indirect* formula, even
though the surrounding text described it as the thing that is "minimized
automatically" (true only of Eq. 16, the *direct* formula, which is what
`directional_force_cost`/`ManipulabilityObjective.DIRECTIONAL_FORCE` has
always actually computed). The manuscript defines both Eq. (16) and Eq. (17)
as genuinely different, non-equivalent objectives (Appendix A.2); collapsing
them into one undifferentiated "directional force" concept was incomplete
documentation, not a code bug — the code computed the right thing under an
ambiguous name.

**Fixed in this refactor**:

- `README.md`'s "Manipulability objectives" section now states both Eq. (16)
  and Eq. (17) explicitly, with which is minimized/maximized and which
  capability matrix each uses.
- New unambiguous core-API method names:
  - `directional_force_direct_cost(data)` / `_raw` / `_scaled` — Eq. (16).
  - `directional_force_indirect_cost(data)` / `_raw` / `_scaled` — Eq. (17)
    (name unchanged; it was already unambiguous).
- The old `directional_force_cost` / `directional_force_cost_raw` /
  `directional_force_cost_scaled` names are kept as deprecated aliases
  (`DeprecationWarning`, calling straight through to the `_direct_cost*`
  methods) so existing external callers are not broken immediately.
- **Not renamed**: the CSV column names (`directional_force_cost`,
  `directional_force_cost_raw`, `directional_force_cost_scaled` in
  `simulation/recording/csv_recorder.py`'s `CSV_COLUMNS`) and every plotting
  script that reads those columns from historical CSV files on disk. Those
  are a persisted data format, not the core API this refactor's naming
  section addresses; renaming them would silently break every previously
  recorded dataset under `outputs/mujoco_data/`. Live method calls inside
  `csv_recorder.py` were updated to call the new
  `directional_force_direct_cost_raw/_scaled` methods; the dict keys they
  write are unchanged.
- `ManipulabilityObjective.DIRECTIONAL_FORCE` / `DIRECTIONAL_FORCE_INDIRECT`
  enum member names, and their string values (`"directional_force"` /
  `"directional_force_indirect"`, used as CLI `--objective` choices and CSV
  `optimization_mode`/`objective` values) were **not** renamed: the pairing
  is already unambiguous, and the string values are also a persisted format
  (recorded CSVs, output directory names).

## Math vs. gradients vs. collision penalties

Previously, `core/objectives.py` mixed three concerns in one ~1000-line
class: the paper's manipulability mathematics, finite-difference gradient
evaluation, and collision/safety penalty shaping. The manuscript is explicit
that collisions are not part of the manipulability metrics: "The
manipulability metrics do not account for collisions... In the spatial
simulations, a standard potential-based collision penalty was added to the
null-space objective." This refactor splits accordingly, without changing
any formula:

- `core/objectives.py` — Eq. (12)-(17) only: capability matrices,
  manipulability/directional-force costs, `value()`/`optimization_velocity()`
  dispatch. Now inspectable without scrolling through collision code.
- `core/gradients.py` — `central_difference_gradient`, the finite-difference
  mechanics behind Eq. (4)'s `dW/dphi`, shared by `ManipulabilityOptimizer`
  and `DirectionalDistancePermutationOptimizer` (previously duplicated).
- `core/collision_penalties.py` — `CollisionPenaltiesMixin`, the added-on
  penalty described above (inter-arm, arm-table, self-collision). Mixed
  into `ManipulabilityOptimizer` (`class ManipulabilityOptimizer
  (CollisionPenaltiesMixin)`) so every existing public attribute and method
  (`left_table_body_ids`, `inter_arm_collision_cost(data)`, ...) is
  unchanged for external callers; only the source file moved.

A paper objective is directly evaluable without constructing any collision
state: `ManipulabilityOptimizer(kinematics, arm_qpos_indices,
objective=...)` with all `enable_*_collision_penalty` flags left at their
default `False` never touches `core/collision_penalties.py` beyond
resolving two body-name lists in `__init__`.

## `core.directional_distance_optimization` — not a numbered equation

`DirectionalDistancePermutationOptimizer` (in
`core/directional_distance_optimization.py`) generalizes Eq. (16)/(17) into
four exploratory permutations (capability matrix x distance direction). Only
`DirectionalDistanceCase.FORCE_MINIMIZE` reproduces Eq. (16) and only
`VELOCITY_MAXIMIZE` reproduces Eq. (17); `FORCE_MAXIMIZE` and
`VELOCITY_MINIMIZE` have no corresponding manuscript equation and are
experimental controls only (see the module's own docstring). It is not
substituted into the paper's `ManipulabilityOptimizer` automatically and is
not used by any of the manuscript's reported results.

## Known code/paper discrepancies (flagged, not silently fixed)

- **Eq. (8)'s tracking map vs. Eq. (10)'s objective map.** As noted above,
  `Equation8Controller.update` solves the primary tracking term with
  `kinematics.tracking_object_velocity_map(data)`, a different
  least-squares solution of the Eq. (1)/(9) constraint than the `A` (Eq. 10)
  used by every manipulability objective. This was already true before this
  refactor and is intentional per the existing code comments ("This is kept
  separate from the paper map used by the manipulability objectives"), but
  the manuscript's Eq. (8) does not itself distinguish two different `A`
  matrices — a reader matching Eq. (8) line-for-line against `update()`
  should know `A^dagger` there is not literally `paper_object_velocity_map`.
- **`q_dot_grasp` term.** `Equation8Controller.update` adds a grasp-feedback
  correction term not present in the manuscript's Eq. (8). It is zero unless
  `grasp_feedback_gain` is explicitly configured, and exists to correct
  physical drift that Eq. (8)'s idealized instantaneous-velocity model does
  not capture (see the controller's docstring). Not changed by this
  refactor; flagged here per the task's request to surface, not silently
  patch, discrepancies.
- **Eq. (11) has no standalone implementation.** It is a derivation step
  (unit joint-velocity input used to derive the Eq. 12 ellipsoid from Eq. 10)
  rather than a quantity the code evaluates directly; see the Eq. (11)
  section above. `maximum_joint_speed` is a related but distinct
  implementation-only safety bound, not a rendering of Eq. (11).
- **`*_scaled` diagnostics are not paper quantities.** Every `*_scaled`
  method (`velocity_manipulability_scaled`, `force_manipulability_scaled`,
  `directional_force_direct_cost_scaled`,
  `directional_force_indirect_cost_scaled`) exists for CSV
  characteristic-length comparisons only; only the unsuffixed and `*_raw`
  methods are the manuscript's actual Eq. 13/14/16/17 quantities. This was
  already the case; this refactor's docstrings now say so explicitly on
  every method instead of only in the class-level docstring.
