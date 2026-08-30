# Paper -> Code Map

Authoritative source: *Task-Specific Manipulability Metrics for Redundancy
Optimization in Cooperative Manipulation* (Das, Barat S., Palanthandalam-
Madapusi; IITGN Robotics Laboratory; *Industrial Robot*, final revision).
Equation numbers below are the manuscript's own numbers.

## Quick index

| Eq. | What | Function | File | Test |
|---|---|---|---|---|
| 1 | `J_H phi_dot = G^T q_dot` | `grasp_matrix`, `hand_jacobian` | `core/cooperative_kinematics.py` | `tests/test_paper_equations.py` (Eq. 8 null-space tests exercise both) |
| 2 | `q_dot = A phi_dot` | `paper_object_velocity_map` | `core/cooperative_kinematics.py` | `tests/test_spatial_manipulability_scaling.py::test_scaling_matrix_and_velocity_map` |
| 4 | `phi_dot_opt = Lambda (dW/dphi)^T` | `ManipulabilityOptimizer.optimization_velocity`, `core.gradients.central_difference_gradient` | `core/objectives.py`, `core/gradients.py` | `tests/test_paper_equations.py::test_direct_is_minimized_indirect_is_maximized` |
| 8 | closed-loop tracking + `(I-J_H^dagger J_H)phi_dot_opt` | `Equation8Controller.update`, `CooperativeManipulationKinematics.null_space_projector` | `core/controller.py`, `core/cooperative_kinematics.py` | `tests/test_paper_equations.py::Equation8NullSpaceTests`, `tests/test_control_compute_timing.py` |
| 9 | grasp/contact compatibility constraint | `grasp_matrix`, `hand_jacobian` (same relation as Eq. 1) | `core/cooperative_kinematics.py` | see Eq. 1 |
| 10 | implemented object-velocity map `A` | `paper_object_velocity_map` | `core/cooperative_kinematics.py` | `tests/test_spatial_manipulability_scaling.py::test_velocity_and_force_capability_matrices` |
| 11 | unit joint-velocity bound `phi_dot^T phi_dot <= 1` | derivation step only, see below | - | - |
| 12 | velocity manipulability ellipsoid `q_dot^T(AA^T)^dagger q_dot <= 1` | `ManipulabilityOptimizer.velocity_capability_matrices` (builds `AA^T`) | `core/objectives.py` | `tests/test_spatial_manipulability_scaling.py::test_velocity_and_force_capability_matrices` |
| 13 | `W_v = sqrt(det(A A^T))`, velocity manipulability, maximized | `ManipulabilityOptimizer.velocity_manipulability` (+ `_raw`/`_scaled`) | `core/objectives.py` | `tests/test_paper_equations.py::test_eq13_velocity_manipulability` |
| 14 | `W_f = sqrt(det((A A^T)^dagger))`, force manipulability, maximized | `ManipulabilityOptimizer.force_manipulability` (+ `_raw`/`_scaled`) | `core/objectives.py` | `tests/test_paper_equations.py::test_eq14_force_manipulability` |
| 15 | `F = diag(\|Fx\|,\|Fy\|,\|Fz\|,\|Mx\|,\|My\|,\|Mz\|)` | `ManipulabilityOptimizer._make_direction_matrices` | `core/objectives.py` | `tests/test_paper_equations.py::test_eq15_desired_wrench_matrix_construction` |
| 16 | direct directional-force objective, minimized | `ManipulabilityOptimizer.directional_force_direct_cost` (+ `_raw`/`_scaled`) | `core/objectives.py` | `tests/test_paper_equations.py::test_eq16_direct_directional_force_cost` |
| 17 | indirect directional-force objective, maximized | `ManipulabilityOptimizer.directional_force_indirect_cost` (+ `_raw`/`_scaled`) | `core/objectives.py` | `tests/test_paper_equations.py::test_eq17_indirect_directional_force_cost` |
| App. A.1 | direct force-space derivation, trace normalization | `ManipulabilityOptimizer._normalized_frobenius_distance` | `core/objectives.py` | `tests/test_paper_equations.py::test_eq16_*` |
| App. A.2 | indirect velocity-dual derivation | same, applied to `AA^T` instead of `(AA^T)^dagger` | `core/objectives.py` | `tests/test_paper_equations.py::test_eq17_*` |

## Per-equation notes

**Eq. (1) / Eq. (9)** (restated in Sec. 2.4 as the contact compatibility
constraint): both realized by `CooperativeManipulationKinematics.grasp_matrix(data)`
(`G`, `R^(6x12)`) and `.hand_jacobian(data)` (`J_H`, `R^(12x14)`). Two
different least-squares solutions of this constraint are used elsewhere in
the code; see the Eq. (8) note below.

**Eq. (2) / Eq. (10)**: `paper_object_velocity_map(data)` returns
`A = (G^T)^dagger J_H`, `R^(6x14)`, the map used to build every
manipulability objective (Eq. 12-17).

**Eq. (4)**: `ManipulabilityOptimizer.gradient(data)` evaluates `dW/dphi` via
`core.gradients.central_difference_gradient` (central joint-space finite
differences, shared with `DirectionalDistancePermutationOptimizer`).
`optimization_velocity(data)` applies the ascent/descent sign and speed
limit, returning `OptimizationResult.phi_dot_opt`.

**Eq. (8)**: null-space projector `(I - J_H^dagger J_H)` is
`CooperativeManipulationKinematics.null_space_projector(data)`; the full
discrete update is `Equation8Controller.update(...)`. Two implementation
details worth knowing when matching this against the manuscript line for
line:

1. the tracking pseudoinverse uses `A_control =
   kinematics.tracking_object_velocity_map(data)`, a different
   least-squares solution of the Eq. (1)/(9) constraint than the
   manipulability objectives' `A` (Eq. 10). Both solve
   `J_H phi_dot = G^T q_dot`; they are generally not equal matrices.
2. `Equation8Controller.update` adds an optional `q_dot_grasp` term beyond
   the manuscript's formula:

   ```
   phi_next = phi + [A_control^dagger(q_dot_d + K_p e) + q_dot_grasp
                      + (I-J_H^dagger J_H)phi_dot_opt] dt
   ```

   `q_dot_grasp = J_H^dagger (grasp_feedback_gain @ grasp_pose_error)`
   compensates position-level hand/object drift from integration, actuator
   lag, and contact compliance that Eq. (8)'s instantaneous-velocity form
   does not model. It only modifies the tracking term, never the null-space
   term or any manipulability objective. Default gain is `None` (disabled);
   `dual_franka_eq8_optimized_pick_place.py`,
   `dual_franka_eq8_optimized_6d_pick_place.py`,
   `dual_franka_eq8_static_optimization.py`, and
   `dual_franka_eq8_directional_distance_comparison.py` enable it
   (`GRASP_K_P = diag([8, 8, 8, 6, 6, 6])`);
   `dual_franka_eq8_baseline_pick_place.py` does not. The comparison wrapper
   scripts call into these runners' `main()` rather than constructing their
   own controller, so they inherit whichever setting the wrapped runner
   uses.

The null-space property `J_H @ [(I - J_H^dagger J_H) phi_dot_opt] ~= 0` is
verified numerically in
`tests/test_paper_equations.py::Equation8NullSpaceTests` and checked as a
live diagnostic (`Equation8Diagnostics.unscaled_null_space_leakage`) every
controller step.

**Eq. (11)**: a derivation step, not an implemented quantity. Substituting
unit `phi_dot` direction vectors through `A` (Eq. 10) traces the
velocity-manipulability ellipsoid of Eq. (12); the code never clamps
`phi_dot` to the unit ball. `ManipulabilityOptimizer.maximum_joint_speed` is
an unrelated implementation-only safety bound on `phi_dot_opt`.

**Eq. (12)**: implemented implicitly. `velocity_capability_matrices(data)`
returns `AA^T`; `force_capability_matrices` returns `(AA^T)^dagger`. Nothing
evaluates the quadratic form directly since the optimizer only needs the
ellipsoid's shape (via Eq. 13's `sqrt(det(...))` and Eq. 16/17's directional
distance), not pointwise membership.

**Eq. (13)/(14)**: `velocity_manipulability(data)` / `force_manipulability(data)`,
each with a `_raw` alias (identical) and a `_scaled` diagnostic (evaluated on
`A_scaled`, not a paper quantity; see "Implementation notes" below).

**Eq. (15)**: `_make_direction_matrices(wrench_direction)`, called from
`__init__` with the `desired_wrench_direction` constructor argument,
producing `desired_force_matrix_raw` (Eq. 15 itself) and
`desired_force_matrix_scaled` (diagnostic).

**Eq. (16)**, direct directional-force, minimized:

```
W_df^direct = || (AA^T)^dagger/tr((AA^T)^dagger) - F/tr(F) ||_F
```

`directional_force_direct_cost(data)` operates on the force-capability
matrix `(AA^T)^dagger`. Selected by
`ManipulabilityObjective.DIRECTIONAL_FORCE`; the default/primary
directional-force mode across the spatial experiment scripts.

**Eq. (17)**, indirect directional-force, maximized:

```
W_df^indirect = || AA^T/tr(AA^T) - F/tr(F) ||_F
```

`directional_force_indirect_cost(data)` operates on the velocity-capability
matrix `AA^T`. Selected by
`ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT`; this is the
formulation used by the manuscript's separate planar hardware experiments
(Appendix A.2 notes the direct and indirect formulations are not
mathematically equivalent, even under the same prescribed load direction).

## Planar vs. spatial usage

The manuscript reports both planar physical-hardware experiments and
spatial MuJoCo simulation experiments. This repository implements and
reproduces the spatial simulation study; the planar hardware implementation
is not distributed here (see the README's "Hardware implementation"
section).

> "The planar hardware experiments employ the indirect velocity-dual
> formulation in equation (17), whereas the spatial directional-force
> simulations employ the direct force-space formulation in equation (16).
> Both formulations are additionally evaluated under matched conditions in
> the static spatial comparison."

In the spatial MuJoCo study: `DIRECTIONAL_FORCE` (Eq. 16, direct) is the
default/primary directional-force mode across `dual_franka_eq8_*` scripts;
`DIRECTIONAL_FORCE_INDIRECT` (Eq. 17, indirect) is additionally exercised in
the static and six-dimensional comparisons
(`dual_franka_eq8_static_comparison.py`,
`dual_franka_eq8_6d_pick_place_comparison.py`, `comparison_main.py`,
`add_directional_force_indirect_to_batch.py`).

## Implementation notes

- **Deprecated aliases.** `directional_force_cost(data)` / `_raw` / `_scaled`
  remain as deprecated wrappers (`DeprecationWarning`) around
  `directional_force_direct_cost(data)` / `_raw` / `_scaled`. CSV column
  names (`directional_force_cost*` in `csv_recorder.py`'s `CSV_COLUMNS`) and
  the CLI/enum string values (`"directional_force"`,
  `"directional_force_indirect"`) are unchanged, since they are a persisted
  on-disk format used by every previously recorded dataset under
  `outputs/mujoco_data/`.
- **`*_scaled` methods are not paper quantities.** Every `*_scaled` method
  (`velocity_manipulability_scaled`, `force_manipulability_scaled`,
  `directional_force_direct_cost_scaled`,
  `directional_force_indirect_cost_scaled`) exists for CSV
  characteristic-length comparisons only; the unsuffixed and `*_raw` methods
  are the manuscript's actual Eq. 13/14/16/17 quantities.
- **Collision penalties are separate from the objectives.** The manuscript
  states the manipulability metrics do not account for collisions, and that
  a standard potential-based collision penalty is added to the null-space
  objective in the spatial simulations. `core/objectives.py` implements only
  Eq. (12)-(17); `core/collision_penalties.py`'s `CollisionPenaltiesMixin`
  (inter-arm, arm-table, self-collision) is mixed into
  `ManipulabilityOptimizer` separately. A paper objective is evaluable
  without constructing any collision state: `ManipulabilityOptimizer(...)`
  with all `enable_*_collision_penalty` flags left at their default `False`
  never touches `collision_penalties.py` beyond resolving two body-name
  lists in `__init__`.
- **`core.directional_distance_optimization` is not a numbered equation.**
  `DirectionalDistancePermutationOptimizer` generalizes Eq. (16)/(17) into
  four exploratory permutations (capability matrix x distance direction).
  Only `DirectionalDistanceCase.FORCE_MINIMIZE` reproduces Eq. (16) and only
  `VELOCITY_MAXIMIZE` reproduces Eq. (17); `FORCE_MAXIMIZE` and
  `VELOCITY_MINIMIZE` have no corresponding manuscript equation and are
  experimental controls only. It is not used by any manuscript-reported
  result.
