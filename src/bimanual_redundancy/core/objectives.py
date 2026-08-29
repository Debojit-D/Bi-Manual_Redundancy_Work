"""Task-specific manipulability objectives: Equations (12)-(17).

Pure paper mathematics only. Collision penalties/safety shaping live in
``core.collision_penalties`` (mixed into :class:`ManipulabilityOptimizer`
below so its public surface is unchanged); finite-difference gradient
mechanics live in ``core.gradients``. See ``docs/PAPER_CODE_MAP.md`` for the
full equation-to-function map.
"""

from dataclasses import dataclass
from enum import Enum
import warnings

import numpy as np

from .collision_penalties import (
    LEFT_COLLISION_BODIES,
    RIGHT_COLLISION_BODIES,
    CollisionModelVersion,
    CollisionPenaltiesMixin,
)
from .gradients import central_difference_gradient


class ManipulabilityObjective(str, Enum):
    """The supported redundancy objectives.

    ``DIRECTIONAL_FORCE`` is the Equation (16) *direct* force-space
    formulation; ``DIRECTIONAL_FORCE_INDIRECT`` is the Equation (17)
    *indirect* velocity-dual formulation. They are not mathematically
    equivalent (Appendix A.2) and are evaluated on different capability
    matrices.
    """

    VELOCITY = "velocity"
    FORCE = "force"
    DIRECTIONAL_FORCE = "directional_force"
    DIRECTIONAL_FORCE_INDIRECT = "directional_force_indirect"


@dataclass(frozen=True)
class OptimizationResult:
    objective: ManipulabilityObjective
    value: float
    gradient: np.ndarray
    phi_dot_opt: np.ndarray


class ManipulabilityOptimizer(CollisionPenaltiesMixin):
    """Evaluate paper objectives (Eq. 12-17) and produce Equation (4)'s
    ``phi_dot_opt`` null-space motion.

    The raw spatial map is ``A_raw = A`` (Eq. 10). With characteristic
    length ``l``, ``S_l = diag(1, 1, 1, l, l, l)`` and
    ``A_scaled = S_l A_raw``. The active controller objectives use the
    original raw quantities; scaled spatial capabilities are retained as
    diagnostics for CSV analysis only and are not part of the paper
    formulation (see ``*_scaled`` methods below).

    Velocity manipulability (Eq. 13) and force manipulability (Eq. 14) are
    maximized. The direct directional-force distance (Eq. 16) compares the
    force-capability matrix with the desired wrench direction and is
    minimized. The indirect directional-force distance (Eq. 17) compares the
    velocity-capability matrix with that direction and is maximized.
    Gradients (Eq. 4's ``dW/dphi``) are evaluated with central joint-space
    finite differences via ``core.gradients.central_difference_gradient``.

    Unsuffixed public objective methods (``velocity_manipulability``,
    ``force_manipulability``, ``directional_force_direct_cost``,
    ``directional_force_indirect_cost``) preserve the original raw
    behavior. Explicit ``*_raw`` and ``*_scaled`` methods support
    comparative reporting. ``directional_force_cost*`` names are deprecated
    aliases for the ``*_direct_cost*`` names below and will be removed in a
    future release; they exist because ``directional_force_cost`` alone
    does not say whether it means Equation (16) or Equation (17).
    """

    def __init__(
        self,
        kinematics,
        arm_qpos_indices,
        *,
        objective=ManipulabilityObjective.VELOCITY,
        gain=1.0,
        finite_difference_step=1e-4,
        maximum_joint_speed=0.15,
        desired_wrench_direction=(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        characteristic_length=0.4,
        determinant_floor=1e-18,
        enable_collision_penalty=False,
        collision_weight=25.0,
        collision_safety_margin=0.08,
        collision_proximity_scale=0.01,
        collision_sphere_radius=0.07,
        collision_version=CollisionModelVersion.VERSION1,
        collision_sphere_model_path=None,
        enable_table_collision_penalty=False,
        table_collision_weight=20000.0,
        table_collision_safety_margin=0.015,
        table_collision_proximity_scale=0.003,
        table_collision_geom_name=None,
        enable_self_collision_penalty=False,
        self_collision_weight=20000.0,
        self_collision_safety_margin=0.01,
        self_collision_proximity_scale=0.003,
    ):
        self.kinematics = kinematics
        self.model = kinematics.model
        self.arm_qpos_indices = np.asarray(arm_qpos_indices, dtype=int)
        self.objective = ManipulabilityObjective(objective)
        self.gain = float(gain)
        self.finite_difference_step = float(finite_difference_step)
        self.maximum_joint_speed = float(maximum_joint_speed)
        self.characteristic_length = float(characteristic_length)
        if self.characteristic_length <= 0.0:
            raise ValueError("characteristic_length must be positive")
        self.determinant_floor = float(determinant_floor)
        self.enable_collision_penalty = bool(enable_collision_penalty)
        self.collision_weight = float(collision_weight)
        self.collision_safety_margin = float(collision_safety_margin)
        self.collision_proximity_scale = float(collision_proximity_scale)
        self.collision_sphere_radius = float(collision_sphere_radius)
        self.collision_version = CollisionModelVersion(collision_version)
        self.enable_table_collision_penalty = bool(
            enable_table_collision_penalty
        )
        self.table_collision_weight = float(table_collision_weight)
        self.table_collision_safety_margin = float(
            table_collision_safety_margin
        )
        self.table_collision_proximity_scale = float(
            table_collision_proximity_scale
        )
        self.table_collision_geom_name = table_collision_geom_name
        self.table_collision_geom_id = -1
        self.enable_self_collision_penalty = bool(
            enable_self_collision_penalty
        )
        self.self_collision_weight = float(self_collision_weight)
        self.self_collision_safety_margin = float(
            self_collision_safety_margin
        )
        self.self_collision_proximity_scale = float(
            self_collision_proximity_scale
        )
        # Resolve names once: finite-difference evaluations reuse these IDs.
        self.left_collision_body_ids = np.array(
            [self.model.body(name).id for name in LEFT_COLLISION_BODIES],
            dtype=int,
        )
        self.right_collision_body_ids = np.array(
            [self.model.body(name).id for name in RIGHT_COLLISION_BODIES],
            dtype=int,
        )
        self.left_detailed_body_ids = np.empty(0, dtype=int)
        self.left_detailed_centers = np.empty((0, 3))
        self.left_detailed_radii = np.empty(0)
        self.right_detailed_body_ids = np.empty(0, dtype=int)
        self.right_detailed_centers = np.empty((0, 3))
        self.right_detailed_radii = np.empty(0)
        self.left_table_body_ids = np.empty(0, dtype=int)
        self.left_table_centers = np.empty((0, 3))
        self.left_table_radii = np.empty(0)
        self.right_table_body_ids = np.empty(0, dtype=int)
        self.right_table_centers = np.empty((0, 3))
        self.right_table_radii = np.empty(0)
        self.left_self_collision_pairs = np.empty((0, 2), dtype=int)
        self.right_self_collision_pairs = np.empty((0, 2), dtype=int)
        if (
            self.collision_version is CollisionModelVersion.VERSION2
            or self.enable_table_collision_penalty
            or self.enable_self_collision_penalty
        ):
            self._load_detailed_collision_spheres(collision_sphere_model_path)
        if self.enable_table_collision_penalty:
            self.table_collision_geom_id = self._resolve_table_collision_geom(
                table_collision_geom_name
            )
        (
            self.desired_force_matrix_raw,
            self.desired_force_matrix_scaled,
        ) = self._make_direction_matrices(
            desired_wrench_direction
        )
        # Backward-compatible name used by the raw controller objective.
        self.desired_force_matrix = self.desired_force_matrix_raw

        if self.arm_qpos_indices.size != self.kinematics.arm_dofs.size:
            raise ValueError("Expected one qpos index for every controlled DoF")
        if self.finite_difference_step <= 0.0:
            raise ValueError("finite_difference_step must be positive")
        if self.collision_weight < 0.0:
            raise ValueError("collision_weight must be nonnegative")
        if self.collision_safety_margin < 0.0:
            raise ValueError("collision_safety_margin must be nonnegative")
        if self.collision_proximity_scale <= 0.0:
            raise ValueError("collision_proximity_scale must be positive")
        if self.collision_sphere_radius < 0.0:
            raise ValueError("collision_sphere_radius must be nonnegative")
        if self.table_collision_weight < 0.0:
            raise ValueError("table_collision_weight must be nonnegative")
        if self.table_collision_safety_margin < 0.0:
            raise ValueError(
                "table_collision_safety_margin must be nonnegative"
            )
        if self.table_collision_proximity_scale <= 0.0:
            raise ValueError(
                "table_collision_proximity_scale must be positive"
            )
        if self.self_collision_weight < 0.0:
            raise ValueError("self_collision_weight must be nonnegative")
        if self.self_collision_safety_margin < 0.0:
            raise ValueError(
                "self_collision_safety_margin must be nonnegative"
            )
        if self.self_collision_proximity_scale <= 0.0:
            raise ValueError(
                "self_collision_proximity_scale must be positive"
            )

    def _make_direction_matrices(self, wrench_direction):
        """Return raw and dual-scaled diagonal wrench weighting matrices.

        Equation (15): ``F = diag(|Fx|, |Fy|, |Fz|, |Mx|, |My|, |Mz|)``,
        6x6, built from the supplied ``desired_wrench_direction``. Load-axis
        weights are nonnegative because the manipulability ellipsoids are
        sign-symmetric. The scaled variant additionally divides moment
        entries by the characteristic length (``q_dot_scaled=S_l q_dot``
        implies ``w_scaled=S_l^-T w``); it is a spatial diagnostic, not part
        of Equation (15) itself.
        """
        raw_weights = np.asarray(wrench_direction, dtype=float).copy()
        if raw_weights.shape != (6,):
            raise ValueError("desired_wrench_direction must contain six values")
        raw_weights = np.abs(raw_weights)
        if np.sum(raw_weights) <= 0.0:
            raise ValueError("desired_wrench_direction cannot be zero")
        scaled_weights = raw_weights.copy()
        scaled_weights[3:] /= self.characteristic_length
        return np.diag(raw_weights), np.diag(scaled_weights)

    def spatial_scaling_matrix(self):
        """Return ``S_l = diag(1, 1, 1, l, l, l)``, 6x6.

        Not a paper equation; a diagnostic scaling used only to build the
        ``*_scaled`` spatial-comparison quantities below.
        """
        return np.diag(
            [1.0, 1.0, 1.0]
            + [self.characteristic_length] * 3
        )

    def object_velocity_maps(self, data):
        """Return ``A_raw=A`` and ``A_scaled=S_l A_raw``.

        Equation (10): ``A_raw`` is the raw paper object-velocity map,
        ``A = (G.T)^dagger J_H`` (Eq. 1, 2, 10), shape ``(6, N)`` for ``N``
        controlled joints. ``A_scaled`` is a spatial diagnostic, not part of
        Equation (10).
        """
        raw_map = np.asarray(
            self.kinematics.paper_object_velocity_map(data),
            dtype=float,
        )
        scaled_map = self.spatial_scaling_matrix() @ raw_map
        return raw_map, scaled_map

    @staticmethod
    def _velocity_capabilities_from_maps(raw_map, scaled_map):
        return raw_map @ raw_map.T, scaled_map @ scaled_map.T

    def velocity_capability_matrices(self, data):
        """Return raw and scaled velocity capabilities ``A A.T``, each 6x6.

        The raw matrix is the ``A A^T`` of Equations (12) and (13): the
        velocity-capability matrix whose ellipsoid
        ``q_dot^T (AA^T)^dagger q_dot <= 1`` (Eq. 12) is the numerical
        manipulability model, and whose determinant gives ``W_v`` (Eq. 13).
        The scaled matrix is a spatial diagnostic built from ``A_scaled``.
        """
        return self._velocity_capabilities_from_maps(
            *self.object_velocity_maps(data)
        )

    def _force_capabilities_from_velocity(self, raw_matrix, scaled_matrix):
        rcond = self.kinematics.pinv_rcond
        return (
            np.linalg.pinv(raw_matrix, rcond=rcond),
            np.linalg.pinv(scaled_matrix, rcond=rcond),
        )

    def force_capability_matrices(self, data):
        """Return ``Mf_raw=pinv(Mv_raw)`` and ``Mf_scaled=pinv(Mv_scaled)``.

        The raw matrix is ``(AA^T)^dagger`` from Equation (14) (equal to
        ``(AA^T)^-1`` for full-row-rank ``A``, by velocity-force duality),
        each 6x6. The scaled matrix is a spatial diagnostic.
        """
        return self._force_capabilities_from_velocity(
            *self.velocity_capability_matrices(data)
        )

    def spatial_capability_matrices(self, data):
        """Return ``A_raw, A_scaled, Mv_raw, Mv_scaled, Mf_raw, Mf_scaled``."""
        raw_map, scaled_map = self.object_velocity_maps(data)
        velocity_raw, velocity_scaled = (
            self._velocity_capabilities_from_maps(raw_map, scaled_map)
        )
        force_raw, force_scaled = self._force_capabilities_from_velocity(
            velocity_raw,
            velocity_scaled,
        )
        return (
            raw_map,
            scaled_map,
            velocity_raw,
            velocity_scaled,
            force_raw,
            force_scaled,
        )

    @staticmethod
    def velocity_map_diagnostics(velocity_map):
        """Return rank, extrema of singular values, and a safe condition number."""
        velocity_map = np.asarray(velocity_map, dtype=float)
        singular_values = np.linalg.svd(velocity_map, compute_uv=False)
        rank = int(np.linalg.matrix_rank(velocity_map))
        if singular_values.size == 0:
            return rank, 0.0, 0.0, float("inf")
        sigma_max = float(singular_values[0])
        sigma_min = float(singular_values[-1])
        tolerance = (
            np.finfo(float).eps
            * max(velocity_map.shape, default=0)
            * sigma_max
        )
        condition = (
            float("inf")
            if sigma_min <= tolerance
            else sigma_max / sigma_min
        )
        return rank, sigma_min, sigma_max, float(condition)

    def velocity_manipulability_raw(self, data):
        """Return the raw paper quantity ``W_v = sqrt(det(A A.T))``.

        Equation (13). Scalar. Maximized. ``A A.T`` is 6x6 (``m x m`` for
        task-space dimension ``m``); larger values indicate greater overall
        task-space velocity capability.
        """
        raw_matrix, _ = self.velocity_capability_matrices(data)
        return self._sqrt_determinant(raw_matrix)

    def velocity_manipulability_scaled(self, data):
        """Return diagnostic ``sqrt(det(Mv_scaled))`` for CSV comparison.

        Not part of Equation (13); a spatial-scaling diagnostic evaluated on
        ``A_scaled`` instead of the raw paper map ``A``.
        """
        _, scaled_matrix = self.velocity_capability_matrices(data)
        return self._sqrt_determinant(scaled_matrix)

    def velocity_manipulability(self, data):
        """Return raw velocity manipulability (Eq. 13) used by the optimizer."""
        return self.velocity_manipulability_raw(data)

    def force_manipulability_raw(self, data):
        """Return the raw paper quantity ``W_f = sqrt(det((A A.T)^dagger))``.

        Equation (14). Scalar. Maximized. By velocity-force duality the
        velocity- and force-capability ellipsoids share principal directions
        with reciprocal axis lengths, so maximum force capability corresponds
        to minimum velocity capability.
        """
        raw_matrix, _ = self.force_capability_matrices(data)
        return self._sqrt_determinant(raw_matrix)

    def force_manipulability_scaled(self, data):
        """Return diagnostic ``sqrt(det(Mf_scaled))`` for CSV comparison.

        Not part of Equation (14); a spatial-scaling diagnostic.
        """
        _, scaled_matrix = self.force_capability_matrices(data)
        return self._sqrt_determinant(scaled_matrix)

    def force_manipulability(self, data):
        """Return raw force manipulability (Eq. 14) used by the optimizer."""
        return self.force_manipulability_raw(data)

    @staticmethod
    def _normalized_frobenius_distance(capability, desired):
        capability_trace = float(np.trace(capability))
        desired_trace = float(np.trace(desired))
        if capability_trace <= 0.0 or desired_trace <= 0.0:
            return float("inf")
        difference = (
            capability / capability_trace
            - desired / desired_trace
        )
        return float(np.linalg.norm(difference, ord="fro"))

    def directional_force_direct_cost_raw(self, data):
        """Return the raw paper quantity ``W_df^direct`` (Equation 16).

        ``|| (AA^T)^dagger/tr((AA^T)^dagger) - F/tr(F) ||_F``: the
        trace-normalized Frobenius distance between the normalized
        force-capability matrix and the normalized desired-wrench matrix
        ``F`` (Eq. 15). Scalar. Minimized (Appendix A.1). Compares in
        force-capability space, i.e. it depends on ``(AA^T)^dagger``, not
        ``AA^T`` -- see ``directional_force_indirect_cost_raw`` for the
        velocity-space (Eq. 17) counterpart.
        """
        force_raw, _ = self.force_capability_matrices(data)
        return self._normalized_frobenius_distance(
            force_raw,
            self.desired_force_matrix_raw,
        )

    def directional_force_direct_cost_scaled(self, data):
        """Return diagnostic direct directional-force distance in scaled
        coordinates. Not part of Equation (16); a spatial diagnostic.
        """
        _, force_scaled = self.force_capability_matrices(data)
        return self._normalized_frobenius_distance(
            force_scaled,
            self.desired_force_matrix_scaled,
        )

    def directional_force_direct_cost(self, data):
        """Return raw direct directional-force distance (Eq. 16), minimized."""
        return self.directional_force_direct_cost_raw(data)

    def directional_force_cost_raw(self, data):
        """Deprecated alias for :meth:`directional_force_direct_cost_raw`."""
        warnings.warn(
            "directional_force_cost_raw is deprecated; use "
            "directional_force_direct_cost_raw (Equation 16) instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.directional_force_direct_cost_raw(data)

    def directional_force_cost_scaled(self, data):
        """Deprecated alias for :meth:`directional_force_direct_cost_scaled`."""
        warnings.warn(
            "directional_force_cost_scaled is deprecated; use "
            "directional_force_direct_cost_scaled (Equation 16) instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.directional_force_direct_cost_scaled(data)

    def directional_force_cost(self, data):
        """Deprecated alias for :meth:`directional_force_direct_cost`."""
        warnings.warn(
            "directional_force_cost is deprecated and ambiguous between "
            "Equation (16) direct and Equation (17) indirect; use "
            "directional_force_direct_cost instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        return self.directional_force_direct_cost(data)

    def directional_force_indirect_cost_raw(self, data):
        """Return the raw paper quantity ``W_df^indirect`` (Equation 17).

        ``|| AA^T/tr(AA^T) - F/tr(F) ||_F``: the trace-normalized Frobenius
        distance between the normalized velocity-capability matrix and the
        normalized desired-wrench matrix ``F`` (Eq. 15). Scalar. Maximized
        (Appendix A.2) -- maximizing this distance separates the normalized
        velocity-capability structure from the prescribed load direction,
        shaping the configuration indirectly through velocity-force duality.
        Compares in velocity-capability space, i.e. it depends on ``AA^T``,
        not ``(AA^T)^dagger``.
        """
        velocity_raw, _ = self.velocity_capability_matrices(data)
        return self._normalized_frobenius_distance(
            velocity_raw,
            self.desired_force_matrix_raw,
        )

    def directional_force_indirect_cost_scaled(self, data):
        """Return scaled velocity-capability directional distance.

        Not part of Equation (17); a spatial diagnostic.
        """
        _, velocity_scaled = self.velocity_capability_matrices(data)
        return self._normalized_frobenius_distance(
            velocity_scaled,
            self.desired_force_matrix_scaled,
        )

    def directional_force_indirect_cost(self, data):
        """Return raw indirect directional-force distance (Eq. 17), maximized."""
        return self.directional_force_indirect_cost_raw(data)

    def paper_objective_values(self, data):
        """Return raw and scaled selected paper metrics before collision costs."""
        if self.objective is ManipulabilityObjective.VELOCITY:
            return (
                self.velocity_manipulability_raw(data),
                self.velocity_manipulability_scaled(data),
            )
        if self.objective is ManipulabilityObjective.FORCE:
            return (
                self.force_manipulability_raw(data),
                self.force_manipulability_scaled(data),
            )
        if (
            self.objective
            is ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT
        ):
            return (
                self.directional_force_indirect_cost_raw(data),
                self.directional_force_indirect_cost_scaled(data),
            )
        return (
            self.directional_force_direct_cost_raw(data),
            self.directional_force_direct_cost_scaled(data),
        )

    def _sqrt_determinant(self, matrix):
        sign, log_determinant = np.linalg.slogdet(matrix)
        if sign <= 0:
            return float(np.sqrt(self.determinant_floor))
        value = np.exp(0.5 * log_determinant)
        return float(max(value, np.sqrt(self.determinant_floor)))

    def value(self, data, objective=None):
        selected = ManipulabilityObjective(objective or self.objective)
        collision_cost = self.total_collision_cost(data)
        if selected is ManipulabilityObjective.VELOCITY:
            value = self.velocity_manipulability(data)
        elif selected is ManipulabilityObjective.FORCE:
            value = self.force_manipulability(data)
        elif selected is ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT:
            value = self.directional_force_indirect_cost(data)
        else:
            # optimization_velocity() reverses this gradient to minimize the
            # directional distance. Adding collision cost here therefore
            # minimizes both directional error and collision violations.
            return self.directional_force_direct_cost(data) + collision_cost

        return value - collision_cost

    def gradient(self, data, objective=None):
        """Return the central-difference derivative with respect to N joints.

        Numerical realization of Equation (4)'s ``dW/dphi`` for whichever
        objective ``W`` is selected; see ``core.gradients`` for the shared
        finite-difference mechanics.
        """
        selected = ManipulabilityObjective(objective or self.objective)
        return central_difference_gradient(
            self.model,
            data,
            self.arm_qpos_indices,
            self.finite_difference_step,
            lambda evaluated_data: self.value(evaluated_data, selected),
        )

    def optimization_velocity(self, data):
        """Return the signed and speed-limited phi_dot_opt from Equation (4).

        ``phi_dot_opt = Lambda (dW/dphi)^T`` (Eq. 4). The sign of the
        gradient is flipped for the direct directional-force objective
        (Eq. 16, minimized); all other objectives ascend their gradient
        directly (Eq. 13, 14 maximized; Eq. 17 maximized).
        """
        objective_value = self.value(data)
        gradient = self.gradient(data)
        direction_sign = (
            -1.0
            if self.objective is ManipulabilityObjective.DIRECTIONAL_FORCE
            else 1.0
        )
        phi_dot_opt = direction_sign * self.gain * gradient

        peak_speed = np.max(np.abs(phi_dot_opt))
        if peak_speed > self.maximum_joint_speed:
            phi_dot_opt *= self.maximum_joint_speed / peak_speed

        return OptimizationResult(
            objective=self.objective,
            value=objective_value,
            gradient=gradient,
            phi_dot_opt=phi_dot_opt,
        )
