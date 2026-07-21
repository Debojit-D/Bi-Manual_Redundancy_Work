"""Experimental 2x2 directional-distance optimization permutations.

This module intentionally lives beside, rather than inside, the paper's
``ManipulabilityOptimizer``. It tests two independent choices:

1. capability matrix: ``A A.T`` or ``(A A.T)^dagger``;
2. distance direction: minimize or maximize the normalized Frobenius distance.

All four cases maximize one signed score internally::

    score = distance_sign * directional_distance - collision_penalty

``distance_sign`` is -1 for minimization and +1 for maximization. Therefore
collision avoidance retains the same sign in every experimental permutation.
"""

from dataclasses import dataclass
from enum import Enum

import mujoco
import numpy as np

from .manipulability_optimization import (
    CollisionModelVersion,
    ManipulabilityObjective,
    ManipulabilityOptimizer,
)


class CapabilityMatrixKind(str, Enum):
    """Matrix normalized before comparison with the desired direction."""

    VELOCITY = "velocity"  # C_v = A A.T
    FORCE = "force"  # C_f = (A A.T)^dagger


class DistanceDirection(str, Enum):
    """Whether the Frobenius distance is minimized or maximized."""

    MINIMIZE = "minimize"
    MAXIMIZE = "maximize"


class DirectionalDistanceCase(str, Enum):
    """The four capability-matrix/distance-direction permutations."""

    FORCE_MINIMIZE = "force_minimize"
    FORCE_MAXIMIZE = "force_maximize"
    VELOCITY_MINIMIZE = "velocity_minimize"
    VELOCITY_MAXIMIZE = "velocity_maximize"

    @property
    def capability_kind(self):
        if self in (self.FORCE_MINIMIZE, self.FORCE_MAXIMIZE):
            return CapabilityMatrixKind.FORCE
        return CapabilityMatrixKind.VELOCITY

    @property
    def distance_direction(self):
        if self in (self.FORCE_MINIMIZE, self.VELOCITY_MINIMIZE):
            return DistanceDirection.MINIMIZE
        return DistanceDirection.MAXIMIZE

    @property
    def distance_sign(self):
        return (
            -1.0
            if self.distance_direction is DistanceDirection.MINIMIZE
            else 1.0
        )


@dataclass(frozen=True)
class DirectionalDistanceOptimizationResult:
    """One signed-gradient evaluation of an experimental permutation."""

    objective: DirectionalDistanceCase
    value: float
    score: float
    gradient: np.ndarray
    phi_dot_opt: np.ndarray

    @property
    def distance(self):
        """Unmodified normalized Frobenius distance."""
        return self.value


class DirectionalDistancePermutationOptimizer(ManipulabilityOptimizer):
    """Optimize any of the four directional-distance test permutations.

    ``FORCE_MINIMIZE`` is the reviewer-safe force-capability alignment case.
    The other three cases are experimental controls and are not substituted
    into the paper optimizer automatically.
    """

    def __init__(
        self,
        kinematics,
        arm_qpos_indices,
        *,
        case=DirectionalDistanceCase.FORCE_MINIMIZE,
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
    ):
        super().__init__(
            kinematics,
            arm_qpos_indices,
            # The parent objective is used only during parent initialization.
            objective=ManipulabilityObjective.VELOCITY,
            gain=gain,
            finite_difference_step=finite_difference_step,
            maximum_joint_speed=maximum_joint_speed,
            desired_wrench_direction=desired_wrench_direction,
            characteristic_length=characteristic_length,
            determinant_floor=determinant_floor,
            enable_collision_penalty=enable_collision_penalty,
            collision_weight=collision_weight,
            collision_safety_margin=collision_safety_margin,
            collision_proximity_scale=collision_proximity_scale,
            collision_sphere_radius=collision_sphere_radius,
            collision_version=collision_version,
            collision_sphere_model_path=collision_sphere_model_path,
        )
        self.case = DirectionalDistanceCase(case)
        # Preserve the ``optimizer.objective.value`` interface used by recorders.
        self.objective = self.case

    def capability_matrix(self, data):
        """Return C_v=A A.T or C_f=(A A.T)^dagger for the selected case."""
        velocity_map = self.kinematics.paper_object_velocity_map(data)
        velocity_capability = velocity_map @ velocity_map.T
        if self.case.capability_kind is CapabilityMatrixKind.FORCE:
            return np.linalg.pinv(
                velocity_capability,
                rcond=self.kinematics.pinv_rcond,
            )
        return velocity_capability

    @staticmethod
    def normalized_frobenius_distance(capability, desired):
        """Return ||C/tr(C) - F_d/tr(F_d)||_F."""
        capability = np.asarray(capability, dtype=float)
        desired = np.asarray(desired, dtype=float)
        if capability.shape != desired.shape:
            raise ValueError("capability and desired matrices must match")
        capability_trace = float(np.trace(capability))
        desired_trace = float(np.trace(desired))
        if capability_trace <= 0.0 or desired_trace <= 0.0:
            return float("inf")
        difference = (
            capability / capability_trace - desired / desired_trace
        )
        return float(np.linalg.norm(difference, ord="fro"))

    def directional_distance(self, data):
        """Return the raw, unsigned distance D(q) for the selected matrix."""
        return self.normalized_frobenius_distance(
            self.capability_matrix(data),
            self.desired_force_matrix,
        )

    def value(self, data, objective=None):
        """Return raw D(q); ``objective`` is rejected to avoid ambiguity."""
        if objective is not None and DirectionalDistanceCase(objective) is not self.case:
            raise ValueError(
                "Create a separate optimizer instance to evaluate another case"
            )
        return self.directional_distance(data)

    def optimization_score(self, data):
        """Return the scalar score that is always maximized by the controller."""
        score = self.case.distance_sign * self.directional_distance(data)
        if self.enable_collision_penalty:
            score -= self.collision_weight * self.inter_arm_collision_cost(data)
        return float(score)

    def gradient(self, data, objective=None):
        """Central-difference gradient of the signed optimization score."""
        if objective is not None and DirectionalDistanceCase(objective) is not self.case:
            raise ValueError(
                "Create a separate optimizer instance to evaluate another case"
            )
        step = self.finite_difference_step
        gradient = np.zeros(self.arm_qpos_indices.size)
        original_qpos = data.qpos.copy()

        try:
            for column, qpos_index in enumerate(self.arm_qpos_indices):
                center = original_qpos[qpos_index]

                data.qpos[qpos_index] = center + step
                mujoco.mj_forward(self.model, data)
                score_plus = self.optimization_score(data)

                data.qpos[qpos_index] = center - step
                mujoco.mj_forward(self.model, data)
                score_minus = self.optimization_score(data)

                gradient[column] = (score_plus - score_minus) / (2.0 * step)
                data.qpos[qpos_index] = center
        finally:
            data.qpos[:] = original_qpos
            mujoco.mj_forward(self.model, data)

        return gradient

    def optimization_velocity(self, data):
        """Return the speed-limited ascent direction for the signed score."""
        distance = self.directional_distance(data)
        score = self.optimization_score(data)
        gradient = self.gradient(data)
        phi_dot_opt = self.gain * gradient

        peak_speed = np.max(np.abs(phi_dot_opt))
        if peak_speed > self.maximum_joint_speed:
            phi_dot_opt *= self.maximum_joint_speed / peak_speed

        return DirectionalDistanceOptimizationResult(
            objective=self.case,
            value=distance,
            score=score,
            gradient=gradient,
            phi_dot_opt=phi_dot_opt,
        )
