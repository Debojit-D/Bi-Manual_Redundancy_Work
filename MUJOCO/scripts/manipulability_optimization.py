"""Task-specific manipulability objectives from Equations (13)-(15)."""

from dataclasses import dataclass
from enum import Enum

import mujoco
import numpy as np


class ManipulabilityObjective(str, Enum):
    """The three redundancy objectives compared in the paper."""

    VELOCITY = "velocity"
    FORCE = "force"
    DIRECTIONAL_FORCE = "directional_force"


@dataclass(frozen=True)
class OptimizationResult:
    objective: ManipulabilityObjective
    value: float
    gradient: np.ndarray
    phi_dot_opt: np.ndarray


class ManipulabilityOptimizer:
    """Evaluate paper costs and produce phi_dot_opt=Lambda*dW/dphi.

    Velocity and force manipulability are maximized.  Directional-force
    distance is minimized, so its gradient sign is reversed automatically.
    Gradients are evaluated with central joint-space finite differences.
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
    ):
        self.kinematics = kinematics
        self.model = kinematics.model
        self.arm_qpos_indices = np.asarray(arm_qpos_indices, dtype=int)
        self.objective = ManipulabilityObjective(objective)
        self.gain = float(gain)
        self.finite_difference_step = float(finite_difference_step)
        self.maximum_joint_speed = float(maximum_joint_speed)
        self.characteristic_length = float(characteristic_length)
        self.determinant_floor = float(determinant_floor)
        self.desired_force_matrix = self._make_direction_matrix(
            desired_wrench_direction
        )

        if self.arm_qpos_indices.size != self.kinematics.arm_dofs.size:
            raise ValueError("Expected one qpos index for every controlled DoF")
        if self.finite_difference_step <= 0.0:
            raise ValueError("finite_difference_step must be positive")
        if self.characteristic_length <= 0.0:
            raise ValueError("characteristic_length must be positive")

    def _make_direction_matrix(self, wrench_direction):
        weights = np.asarray(wrench_direction, dtype=float).copy()
        if weights.shape != (6,):
            raise ValueError("desired_wrench_direction must contain six values")
        # Equation (15) defines F=diag(Fx,Fy,Fz,Mx,My,Mz).  Manipulability
        # ellipsoids are sign-symmetric, so load-axis weights are nonnegative.
        # Spatial moment entries are converted to force-equivalent units first.
        weights = np.abs(weights)
        weights[3:] /= self.characteristic_length
        if np.sum(weights) <= 0.0:
            raise ValueError("desired_wrench_direction cannot be zero")
        return np.diag(weights)

    def velocity_manipulability(self, data):
        """Equation (13): sqrt(det(A A.T))."""
        velocity_map = self.kinematics.paper_object_velocity_map(data)
        return self._sqrt_determinant(velocity_map @ velocity_map.T)

    def force_manipulability(self, data):
        """Equation (14): sqrt(det((A A.T)^dagger))."""
        velocity_map = self.kinematics.paper_object_velocity_map(data)
        force_matrix = np.linalg.pinv(
            velocity_map @ velocity_map.T,
            rcond=self.kinematics.pinv_rcond,
        )
        return self._sqrt_determinant(force_matrix)

    def directional_force_cost(self, data):
        """Equation (15): normalized Frobenius directional distance."""
        velocity_map = self.kinematics.paper_object_velocity_map(data)
        capability = velocity_map @ velocity_map.T
        capability_trace = np.trace(capability)
        desired_trace = np.trace(self.desired_force_matrix)
        if capability_trace <= 0.0 or desired_trace <= 0.0:
            return float("inf")
        difference = (
            capability / capability_trace
            - self.desired_force_matrix / desired_trace
        )
        return float(np.linalg.norm(difference, ord="fro"))

    def _sqrt_determinant(self, matrix):
        sign, log_determinant = np.linalg.slogdet(matrix)
        if sign <= 0:
            return float(np.sqrt(self.determinant_floor))
        value = np.exp(0.5 * log_determinant)
        return float(max(value, np.sqrt(self.determinant_floor)))

    def value(self, data, objective=None):
        selected = ManipulabilityObjective(objective or self.objective)
        if selected is ManipulabilityObjective.VELOCITY:
            return self.velocity_manipulability(data)
        if selected is ManipulabilityObjective.FORCE:
            return self.force_manipulability(data)
        return self.directional_force_cost(data)

    def gradient(self, data, objective=None):
        """Return the central-difference derivative with respect to 14 joints."""
        selected = ManipulabilityObjective(objective or self.objective)
        step = self.finite_difference_step
        gradient = np.zeros(self.arm_qpos_indices.size)
        original_qpos = data.qpos.copy()

        try:
            for column, qpos_index in enumerate(self.arm_qpos_indices):
                center = original_qpos[qpos_index]

                data.qpos[qpos_index] = center + step
                mujoco.mj_forward(self.model, data)
                value_plus = self.value(data, selected)

                data.qpos[qpos_index] = center - step
                mujoco.mj_forward(self.model, data)
                value_minus = self.value(data, selected)

                gradient[column] = (value_plus - value_minus) / (2.0 * step)
                data.qpos[qpos_index] = center
        finally:
            data.qpos[:] = original_qpos
            mujoco.mj_forward(self.model, data)

        return gradient

    def optimization_velocity(self, data):
        """Return the signed and speed-limited phi_dot_opt from Equation (4)."""
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
