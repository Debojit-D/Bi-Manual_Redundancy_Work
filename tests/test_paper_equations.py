"""Equation-level tests for Equations (8) and (12)-(17).

Each test independently recomputes a paper quantity from plain numpy on a
synthetic, deterministic matrix (or a real grasped scene for Equation (8))
and asserts the implementation matches. See ``docs/PAPER_CODE_MAP.md`` for
the paper -> code map these tests exercise.
"""

from types import SimpleNamespace
import unittest

import numpy as np

from bimanual_redundancy.core import (
    CooperativeManipulationKinematics,
    ManipulabilityObjective,
    ManipulabilityOptimizer,
)
from bimanual_redundancy.simulation import DualFrankaMuJoCoScene


class _FakeNamedObject:
    def __init__(self, object_id=0):
        self.id = object_id


class _FakeModel:
    def body(self, _name):
        return _FakeNamedObject()


class _FakeKinematics:
    """Returns a fixed synthetic object-velocity map A, bypassing MuJoCo."""

    def __init__(self, velocity_map):
        self.model = _FakeModel()
        self.velocity_map = np.asarray(velocity_map, dtype=float)
        self.arm_dofs = np.arange(self.velocity_map.shape[1])
        self.pinv_rcond = 1e-9

    def paper_object_velocity_map(self, _data):
        return self.velocity_map.copy()


def _synthetic_A():
    """A deterministic, full-row-rank 6x14 object-velocity map."""
    matrix = np.zeros((6, 14))
    matrix[:, :6] = np.diag([2.0, 1.5, 1.1, 0.9, 1.3, 0.7])
    matrix[:, 6:] = (np.arange(48, dtype=float).reshape(6, 8) - 24.0) / 37.0
    return matrix


def _optimizer(objective, wrench_direction=(0.0, 0.0, 1.0, 0.0, 0.0, 0.0)):
    return ManipulabilityOptimizer(
        _FakeKinematics(_synthetic_A()),
        np.arange(14),
        objective=objective,
        desired_wrench_direction=wrench_direction,
        maximum_joint_speed=100.0,
    )


class ManipulabilityEquationTests(unittest.TestCase):
    """Eq. 13-17 recomputed independently from plain numpy."""

    def test_eq13_velocity_manipulability(self):
        A = _synthetic_A()
        optimizer = _optimizer(ManipulabilityObjective.VELOCITY)
        expected = np.sqrt(np.linalg.det(A @ A.T))
        self.assertAlmostEqual(
            optimizer.velocity_manipulability(None), expected, places=9
        )

    def test_eq14_force_manipulability(self):
        A = _synthetic_A()
        optimizer = _optimizer(ManipulabilityObjective.FORCE)
        expected = np.sqrt(np.linalg.det(np.linalg.pinv(A @ A.T)))
        self.assertAlmostEqual(
            optimizer.force_manipulability(None), expected, places=9
        )

    def test_eq15_desired_wrench_matrix_construction(self):
        direction = (3.0, -4.0, 0.0, 0.0, 2.0, 0.0)
        optimizer = _optimizer(
            ManipulabilityObjective.FORCE, wrench_direction=direction
        )
        expected = np.diag(np.abs(np.asarray(direction)))
        np.testing.assert_allclose(optimizer.desired_force_matrix_raw, expected)

    def test_eq16_direct_directional_force_cost(self):
        A = _synthetic_A()
        direction = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
        optimizer = _optimizer(
            ManipulabilityObjective.DIRECTIONAL_FORCE,
            wrench_direction=direction,
        )
        force_capability = np.linalg.pinv(A @ A.T)
        F = np.diag(np.abs(np.asarray(direction)))
        expected = np.linalg.norm(
            force_capability / np.trace(force_capability) - F / np.trace(F),
            ord="fro",
        )
        self.assertAlmostEqual(
            optimizer.directional_force_direct_cost(None), expected, places=9
        )

    def test_eq17_indirect_directional_force_cost(self):
        A = _synthetic_A()
        direction = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
        optimizer = _optimizer(
            ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT,
            wrench_direction=direction,
        )
        velocity_capability = A @ A.T
        F = np.diag(np.abs(np.asarray(direction)))
        expected = np.linalg.norm(
            velocity_capability / np.trace(velocity_capability)
            - F / np.trace(F),
            ord="fro",
        )
        self.assertAlmostEqual(
            optimizer.directional_force_indirect_cost(None),
            expected,
            places=9,
        )

    def test_direct_objective_uses_force_capability_not_velocity(self):
        """Eq. 16 must depend on (AA^T)^dagger, not AA^T: on the same A and
        F, the direct cost matches only the force-capability ground truth
        and the indirect cost matches only the velocity-capability one."""
        A = _synthetic_A()
        direction = (0.0, 0.0, 1.0, 0.0, 0.0, 0.0)
        direct = _optimizer(
            ManipulabilityObjective.DIRECTIONAL_FORCE,
            wrench_direction=direction,
        )
        indirect = _optimizer(
            ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT,
            wrench_direction=direction,
        )
        force_capability = np.linalg.pinv(A @ A.T)
        velocity_capability = A @ A.T
        F = np.diag(np.abs(np.asarray(direction)))
        expected_direct = np.linalg.norm(
            force_capability / np.trace(force_capability) - F / np.trace(F),
            ord="fro",
        )
        expected_indirect = np.linalg.norm(
            velocity_capability / np.trace(velocity_capability)
            - F / np.trace(F),
            ord="fro",
        )
        direct_value = direct.directional_force_direct_cost(None)
        indirect_value = indirect.directional_force_indirect_cost(None)

        self.assertAlmostEqual(direct_value, expected_direct, places=9)
        self.assertAlmostEqual(indirect_value, expected_indirect, places=9)
        # The two capability matrices are not equal here, so a direct
        # objective computed on the wrong (velocity) matrix would not match
        # expected_direct, and vice versa.
        self.assertFalse(
            np.isclose(direct_value, expected_indirect, atol=1e-6)
        )
        self.assertFalse(
            np.isclose(indirect_value, expected_direct, atol=1e-6)
        )

    def test_direct_is_minimized_indirect_is_maximized(self):
        """Equation (4): phi_dot_opt = Lambda*(dW/dphi)^T. The optimizer's
        gain ascends the gradient by default; Eq. (16) (minimized) must flip
        that sign, and Eq. (17) (maximized) must not."""
        direct = _optimizer(ManipulabilityObjective.DIRECTIONAL_FORCE)
        indirect = _optimizer(ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT)
        fixed_gradient = np.linspace(0.1, 1.4, 14)
        direct.gradient = lambda data: fixed_gradient
        indirect.gradient = lambda data: fixed_gradient
        direct.value = lambda data: 0.0
        indirect.value = lambda data: 0.0

        direct_result = direct.optimization_velocity(None)
        indirect_result = indirect.optimization_velocity(None)

        np.testing.assert_allclose(direct_result.phi_dot_opt, -fixed_gradient)
        np.testing.assert_allclose(indirect_result.phi_dot_opt, fixed_gradient)


class Equation8NullSpaceTests(unittest.TestCase):
    """Numerically verify J_H @ [(I - J_H^dagger J_H) phi_dot_opt] ~= 0."""

    @classmethod
    def setUpClass(cls):
        cls.scene = DualFrankaMuJoCoScene()
        cls.kinematics = CooperativeManipulationKinematics(
            cls.scene.model,
            cls.scene.left_arm_dofs,
            cls.scene.right_arm_dofs,
        )

    def test_null_space_term_produces_zero_hand_velocity(self):
        data = self.scene.data
        hand_jacobian = self.kinematics.hand_jacobian(data)
        projector = self.kinematics.null_space_projector(data)

        phi_dot_opt = np.linspace(-0.7, 0.9, hand_jacobian.shape[1])
        projected_null_space_velocity = projector @ phi_dot_opt
        residual = hand_jacobian @ projected_null_space_velocity

        np.testing.assert_allclose(
            residual,
            np.zeros(hand_jacobian.shape[0]),
            atol=1e-9,
        )

    def test_projector_is_idempotent(self):
        """(I - J_H^dagger J_H) is a projector, so applying it twice is the
        same as applying it once -- a basic sanity check on the same object
        used by the null-space test above."""
        projector = self.kinematics.null_space_projector(self.scene.data)
        np.testing.assert_allclose(
            projector @ projector,
            projector,
            atol=1e-9,
        )


if __name__ == "__main__":
    unittest.main()
