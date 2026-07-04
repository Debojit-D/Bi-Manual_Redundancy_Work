"""Tests for the experimental directional-distance permutation definitions."""

import unittest

import numpy as np

from MUJOCO.utils.redundancy_optimization import (
    CapabilityMatrixKind,
    DirectionalDistanceCase,
    DirectionalDistancePermutationOptimizer,
    DistanceDirection,
)


class DirectionalDistancePermutationTests(unittest.TestCase):
    def test_four_cases_map_to_expected_axes(self):
        expected = {
            DirectionalDistanceCase.FORCE_MINIMIZE: (
                CapabilityMatrixKind.FORCE,
                DistanceDirection.MINIMIZE,
                -1.0,
            ),
            DirectionalDistanceCase.FORCE_MAXIMIZE: (
                CapabilityMatrixKind.FORCE,
                DistanceDirection.MAXIMIZE,
                1.0,
            ),
            DirectionalDistanceCase.VELOCITY_MINIMIZE: (
                CapabilityMatrixKind.VELOCITY,
                DistanceDirection.MINIMIZE,
                -1.0,
            ),
            DirectionalDistanceCase.VELOCITY_MAXIMIZE: (
                CapabilityMatrixKind.VELOCITY,
                DistanceDirection.MAXIMIZE,
                1.0,
            ),
        }
        for case, mapping in expected.items():
            self.assertEqual(case.capability_kind, mapping[0])
            self.assertEqual(case.distance_direction, mapping[1])
            self.assertEqual(case.distance_sign, mapping[2])

    def test_identical_normalized_matrices_have_zero_distance(self):
        matrix = np.diag([2.0, 1.0, 0.5])
        distance = (
            DirectionalDistancePermutationOptimizer.normalized_frobenius_distance(
                matrix,
                matrix,
            )
        )
        self.assertAlmostEqual(distance, 0.0)

    def test_orthogonal_unit_directions_have_sqrt_two_distance(self):
        first = np.diag([1.0, 0.0])
        second = np.diag([0.0, 1.0])
        distance = (
            DirectionalDistancePermutationOptimizer.normalized_frobenius_distance(
                first,
                second,
            )
        )
        self.assertAlmostEqual(distance, np.sqrt(2.0))


if __name__ == "__main__":
    unittest.main()
