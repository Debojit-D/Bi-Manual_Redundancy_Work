"""Tests for rigid-grasp characteristic-length initialization."""

import importlib
from types import SimpleNamespace
import sys
import unittest
from unittest.mock import patch

import numpy as np

from bimanual_redundancy.core import (
    CooperativeManipulationKinematics,
)


class _NamedSite:
    def __init__(self, site_id):
        self.id = site_id


class _SiteModel:
    def __init__(self):
        self.site_ids = {
            "site_top_middle": 0,
            "site_left": 1,
            "site_right": 2,
        }

    def site(self, name):
        return _NamedSite(self.site_ids[name])


def _kinematics_and_data(object_position, left_position, right_position):
    kinematics = CooperativeManipulationKinematics(
        _SiteModel(),
        np.arange(7),
        np.arange(7, 14),
        object_reference_site="site_top_middle",
        object_contact_sites=("site_left", "site_right"),
        hand_sites=("left_hand", "right_hand"),
    )
    data = SimpleNamespace(
        site_xpos=np.asarray(
            [object_position, left_position, right_position],
            dtype=float,
        )
    )
    return kinematics, data


class GraspCharacteristicLengthTests(unittest.TestCase):
    def test_symmetric_contacts_equal_half_contact_separation(self):
        kinematics, data = _kinematics_and_data(
            [0.0, 0.0, 0.0],
            [-0.2, 0.0, 0.0],
            [0.2, 0.0, 0.0],
        )
        characteristic_length = kinematics.grasp_characteristic_length(data)
        contact_separation = np.linalg.norm(
            data.site_xpos[2] - data.site_xpos[1]
        )
        self.assertAlmostEqual(
            characteristic_length,
            0.5 * contact_separation,
        )

    def test_asymmetric_reference_uses_rms_contact_radius(self):
        kinematics, data = _kinematics_and_data(
            [0.0, 0.0, 0.0],
            [0.1, 0.0, 0.0],
            [0.4, 0.0, 0.0],
        )
        expected = np.sqrt((0.1**2 + 0.4**2) / 2.0)
        length, midpoint, midpoint_distance = (
            kinematics.grasp_characteristic_length_diagnostics(data)
        )
        self.assertAlmostEqual(length, expected)
        np.testing.assert_allclose(midpoint, [0.25, 0.0, 0.0])
        self.assertAlmostEqual(midpoint_distance, 0.25)

    def test_invariant_under_rigid_translation_and_rotation(self):
        object_position = np.array([0.1, -0.2, 0.3])
        left_position = np.array([-0.3, 0.2, 0.4])
        right_position = np.array([0.5, 0.1, -0.1])
        kinematics, original_data = _kinematics_and_data(
            object_position,
            left_position,
            right_position,
        )
        original_length, _, original_midpoint_distance = (
            kinematics.grasp_characteristic_length_diagnostics(original_data)
        )

        rotation = np.array(
            [
                [0.0, -1.0, 0.0],
                [1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0],
            ]
        )
        translation = np.array([3.0, -4.0, 2.0])
        transformed = (
            original_data.site_xpos @ rotation.T + translation
        )
        transformed_data = SimpleNamespace(site_xpos=transformed)
        transformed_length, _, transformed_midpoint_distance = (
            kinematics.grasp_characteristic_length_diagnostics(
                transformed_data
            )
        )
        self.assertAlmostEqual(transformed_length, original_length)
        self.assertAlmostEqual(
            transformed_midpoint_distance,
            original_midpoint_distance,
        )

    def test_manual_override_is_selected_without_changing_computed_value(self):
        kinematics, data = _kinematics_and_data(
            [0.0, 0.0, 0.0],
            [-0.2, 0.0, 0.0],
            [0.2, 0.0, 0.0],
        )
        selected, computed, midpoint, midpoint_distance = (
            kinematics.resolve_characteristic_length(data, 0.73)
        )
        self.assertEqual(selected, 0.73)
        self.assertAlmostEqual(computed, 0.2)
        np.testing.assert_allclose(midpoint, np.zeros(3))
        self.assertEqual(midpoint_distance, 0.0)

        automatic, computed_again, _, _ = (
            kinematics.resolve_characteristic_length(data)
        )
        self.assertEqual(automatic, computed_again)

    def test_all_relevant_entry_points_accept_manual_override(self):
        module_names = (
            "bimanual_redundancy.experiments.dual_franka_eq8_static_optimization",
            "bimanual_redundancy.experiments.dual_franka_eq8_optimized_pick_place",
            "bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place",
            "bimanual_redundancy.experiments.dual_franka_eq8_static_comparison",
            "bimanual_redundancy.experiments.dual_franka_eq8_pick_place_comparison",
            "bimanual_redundancy.experiments.dual_franka_eq8_6d_pick_place_comparison",
            "bimanual_redundancy.experiments.dual_franka_eq8_directional_distance_comparison",
        )
        for module_name in module_names:
            module = importlib.import_module(module_name)
            with self.subTest(module=module_name, mode="automatic"), patch.object(
                sys,
                "argv",
                [module_name],
            ):
                arguments = module.parse_arguments()
                self.assertIsNone(arguments.characteristic_length)
            with self.subTest(module=module_name), patch.object(
                sys,
                "argv",
                [module_name, "--characteristic-length", "0.73"],
            ):
                arguments = module.parse_arguments()
                self.assertEqual(arguments.characteristic_length, 0.73)


if __name__ == "__main__":
    unittest.main()
