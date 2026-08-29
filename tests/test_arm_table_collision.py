import unittest

import mujoco
import numpy as np

from bimanual_redundancy import paths
from bimanual_redundancy.core import (
    CooperativeManipulationKinematics,
)
from bimanual_redundancy.core import (
    ManipulabilityObjective,
    ManipulabilityOptimizer,
)
from bimanual_redundancy.simulation import DualFrankaMuJoCoScene


SPHERE_MODEL_PATH = (
    paths.ROBOT_MODELS_DIR / "franka_emika_panda" / "dual_panda_robots_only_spherefit.xml"
)


class ArmTableCollisionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene = DualFrankaMuJoCoScene()
        cls.kinematics = CooperativeManipulationKinematics(
            cls.scene.model,
            cls.scene.left_arm_dofs,
            cls.scene.right_arm_dofs,
        )
        cls.optimizer = ManipulabilityOptimizer(
            cls.kinematics,
            cls.scene.arm_qpos,
            objective=ManipulabilityObjective.FORCE,
            collision_version="version2",
            collision_sphere_model_path=SPHERE_MODEL_PATH,
            enable_collision_penalty=False,
            enable_table_collision_penalty=True,
            table_collision_weight=2.0,
            table_collision_safety_margin=1.0,
            table_collision_proximity_scale=0.01,
        )
        cls.self_optimizer = ManipulabilityOptimizer(
            cls.kinematics,
            cls.scene.arm_qpos,
            objective=ManipulabilityObjective.FORCE,
            collision_version="version2",
            collision_sphere_model_path=SPHERE_MODEL_PATH,
            enable_collision_penalty=False,
            enable_table_collision_penalty=False,
            enable_self_collision_penalty=True,
            self_collision_weight=2.0,
            self_collision_safety_margin=0.05,
            self_collision_proximity_scale=0.01,
        )

    def test_table_sphere_set_excludes_grasping_bodies(self):
        optimizer = self.optimizer
        model = self.scene.model
        left_names = {
            model.body(int(body_id)).name
            for body_id in optimizer.left_table_body_ids
        }
        right_names = {
            model.body(int(body_id)).name
            for body_id in optimizer.right_table_body_ids
        }

        self.assertEqual(
            left_names,
            {"link1_l", "link2", *(f"link{i}_l" for i in range(3, 8))},
        )
        self.assertEqual(right_names, {f"link{i}_r" for i in range(1, 8)})
        all_names = left_names | right_names
        self.assertFalse(any("finger" in name.lower() for name in all_names))
        self.assertNotIn("hand_l", all_names)
        self.assertNotIn("hand_r", all_names)
        self.assertEqual(optimizer.left_table_radii.size, 91)
        self.assertEqual(optimizer.right_table_radii.size, 101)

    def test_auto_detects_the_table_top_collision_box(self):
        optimizer = self.optimizer
        model = self.scene.model
        geom_id = optimizer.table_collision_geom_id
        self.assertGreaterEqual(geom_id, 0)
        self.assertEqual(model.geom_type[geom_id], mujoco.mjtGeom.mjGEOM_BOX)
        np.testing.assert_allclose(
            model.geom_size[geom_id],
            [0.202, 0.134, 0.005],
        )

    def test_oriented_top_plane_clearance_has_expected_sign_and_magnitude(self):
        optimizer = self.optimizer
        data = self.scene.data
        geom_id = optimizer.table_collision_geom_id
        center = data.geom_xpos[geom_id]
        rotation = data.geom_xmat[geom_id].reshape(3, 3)
        half_size = self.scene.model.geom_size[geom_id]
        radius = 0.01
        gap = 0.03
        sphere_center = center + rotation @ np.array(
            [0.0, 0.0, half_size[2] + radius + gap]
        )
        clearance = optimizer._sphere_to_table_top_plane_clearances(
            data,
            sphere_center[np.newaxis, :],
            np.array([radius]),
            geom_id,
        )
        self.assertAlmostEqual(clearance[0], gap, places=12)

        intersecting_clearance = optimizer._sphere_to_table_top_plane_clearances(
            data,
            (center + rotation @ np.array([0.0, 0.0, half_size[2]]))[
                np.newaxis, :
            ],
            np.array([radius]),
            geom_id,
        )
        self.assertAlmostEqual(intersecting_clearance[0], -radius, places=12)

        # The plane is limited to the tabletop footprint; lower-arm spheres
        # beside the table must not collide with an infinite plane.
        translated_center = sphere_center + rotation @ np.array([2.0, -3.0, 0.0])
        translated_clearance = optimizer._sphere_to_table_top_plane_clearances(
            data,
            translated_center[np.newaxis, :],
            np.array([radius]),
            geom_id,
        )
        self.assertGreater(translated_clearance[0], 1.0)

    def test_table_clearance_and_cost_are_finite(self):
        clearance = self.optimizer.minimum_arm_table_clearance(self.scene.data)
        cost = self.optimizer.arm_table_collision_cost(self.scene.data)
        self.assertTrue(np.isfinite(clearance))
        self.assertTrue(np.isfinite(cost))
        self.assertGreater(cost, 0.0)

    def test_self_collision_pairs_use_full_model_with_gripper_exclusions(self):
        optimizer = self.self_optimizer
        model = self.scene.model
        for side in ("left", "right"):
            body_ids = getattr(optimizer, f"{side}_detailed_body_ids")
            pairs = getattr(optimizer, f"{side}_self_collision_pairs")
            self.assertGreater(pairs.shape[0], 0)
            includes_gripper_to_nonadjacent_link = False
            for first, second in pairs:
                first_body = int(body_ids[first])
                second_body = int(body_ids[second])
                first_name = model.body(first_body).name
                second_name = model.body(second_body).name
                first_is_gripper = (
                    "hand" in first_name.lower()
                    or "finger" in first_name.lower()
                )
                second_is_gripper = (
                    "hand" in second_name.lower()
                    or "finger" in second_name.lower()
                )
                self.assertNotEqual(first_body, second_body)
                self.assertNotEqual(
                    int(model.body_parentid[first_body]),
                    second_body,
                )
                self.assertNotEqual(
                    int(model.body_parentid[second_body]),
                    first_body,
                )
                self.assertFalse(first_is_gripper and second_is_gripper)
                self.assertFalse(
                    first_is_gripper and second_name.endswith("link7_l")
                )
                self.assertFalse(
                    first_is_gripper and second_name.endswith("link7_r")
                )
                self.assertFalse(
                    second_is_gripper and first_name.endswith("link7_l")
                )
                self.assertFalse(
                    second_is_gripper and first_name.endswith("link7_r")
                )
                if (
                    first_is_gripper and "link5" in second_name
                ) or (
                    second_is_gripper and "link5" in first_name
                ):
                    includes_gripper_to_nonadjacent_link = True
            self.assertTrue(includes_gripper_to_nonadjacent_link)

    def test_self_collision_cost_is_in_the_null_space_objective(self):
        optimizer = self.self_optimizer
        data = self.scene.data
        clearance = optimizer.minimum_self_collision_clearance(data)
        raw_cost = optimizer.self_collision_cost(data)
        weighted_cost = optimizer.total_collision_cost(data)

        self.assertTrue(np.isfinite(clearance))
        self.assertTrue(np.isfinite(raw_cost))
        self.assertGreater(raw_cost, 0.0)
        self.assertAlmostEqual(
            weighted_cost,
            optimizer.self_collision_weight * raw_cost,
        )

        force_base = optimizer.force_manipulability(data)
        self.assertAlmostEqual(
            optimizer.value(data, ManipulabilityObjective.FORCE),
            force_base - weighted_cost,
        )
        directional_base = optimizer.directional_force_cost(data)
        self.assertAlmostEqual(
            optimizer.value(data, ManipulabilityObjective.DIRECTIONAL_FORCE),
            directional_base + weighted_cost,
        )

    def test_collision_sign_matches_objective_convention(self):
        optimizer = self.optimizer
        data = self.scene.data
        weighted_cost = optimizer.total_collision_cost(data)
        self.assertGreater(weighted_cost, 0.0)

        force_base = optimizer.force_manipulability(data)
        force_value = optimizer.value(data, ManipulabilityObjective.FORCE)
        self.assertAlmostEqual(force_value, force_base - weighted_cost)

        directional_base = optimizer.directional_force_cost(data)
        directional_value = optimizer.value(
            data,
            ManipulabilityObjective.DIRECTIONAL_FORCE,
        )
        self.assertAlmostEqual(
            directional_value,
            directional_base + weighted_cost,
        )

        optimizer.enable_table_collision_penalty = False
        try:
            self.assertAlmostEqual(
                optimizer.value(data, ManipulabilityObjective.FORCE),
                force_base,
            )
        finally:
            optimizer.enable_table_collision_penalty = True


if __name__ == "__main__":
    unittest.main()
