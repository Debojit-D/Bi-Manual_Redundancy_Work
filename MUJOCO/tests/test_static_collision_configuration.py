import unittest

from MUJOCO.scripts import dual_franka_eq8_optimized_6d_pick_place as spatial
from MUJOCO.scripts import dual_franka_eq8_optimized_pick_place as pick_place
from MUJOCO.scripts import dual_franka_eq8_static_optimization as static


class StaticCollisionConfigurationTests(unittest.TestCase):
    def test_static_defaults_match_optimized_pick_and_place_scripts(self):
        names = (
            "ENABLE_COLLISION_PENALTY",
            "ENABLE_TABLE_COLLISION_PENALTY",
            "ENABLE_SELF_COLLISION_PENALTY",
            "COLLISION_WEIGHT",
            "COLLISION_SAFETY_MARGIN",
            "COLLISION_PROXIMITY_SCALE",
            "TABLE_COLLISION_WEIGHT",
            "TABLE_COLLISION_SAFETY_MARGIN",
            "TABLE_COLLISION_PROXIMITY_SCALE",
            "SELF_COLLISION_WEIGHT",
            "SELF_COLLISION_SAFETY_MARGIN",
            "SELF_COLLISION_PROXIMITY_SCALE",
        )
        for name in names:
            with self.subTest(name=name):
                expected = getattr(pick_place, name)
                self.assertEqual(getattr(static, name), expected)
                self.assertEqual(getattr(spatial, name), expected)


if __name__ == "__main__":
    unittest.main()
