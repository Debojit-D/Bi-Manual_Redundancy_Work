from unittest import mock
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from MUJOCO.utils.scene_builder import DualFrankaMuJoCoScene


class _RunningViewer:
    user_scn = None

    @staticmethod
    def is_running():
        return True

    @staticmethod
    def sync():
        return None


class _UnthrottledRate:
    @staticmethod
    def sleep():
        return None


class GraspDisengagementTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.scene = DualFrankaMuJoCoScene()

    def test_home_configuration_is_captured_from_home_keyframe(self):
        keyframe_qpos = self.scene.model.key("home1").qpos
        np.testing.assert_allclose(
            self.scene.home_arm_configuration,
            keyframe_qpos[self.scene.arm_qpos],
        )

    def test_legacy_positive_roll_uses_shorter_alternate_grasp(self):
        scene = DualFrankaMuJoCoScene(
            use_alternate_grasp_orientation=True
        )
        scene.set_table_reference_pose(
            np.array([0.60, 0.00, 0.28]),
            Rotation.from_euler(
                "xyz", [np.pi / 2.0, 0.0, np.pi / 2.0]
            ).as_matrix(),
        )
        target_quaternions = scene._initialize_mocap_targets()

        for hand_name, contact_name, target_quaternion in zip(
            ("attachment_site_left", "attachment_site_right"),
            ("site_left", "site_right"),
            target_quaternions,
        ):
            hand_rotation = scene.data.site_xmat[
                scene.model.site(hand_name).id
            ].reshape(3, 3)
            contact_rotation = scene.data.site_xmat[
                scene.model.site(contact_name).id
            ].reshape(3, 3)
            target_rotation = Rotation.from_quat(
                np.roll(target_quaternion, -1)
            ).as_matrix()
            expected_rotation = contact_rotation @ Rotation.from_rotvec(
                [0.0, 0.0, np.pi]
            ).as_matrix()

            np.testing.assert_allclose(
                target_rotation, expected_rotation, atol=1e-12
            )
            np.testing.assert_allclose(
                target_rotation[:, 2], contact_rotation[:, 2], atol=1e-12
            )
            nominal_angle = Rotation.from_matrix(
                contact_rotation @ hand_rotation.T
            ).magnitude()
            alternate_angle = Rotation.from_matrix(
                target_rotation @ hand_rotation.T
            ).magnitude()
            self.assertLess(alternate_angle, nominal_angle)

    def test_reoriented_pose_4_nominal_pregrasp_reaches_both_waypoints(self):
        scene = DualFrankaMuJoCoScene(
            use_alternate_grasp_orientation=False
        )
        scene.set_table_reference_pose(
            np.array([0.60, 0.00, 0.28]),
            Rotation.from_euler(
                "xyz", [-np.pi / 2.0, 0.0, np.pi / 2.0]
            ).as_matrix(),
        )

        completed = scene.run_grasp_approach(
            _RunningViewer(),
            _UnthrottledRate(),
        )

        self.assertTrue(completed)

    def test_disengagement_opens_then_retreats_then_returns_home(self):
        events = []
        left_postgrasp = (np.ones(3), np.array([1.0, 0.0, 0.0, 0.0]))
        right_postgrasp = (-np.ones(3), np.array([1.0, 0.0, 0.0, 0.0]))
        approach_waypoints = (
            [left_postgrasp, mock.sentinel.left_grasp],
            [right_postgrasp, mock.sentinel.right_grasp],
        )

        with (
            mock.patch.object(
                self.scene,
                "_approach_waypoints",
                return_value=approach_waypoints,
            ),
            mock.patch.object(
                self.scene,
                "open_grippers",
                side_effect=lambda *_: events.append("open") or True,
            ),
            mock.patch.object(
                self.scene,
                "_run_gripper_waypoints",
                side_effect=lambda *_, **__: events.append("retreat") or True,
            ) as retreat,
            mock.patch.object(
                self.scene,
                "return_arms_home",
                side_effect=lambda *_: events.append("home") or True,
            ),
        ):
            completed = self.scene.run_grasp_disengagement(
                _RunningViewer(),
                mock.sentinel.rate,
            )

        self.assertTrue(completed)
        self.assertEqual(events, ["open", "retreat", "home"])
        resolved_left = retreat.call_args.args[2][0]
        resolved_right = retreat.call_args.args[3][0]
        object_position = self.scene.data.site_xpos[
            self.scene.model.site("site_top_middle").id
        ]
        left_contact = self.scene.data.site_xpos[
            self.scene.model.site("site_left").id
        ]
        right_contact = self.scene.data.site_xpos[
            self.scene.model.site("site_right").id
        ]
        left_outward = left_contact - object_position
        left_outward /= np.linalg.norm(left_outward)
        right_outward = right_contact - object_position
        right_outward /= np.linalg.norm(right_outward)
        np.testing.assert_allclose(
            resolved_left[0],
            left_contact
            + 0.14 * left_outward
            + np.array([0.0, 0.0, 0.09]),
        )
        np.testing.assert_allclose(
            resolved_right[0],
            right_contact
            + 0.14 * right_outward
            + np.array([0.0, 0.0, 0.09]),
        )
        np.testing.assert_array_equal(resolved_left[1], left_postgrasp[1])
        np.testing.assert_array_equal(resolved_right[1], right_postgrasp[1])
        self.assertEqual(
            retreat.call_args.kwargs["action"],
            "Retreating to post-grasp",
        )

if __name__ == "__main__":
    unittest.main()
