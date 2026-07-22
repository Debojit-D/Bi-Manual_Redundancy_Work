from unittest import mock
import unittest

import numpy as np

from MUJOCO.utils.scene_builder import DualFrankaMuJoCoScene


class _RunningViewer:
    @staticmethod
    def is_running():
        return True


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
        self.assertIs(retreat.call_args.args[2][0], left_postgrasp)
        self.assertIs(retreat.call_args.args[3][0], right_postgrasp)
        self.assertEqual(
            retreat.call_args.kwargs["action"],
            "Retreating to post-grasp",
        )

if __name__ == "__main__":
    unittest.main()
