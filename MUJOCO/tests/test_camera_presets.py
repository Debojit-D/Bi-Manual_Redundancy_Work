from types import SimpleNamespace
import unittest

import numpy as np

from MUJOCO.utils.scene_builder import DualFrankaMuJoCoScene


class CameraPresetTests(unittest.TestCase):
    def setUp(self):
        self.scene = object.__new__(DualFrankaMuJoCoScene)
        self.viewer = SimpleNamespace(
            user_scn=None,
            cam=SimpleNamespace(
                distance=0.0,
                lookat=np.zeros(3),
                azimuth=0.0,
                elevation=0.0,
            ),
        )

    def test_front_view_uses_front_camera_preset(self):
        self.scene.configure_viewer_camera(self.viewer, front_view=True)

        np.testing.assert_allclose(
            self.viewer.cam.lookat,
            self.scene.FRONT_CAMERA_LOOKAT,
        )
        self.assertEqual(
            self.viewer.cam.azimuth,
            self.scene.FRONT_CAMERA_AZIMUTH,
        )
        self.assertEqual(
            self.viewer.cam.elevation,
            self.scene.FRONT_CAMERA_ELEVATION,
        )
        self.assertEqual(self.viewer.cam.distance, self.scene.CAMERA_DISTANCE)

    def test_front_and_top_views_are_mutually_exclusive(self):
        with self.assertRaisesRegex(ValueError, "mutually exclusive"):
            self.scene.configure_viewer_camera(
                self.viewer,
                top_view=True,
                front_view=True,
            )


if __name__ == "__main__":
    unittest.main()
