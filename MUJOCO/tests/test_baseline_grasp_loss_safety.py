"""Tests for object-based grasp-loss abort and home-return behavior."""

from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from MUJOCO.scripts import dual_franka_eq8_optimized_6d_pick_place as spatial
from MUJOCO.scripts import dual_franka_eq8_optimized_pick_place as simple
from MUJOCO.utils.grasp_safety import (
    OBJECT_LOSS_ORIENTATION_THRESHOLD,
    OBJECT_LOSS_POSITION_THRESHOLD,
    assess_grasp_loss,
    finish_after_grasped_motion,
    object_grasp_loss_status,
)


class _AlwaysRunningViewer:
    @staticmethod
    def is_running():
        return True


class _CompletionScene:
    def __init__(self):
        self.home_calls = 0
        self.disengagement_calls = 0

    def return_arms_home(self, _viewer, _rate):
        self.home_calls += 1
        return True

    def run_grasp_disengagement(self, _viewer, _rate):
        self.disengagement_calls += 1
        return True


class _Trajectory:
    total_duration = 1.0
    times = np.array([0.0, 0.5, 1.0])

    @staticmethod
    def sample(_time):
        return np.zeros(3), np.eye(3), np.zeros(6)


class BaselineGraspLossSafetyTests(unittest.TestCase):
    def test_position_or_orientation_drift_triggers_loss(self):
        safe = np.zeros(12)
        safe[0] = 0.5 * OBJECT_LOSS_POSITION_THRESHOLD
        safe[3] = 0.5 * OBJECT_LOSS_ORIENTATION_THRESHOLD
        self.assertFalse(assess_grasp_loss(safe).lost)

        position_loss = np.zeros(12)
        position_loss[6] = 1.01 * OBJECT_LOSS_POSITION_THRESHOLD
        self.assertTrue(assess_grasp_loss(position_loss).lost)

        orientation_loss = np.zeros(12)
        orientation_loss[10] = (
            1.01 * OBJECT_LOSS_ORIENTATION_THRESHOLD
        )
        self.assertTrue(assess_grasp_loss(orientation_loss).lost)

    def test_only_hand_drift_relative_to_live_object_triggers_safety(self):
        safe_grasp_error = np.zeros(12)
        self.assertIsNone(object_grasp_loss_status(safe_grasp_error))

        lost_grasp_error = np.zeros(12)
        lost_grasp_error[0] = 2.0 * OBJECT_LOSS_POSITION_THRESHOLD
        self.assertIsNotNone(
            object_grasp_loss_status(lost_grasp_error)
        )

    def test_loss_goes_directly_home_without_postgrasp_retreat(self):
        viewer = _AlwaysRunningViewer()
        scene = _CompletionScene()
        self.assertTrue(
            finish_after_grasped_motion(
                scene,
                viewer,
                object(),
                grasp_lost=True,
            )
        )
        self.assertEqual(scene.home_calls, 1)
        self.assertEqual(scene.disengagement_calls, 0)

        scene = _CompletionScene()
        self.assertTrue(
            finish_after_grasped_motion(
                scene,
                viewer,
                object(),
                grasp_lost=False,
            )
        )
        self.assertEqual(scene.home_calls, 0)
        self.assertEqual(scene.disengagement_calls, 1)

    def test_simple_default_optimized_run_aborts_on_object_loss(self):
        loss_error = np.zeros(12)
        loss_error[0] = 2.0 * OBJECT_LOSS_POSITION_THRESHOLD
        diagnostics = SimpleNamespace(
            pose_error=np.zeros(6),
            grasp_pose_error=loss_error,
        )
        control_step = Mock(return_value=(
            np.zeros(14),
            SimpleNamespace(value=1.0),
            diagnostics,
        ))
        scene = SimpleNamespace(
            data=object(),
            arm_configuration=lambda: np.zeros(14),
        )
        kinematics = SimpleNamespace(
            object_pose=lambda _data: (np.zeros(3), np.eye(3))
        )
        optimizer = SimpleNamespace(
            objective=SimpleNamespace(value="force"),
            value=lambda _data: 1.0,
        )

        with patch.object(
            simple,
            "optimized_equation_8_step",
            control_step,
        ), patch.object(simple, "print_grasp_loss"):
            result = simple.run_optimized_lift(
                scene,
                kinematics,
                object(),
                optimizer,
                _AlwaysRunningViewer(),
                object(),
                hold_duration=0.0,
                enable_redundancy_optimization=True,
            )

        self.assertTrue(result["grasp_lost"])
        self.assertEqual(control_step.call_count, 1)

    def test_6d_default_optimized_run_aborts_on_object_loss(self):
        loss_error = np.zeros(12)
        loss_error[0] = 2.0 * OBJECT_LOSS_POSITION_THRESHOLD
        diagnostics = SimpleNamespace(
            pose_error=np.zeros(6),
            grasp_pose_error=loss_error,
        )
        control_step = Mock(return_value=(
            np.zeros(14),
            SimpleNamespace(value=1.0),
            diagnostics,
        ))

        with patch.object(
            spatial,
            "optimized_control_step",
            control_step,
        ), patch.object(spatial, "print_grasp_loss"):
            _, grasp_lost = spatial.run_trajectory(
                SimpleNamespace(),
                object(),
                object(),
                np.zeros(14),
                _Trajectory(),
                _AlwaysRunningViewer(),
                object(),
                enable_redundancy_optimization=True,
            )

        self.assertTrue(grasp_lost)
        self.assertEqual(control_step.call_count, 1)


if __name__ == "__main__":
    unittest.main()
