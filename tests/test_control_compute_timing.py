"""Focused tests for Equation (8) computation-only latency measurement."""

from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

from bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place import (
    optimized_control_step,
)
from bimanual_redundancy.simulation.control_timing import timed_equation_8_update
from bimanual_redundancy.core import ManipulabilityObjective


class _FakeScene:
    def __init__(self, events=None):
        self.events = [] if events is None else events
        self.data = SimpleNamespace()
        self.arm_dofs = np.arange(14)
        self.gripper_closed = 0.0

    def command(self, *_args):
        self.events.append("command")

    def step(self, _viewer):
        self.events.append("step")


class _FakeOptimizer:
    objective = ManipulabilityObjective.VELOCITY

    def __init__(self, events):
        self.events = events
        self.optimization_calls = 0

    def optimization_velocity(self, _data):
        self.events.append("optimizer")
        self.optimization_calls += 1
        return SimpleNamespace(
            objective=self.objective,
            value=2.0,
            gradient=np.ones(14),
            phi_dot_opt=np.ones(14),
        )

    @staticmethod
    def value(_data):
        return 2.0


class _FakeEquation8:
    def __init__(self, events):
        self.events = events

    def update(self, _data, phi, *_args):
        self.events.append("controller_update")
        return np.asarray(phi) + 1.0, SimpleNamespace()


class _FakeRate:
    def __init__(self, events):
        self.events = events

    def sleep(self):
        self.events.append("sleep")


class _FakeRecorder:
    def __init__(self, events):
        self.events = events
        self.keyword_arguments = None

    def record(self, *_args, **kwargs):
        self.events.append("record")
        self.keyword_arguments = kwargs


class ControlComputeTimingTests(unittest.TestCase):
    def test_enabled_timing_measures_optimizer_and_update_separately(self):
        events = []
        scene = _FakeScene(events)
        optimizer = _FakeOptimizer(events)
        equation_8 = _FakeEquation8(events)
        with patch(
            "bimanual_redundancy.simulation.control_timing.time.perf_counter_ns",
            side_effect=(0, 2_000_000, 2_500_000, 5_500_000),
        ):
            _, _, _, timing = timed_equation_8_update(
                scene,
                equation_8,
                optimizer,
                np.zeros(14),
                np.zeros(3),
                np.eye(3),
                np.zeros(6),
            )

        self.assertEqual(events, ["optimizer", "controller_update"])
        self.assertEqual(timing.optimizer_time_ms, 2.0)
        self.assertEqual(timing.controller_update_time_ms, 3.0)
        self.assertEqual(timing.control_compute_time_ms, 5.5)
        self.assertGreaterEqual(
            timing.control_compute_time_ms,
            timing.optimizer_time_ms + timing.controller_update_time_ms,
        )

    def test_baseline_records_zero_optimizer_time(self):
        events = []
        scene = _FakeScene(events)
        optimizer = _FakeOptimizer(events)
        equation_8 = _FakeEquation8(events)
        with patch(
            "bimanual_redundancy.simulation.control_timing.time.perf_counter_ns",
            side_effect=(0, 1_000_000, 4_000_000),
        ):
            _, optimization, _, timing = timed_equation_8_update(
                scene,
                equation_8,
                optimizer,
                np.zeros(14),
                np.zeros(3),
                np.eye(3),
                np.zeros(6),
                enable_redundancy_optimization=False,
            )

        self.assertEqual(optimizer.optimization_calls, 0)
        self.assertEqual(events, ["controller_update"])
        self.assertEqual(timing.optimizer_time_ms, 0.0)
        self.assertEqual(timing.controller_update_time_ms, 3.0)
        self.assertEqual(timing.control_compute_time_ms, 4.0)
        np.testing.assert_array_equal(optimization.phi_dot_opt, np.zeros(14))

    def test_step_and_csv_work_occur_after_compute_timing(self):
        events = []
        scene = _FakeScene(events)
        optimizer = _FakeOptimizer(events)
        equation_8 = _FakeEquation8(events)
        recorder = _FakeRecorder(events)
        rate = _FakeRate(events)
        desired_twist = np.arange(6, dtype=float)
        with patch(
            "bimanual_redundancy.simulation.control_timing.time.perf_counter_ns",
            side_effect=(0, 1_000_000, 1_000_000, 3_000_000),
        ):
            optimized_control_step(
                scene,
                equation_8,
                optimizer,
                np.zeros(14),
                np.ones(3),
                np.eye(3),
                desired_twist,
                SimpleNamespace(),
                rate,
                trajectory_phase="start_to_intermediate",
                trajectory_time=0.5,
                recorder=recorder,
            )

        self.assertEqual(
            events,
            [
                "optimizer",
                "controller_update",
                "command",
                "step",
                "record",
                "sleep",
            ],
        )
        self.assertEqual(recorder.keyword_arguments["optimizer_time_ms"], 1.0)
        self.assertEqual(
            recorder.keyword_arguments["controller_update_time_ms"], 2.0
        )
        self.assertEqual(
            recorder.keyword_arguments["control_compute_time_ms"], 3.0
        )
        np.testing.assert_array_equal(
            recorder.keyword_arguments["desired_twist"], desired_twist
        )
        self.assertEqual(
            recorder.keyword_arguments["trajectory_phase"],
            "start_to_intermediate",
        )
        self.assertEqual(recorder.keyword_arguments["trajectory_time"], 0.5)


if __name__ == "__main__":
    unittest.main()
