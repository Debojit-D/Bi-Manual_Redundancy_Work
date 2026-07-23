from pathlib import Path
import threading
import time
import unittest
from unittest.mock import patch

from MUJOCO.scripts import comparison_main


class ComparisonMainTests(unittest.TestCase):
    def test_default_workers_and_video_settings(self):
        arguments = comparison_main.parse_arguments([])

        self.assertEqual(arguments.workers, 3)
        self.assertEqual(arguments.video_width, 1280)
        self.assertEqual(arguments.video_height, 720)
        self.assertEqual(arguments.video_fps, 50)

    def test_worker_count_must_match_available_stages(self):
        for workers in ("0", "4"):
            with self.subTest(workers=workers):
                with self.assertRaises(SystemExit):
                    comparison_main.parse_arguments(["--workers", workers])

    def test_stage_order_and_recording_flags(self):
        arguments = comparison_main.parse_arguments([])
        commands = comparison_main.build_stage_commands(
            arguments,
            Path("/tmp/equation8_batch"),
        )

        self.assertEqual(
            tuple(name for name, _ in commands),
            ("static", "pick_place", "6d_pick_place"),
        )
        for _, command in commands:
            self.assertIn("--record-data", command)
            self.assertIn("--record-video", command)
            view_index = command.index("--video-view")
            self.assertEqual(command[view_index + 1], "all")

    def test_all_four_modes_remain_configured(self):
        module_names = (
            "dual_franka_eq8_static_comparison",
            "dual_franka_eq8_pick_place_comparison",
            "dual_franka_eq8_6d_pick_place_comparison",
        )
        for module_name in module_names:
            module = __import__(
                f"MUJOCO.scripts.{module_name}",
                fromlist=["EXPERIMENTS"],
            )
            self.assertEqual(
                tuple(case[0] for case in module.EXPERIMENTS),
                ("baseline", "velocity", "force", "directional_force"),
            )

    def test_pickup_positions_and_camera_settings_are_uniform(self):
        pickup_positions, camera_distances = (
            comparison_main.validate_uniform_configuration()
        )
        self.assertEqual(len(pickup_positions), 6)
        self.assertEqual(camera_distances, (2.0, 1.7, 1.7))

    def test_expected_run_count_covers_all_three_stages(self):
        counts = comparison_main.expected_run_counts()
        self.assertEqual(
            counts,
            {"static": 24, "pick_place": 24, "6d_pick_place": 24},
        )
        self.assertEqual(sum(counts.values()), 72)

    def test_three_workers_execute_all_stages_concurrently(self):
        commands = tuple((name, (name,)) for name, _ in comparison_main.STAGES)
        barrier = threading.Barrier(3)
        state_lock = threading.Lock()
        active = 0
        maximum_active = 0

        class Progress:
            def update(self, _amount):
                return None

            def set_postfix_str(self, _text, refresh=True):
                return None

        def fake_run_stage(command, stage_name, progress, **_kwargs):
            nonlocal active, maximum_active
            del command, stage_name, progress
            with state_lock:
                active += 1
                maximum_active = max(maximum_active, active)
            barrier.wait(timeout=2)
            time.sleep(0.01)
            with state_lock:
                active -= 1
            return 0.01, 24

        with patch.object(
            comparison_main,
            "run_stage",
            side_effect=fake_run_stage,
        ):
            results = comparison_main.run_stages(
                commands,
                Progress(),
                workers=3,
            )

        self.assertEqual(maximum_active, 3)
        self.assertEqual(
            tuple(result[0] for result in results),
            ("static", "pick_place", "6d_pick_place"),
        )


if __name__ == "__main__":
    unittest.main()
