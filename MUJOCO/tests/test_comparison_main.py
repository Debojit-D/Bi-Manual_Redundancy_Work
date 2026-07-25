from contextlib import redirect_stderr, redirect_stdout
import io
from pathlib import Path
import subprocess
import sys
import threading
import time
import unittest
from unittest.mock import patch

from MUJOCO.scripts import comparison_main
from MUJOCO.scripts import add_directional_force_indirect_to_batch as addon


class ComparisonMainTests(unittest.TestCase):
    def test_indirect_addon_builds_only_the_new_mode(self):
        arguments = addon.parse_arguments(
            [
                "--dry-run",
                "--video-encoder",
                "x264",
                "--workers",
                "1",
            ]
        )
        commands = addon.build_stage_commands(arguments, Path("/tmp/batch"))

        self.assertEqual(tuple(name for name, _ in commands), tuple(
            name for name, _ in comparison_main.STAGES
        ))
        for _stage_name, command in commands:
            mode_index = command.index("--mode")
            self.assertEqual(
                command[mode_index + 1],
                "directional_force_indirect",
            )
            self.assertIn("--record-data", command)
            self.assertIn("--record-video", command)

    def test_indirect_addon_expected_run_count(self):
        self.assertEqual(
            addon.expected_run_counts(),
            {"static": 6, "pick_place": 6, "6d_pick_place": 6},
        )

    def test_default_workers_and_video_settings(self):
        arguments = comparison_main.parse_arguments([])

        self.assertEqual(arguments.workers, 3)
        self.assertEqual(arguments.video_width, 1280)
        self.assertEqual(arguments.video_height, 720)
        self.assertEqual(arguments.video_fps, 50)
        self.assertEqual(arguments.video_encoder, "nvenc")
        self.assertEqual(arguments.nvenc_session_limit, 8)
        self.assertFalse(arguments.verbose)

    def test_verbose_logging_is_opt_in(self):
        self.assertTrue(
            comparison_main.parse_arguments(["--verbose"]).verbose
        )

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
        expected_nvenc_views = {
            "static": "3",
            "pick_place": "3",
            "6d_pick_place": "2",
        }
        for stage_name, command in commands:
            self.assertIn("--record-data", command)
            self.assertIn("--record-video", command)
            view_index = command.index("--video-view")
            self.assertEqual(command[view_index + 1], "all")
            encoder_index = command.index("--video-encoder")
            self.assertEqual(command[encoder_index + 1], "nvenc")
            limit_index = command.index("--nvenc-max-views")
            self.assertEqual(
                command[limit_index + 1],
                expected_nvenc_views[stage_name],
            )

    def test_x264_fallback_is_forwarded_to_every_stage(self):
        arguments = comparison_main.parse_arguments(
            ["--video-encoder", "x264"],
        )
        commands = comparison_main.build_stage_commands(
            arguments,
            Path("/tmp/equation8_batch"),
        )

        for _, command in commands:
            encoder_index = command.index("--video-encoder")
            self.assertEqual(command[encoder_index + 1], "x264")
            self.assertNotIn("--nvenc-max-views", command)

    def test_sequential_stages_can_use_nvenc_for_all_views(self):
        arguments = comparison_main.parse_arguments(["--workers", "1"])

        self.assertEqual(
            comparison_main.stage_nvenc_view_limits(arguments),
            {"static": 3, "pick_place": 3, "6d_pick_place": 3},
        )

    def test_comparison_modes_remain_configured(self):
        expected_modes = {
            "dual_franka_eq8_static_comparison": (
                "baseline",
                "velocity",
                "force",
                "directional_force",
                "directional_force_indirect",
            ),
            "dual_franka_eq8_pick_place_comparison": (
                "baseline",
                "velocity",
                "force",
                "directional_force",
                "directional_force_indirect",
            ),
            "dual_franka_eq8_6d_pick_place_comparison": (
                "baseline",
                "velocity",
                "force",
                "directional_force",
                "directional_force_indirect",
            ),
        }
        for module_name, expected in expected_modes.items():
            module = __import__(
                f"MUJOCO.scripts.{module_name}",
                fromlist=["EXPERIMENTS"],
            )
            self.assertEqual(
                tuple(case[0] for case in module.EXPERIMENTS),
                expected,
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
            {"static": 30, "pick_place": 30, "6d_pick_place": 30},
        )
        self.assertEqual(sum(counts.values()), 90)

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

    def test_quiet_stage_suppresses_successful_child_output(self):
        class Progress:
            def update(self, _amount):
                return None

            def set_postfix_str(self, _text, refresh=True):
                return None

        output = io.StringIO()
        command = (
            sys.executable,
            "-u",
            "-c",
            "print('Run 1/1:'); print('child noise')",
        )
        with redirect_stdout(output):
            comparison_main.run_stage(
                command,
                "test",
                Progress(),
                verbose=False,
            )

        self.assertEqual(output.getvalue(), "")

    def test_quiet_stage_prints_recent_output_on_failure(self):
        class Progress:
            def update(self, _amount):
                return None

            def set_postfix_str(self, _text, refresh=True):
                return None

        error_output = io.StringIO()
        command = (
            sys.executable,
            "-u",
            "-c",
            "print('useful failure detail'); raise SystemExit(7)",
        )
        with redirect_stderr(error_output):
            with self.assertRaises(subprocess.CalledProcessError):
                comparison_main.run_stage(
                    command,
                    "test",
                    Progress(),
                    verbose=False,
                )

        self.assertIn("useful failure detail", error_output.getvalue())

    def test_quiet_stage_forwards_re_record_warning_and_continues(self):
        class Progress:
            def update(self, _amount):
                return None

            def set_postfix_str(self, _text, refresh=True):
                return None

        output = io.StringIO()
        command = (
            sys.executable,
            "-u",
            "-c",
            (
                "print('Run 1/2:'); "
                "print('\\033[1;91mCOMPARISON_RUN_FAILED: pose_4 / force"
                " -- Please re-record this run.\\033[0m'); "
                "print('Run 2/2:')"
            ),
        )
        with redirect_stdout(output):
            _, observed_runs = comparison_main.run_stage(
                command,
                "6d_pick_place",
                Progress(),
                verbose=False,
            )

        self.assertEqual(observed_runs, 2)
        self.assertIn("COMPARISON_RUN_FAILED:", output.getvalue())
        self.assertIn("Please re-record this run", output.getvalue())


if __name__ == "__main__":
    unittest.main()
