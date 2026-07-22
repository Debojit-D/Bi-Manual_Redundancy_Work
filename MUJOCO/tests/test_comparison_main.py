from pathlib import Path
import unittest

from MUJOCO.scripts import comparison_main


class ComparisonMainTests(unittest.TestCase):
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


if __name__ == "__main__":
    unittest.main()
