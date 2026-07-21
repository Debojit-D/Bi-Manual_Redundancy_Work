import unittest

from MUJOCO.scripts.table_spawn_comparison_positions import (
    ordered_comparison_cases,
)


class ComparisonSweepOrderTests(unittest.TestCase):
    def setUp(self):
        self.positions = (
            ("position_1", (1.0, 0.0, 0.0)),
            ("position_2", (2.0, 0.0, 0.0)),
        )
        self.experiments = (
            ("baseline", "force", False),
            ("velocity", "velocity", True),
        )

    def test_option_1_runs_all_modes_at_each_position(self):
        cases = ordered_comparison_cases(
            self.positions,
            self.experiments,
            sweep_option=1,
        )
        self.assertEqual(
            [(case[0], case[2]) for case in cases],
            [
                ("position_1", "baseline"),
                ("position_1", "velocity"),
                ("position_2", "baseline"),
                ("position_2", "velocity"),
            ],
        )

    def test_option_2_runs_all_positions_for_each_mode(self):
        cases = ordered_comparison_cases(
            self.positions,
            self.experiments,
            sweep_option=2,
        )
        self.assertEqual(
            [(case[0], case[2]) for case in cases],
            [
                ("position_1", "baseline"),
                ("position_2", "baseline"),
                ("position_1", "velocity"),
                ("position_2", "velocity"),
            ],
        )

    def test_unknown_option_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "sweep_option"):
            ordered_comparison_cases(
                self.positions,
                self.experiments,
                sweep_option=3,
            )


if __name__ == "__main__":
    unittest.main()
