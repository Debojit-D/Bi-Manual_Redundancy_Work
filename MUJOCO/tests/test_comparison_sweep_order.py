import unittest
from types import SimpleNamespace

from MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison import (
    EXPERIMENTS,
    experiments_for_mode,
    resolve_trajectory_cases,
    video_views_for_choice,
)
from MUJOCO.scripts.dual_franka_eq8_optimized_6d_pick_place import (
    resolve_cli_trajectory,
)
from MUJOCO.scripts.table_spawn_comparison_positions import (
    SIX_D_TRAJECTORY_CASES,
    TABLE_SPAWN_CASES,
    ordered_comparison_cases,
    six_d_trajectory_case_for_number,
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

    def test_single_comparison_mode_selection(self):
        selected = experiments_for_mode("velocity")
        self.assertEqual(len(selected), 1)
        self.assertEqual(selected[0][0], "velocity")
        self.assertEqual(experiments_for_mode("all"), EXPERIMENTS)

    def test_single_perspective_video_view_selection(self):
        self.assertEqual(
            video_views_for_choice("perspective"),
            ("perspective",),
        )
        self.assertEqual(
            video_views_for_choice("both"),
            ("perspective", "top_view"),
        )
        self.assertEqual(
            video_views_for_choice("all"),
            ("perspective", "top_view", "front_view"),
        )
        self.assertEqual(
            video_views_for_choice("front"),
            ("front_view",),
        )

    def test_6d_trajectory_table_aligns_with_pickup_cases(self):
        self.assertEqual(
            tuple(
                (name.replace("pose_", "position_"), pickup)
                for name, pickup, *_ in SIX_D_TRAJECTORY_CASES
            ),
            TABLE_SPAWN_CASES,
        )
        pose_1 = six_d_trajectory_case_for_number(1)
        self.assertEqual(pose_1[0], "pose_1")
        self.assertEqual(pose_1[3], (0.40, 0.00, 0.52))
        self.assertEqual(pose_1[5], (0.25, -0.45, 0.269))
        self.assertAlmostEqual(pose_1[2][2], 1.5707963267948966)
        self.assertAlmostEqual(pose_1[4][2], 1.1707963267948966)
        self.assertAlmostEqual(pose_1[6][2], 0.7707963267948965)

        pose_2 = six_d_trajectory_case_for_number(2)
        self.assertEqual(pose_2[3], (0.40, 0.00, 0.52))
        self.assertAlmostEqual(pose_2[4][2], 1.1707963267948966)
        self.assertEqual(pose_2[5], (0.25, -0.45, 0.269))
        self.assertAlmostEqual(pose_2[6][2], 0.7707963267948965)

        pose_5 = six_d_trajectory_case_for_number(5)
        self.assertEqual(pose_5[3], (0.40, 0.00, 0.52))
        self.assertAlmostEqual(pose_5[4][2], 1.9707963267948965)
        self.assertEqual(pose_5[5], (0.25, 0.45, 0.269))
        self.assertAlmostEqual(pose_5[6][2], 2.3707963267948964)

        pose_6 = six_d_trajectory_case_for_number(6)
        self.assertEqual(pose_6[3], (0.40, 0.00, 0.52))
        self.assertAlmostEqual(pose_6[4][2], 1.9707963267948965)
        self.assertEqual(pose_6[5], (0.25, 0.45, 0.269))
        self.assertAlmostEqual(pose_6[6][2], 2.3707963267948964)

        for index in (3, 4):
            case = six_d_trajectory_case_for_number(index)
            self.assertTrue(all(value is not None for value in case[1:]))
        self.assertAlmostEqual(
            six_d_trajectory_case_for_number(4)[2][0],
            -1.5707963267948966,
        )

    def test_6d_default_sweep_includes_all_complete_trajectory_entries(self):
        resolved, incomplete = resolve_trajectory_cases(
            SIX_D_TRAJECTORY_CASES
        )
        self.assertEqual(
            tuple(case[0] for case in resolved),
            tuple(f"pose_{index}" for index in range(1, 7)),
        )
        self.assertEqual(incomplete, ())

    def test_6d_cli_overrides_apply_to_all_pose_entries(self):
        intermediate = (1.0, 2.0, 3.0)
        intermediate_euler = (0.1, 0.2, 0.3)
        final = (4.0, 5.0, 6.0)
        final_euler = (0.4, 0.5, 0.6)
        resolved, incomplete = resolve_trajectory_cases(
            SIX_D_TRAJECTORY_CASES,
            intermediate_position_override=intermediate,
            intermediate_euler_xyz_override=intermediate_euler,
            goal_position_override=final,
            goal_euler_xyz_override=final_euler,
        )
        self.assertEqual(len(resolved), 6)
        self.assertEqual(incomplete, ())
        for _, values in resolved:
            resolved_intermediate = values[2]
            resolved_intermediate_euler = values[3]
            resolved_final = values[4]
            resolved_final_euler = values[5]
            self.assertEqual(resolved_intermediate, intermediate)
            self.assertEqual(resolved_intermediate_euler, intermediate_euler)
            self.assertEqual(resolved_final, final)
            self.assertEqual(resolved_final_euler, final_euler)

    def test_standalone_pose_1_resolves_all_orientations(self):
        arguments = SimpleNamespace(
            pose=1,
            start_position=None,
            start_euler_xyz=None,
            intermediate_position=None,
            intermediate_euler_xyz=None,
            goal_position=None,
            goal_euler_xyz=None,
        )
        self.assertEqual(
            resolve_cli_trajectory(arguments),
            SIX_D_TRAJECTORY_CASES[0][1:],
        )

    def test_standalone_pose_4_resolves_complete_path(self):
        arguments = SimpleNamespace(
            pose=4,
            start_position=None,
            start_euler_xyz=None,
            intermediate_position=None,
            intermediate_euler_xyz=None,
            goal_position=None,
            goal_euler_xyz=None,
        )
        self.assertEqual(
            resolve_cli_trajectory(arguments),
            SIX_D_TRAJECTORY_CASES[3][1:],
        )


if __name__ == "__main__":
    unittest.main()
