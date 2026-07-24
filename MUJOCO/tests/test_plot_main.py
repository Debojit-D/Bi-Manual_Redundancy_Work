import csv
import tempfile
from pathlib import Path
import unittest

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

from MUJOCO.plotting_scripts.equation8_plot_style import Equation8PlotStyle
from MUJOCO.scripts import plot_main


class PlotMainTests(unittest.TestCase):
    def setUp(self):
        temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(temporary_directory.cleanup)
        self.batch_dir = Path(temporary_directory.name) / "comparison_main_test"
        self.run_dir = (
            self.batch_dir
            / "data"
            / "6d_pick_place"
            / "6d_pick_place_comparison_test"
        )
        columns = plot_main.required_columns()
        for case_number in range(1, 7):
            case_dir = self.run_dir / f"pose_{case_number}"
            case_dir.mkdir(parents=True)
            for mode_index, mode in enumerate(plot_main.MODES):
                with (case_dir / f"{mode}.csv").open(
                    "w",
                    newline="",
                    encoding="utf-8",
                ) as stream:
                    writer = csv.DictWriter(stream, fieldnames=columns)
                    writer.writeheader()
                    for sample in range(3):
                        row = {
                            column: (
                                sample
                                if column == "time"
                                else case_number + mode_index + sample / 10
                            )
                            for column in columns
                        }
                        writer.writerow(row)

    def test_discovers_timestamped_stage_directory(self):
        discovered = plot_main.discover_stage_run_directory(
            self.batch_dir,
            "6d_pick_place",
        )

        self.assertEqual(discovered, self.run_dir)

    def test_loads_six_cases_with_all_modes(self):
        runs = plot_main.load_stage_runs(
            self.batch_dir,
            "6d_pick_place",
        )

        self.assertEqual(tuple(runs), tuple(f"pose_{i}" for i in range(1, 7)))
        for mode_runs in runs.values():
            self.assertEqual(set(mode_runs), set(plot_main.MODES))

    def test_grid_has_six_panels_and_baseline_behind_optimized_line(self):
        runs = plot_main.load_stage_runs(
            self.batch_dir,
            "6d_pick_place",
        )
        style = Equation8PlotStyle(dpi=72)
        style.apply()
        figure = plot_main.plot_velocity_optimization(
            runs,
            stage="6d_pick_place",
            metric_scale="raw",
            style=style,
        )
        self.addCleanup(plt.close, figure)

        self.assertEqual(len(figure.axes), 6)
        for axis in figure.axes:
            self.assertEqual(len(axis.lines), 2)
            self.assertLess(
                axis.lines[0].get_zorder(),
                axis.lines[1].get_zorder(),
            )

    def test_actuator_effort_grid_has_six_panels_and_all_modes(self):
        runs = plot_main.load_stage_runs(
            self.batch_dir,
            "6d_pick_place",
        )
        style = Equation8PlotStyle(dpi=72)
        style.apply()
        figure = plot_main.plot_actuator_effort(
            runs,
            stage="6d_pick_place",
            style=style,
        )
        self.addCleanup(plt.close, figure)

        self.assertEqual(len(figure.axes), 6)
        self.assertTrue(all(len(axis.lines) == 4 for axis in figure.axes))
        self.assertFalse(
            figure.axes[0].get_shared_x_axes().joined(
                figure.axes[0],
                figure.axes[1],
            )
        )
        self.assertFalse(
            figure.axes[0].get_shared_y_axes().joined(
                figure.axes[0],
                figure.axes[1],
            )
        )

    def test_static_effort_baseline_extends_to_longest_optimization(self):
        runs = plot_main.load_stage_runs(
            self.batch_dir,
            "6d_pick_place",
        )
        runs["pose_1"]["baseline"] = (
            runs["pose_1"]["baseline"].iloc[:2].copy()
        )
        style = Equation8PlotStyle(dpi=72)
        style.apply()
        figure = plot_main.plot_actuator_effort(
            runs,
            stage="static",
            style=style,
        )
        self.addCleanup(plt.close, figure)

        baseline_time = figure.axes[0].lines[0].get_xdata()
        self.assertEqual(baseline_time[-1], 2.0)

    def test_object_tracking_grid_has_command_and_all_measured_modes(self):
        runs = plot_main.load_stage_runs(
            self.batch_dir,
            "6d_pick_place",
        )
        style = Equation8PlotStyle(dpi=72)
        style.apply()
        figure = plot_main.plot_object_position_tracking(
            runs,
            component="x",
            ylabel="World x position (m)",
            style=style,
        )
        self.addCleanup(plt.close, figure)

        self.assertEqual(len(figure.axes), 6)
        self.assertTrue(all(len(axis.lines) == 5 for axis in figure.axes))

    def test_main_writes_optimization_effort_and_tracking_figures(self):
        output_dir = self.batch_dir / "custom_plots"
        written = plot_main.main(
            [
                str(self.batch_dir),
                "--stage",
                "6d_pick_place",
                "--format",
                "png",
                "--dpi",
                "72",
                "--output-dir",
                str(output_dir),
            ]
        )

        self.assertEqual(len(written), 7)
        self.assertTrue(all(path.is_file() for path in written))

    def test_all_stages_are_plotted_by_default(self):
        arguments = plot_main.parse_arguments([])

        self.assertEqual(arguments.stage, "all")


if __name__ == "__main__":
    unittest.main()
