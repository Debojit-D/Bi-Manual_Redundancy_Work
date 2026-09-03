import unittest

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

from bimanual_redundancy.plotting.equation8_plot_style import Equation8PlotStyle
from bimanual_redundancy.plotting import plot_directional_force_cross_evaluation as cross_plot


class DirectionalForceCrossEvaluationTests(unittest.TestCase):
    def test_strict_fonts_flag_is_opt_in(self):
        arguments = cross_plot.parse_arguments([])

        self.assertFalse(arguments.strict_fonts)

    def test_metrics_are_recomputed_from_velocity_map(self):
        velocity_map = np.zeros((6, 14))
        velocity_map[:, :6] = np.diag([2.0, 1.5, 1.0, 0.8, 0.7, 0.6])

        metrics = cross_plot.directional_metrics_from_velocity_map(
            velocity_map,
            0.4,
            pinv_rcond=1e-6,
        )

        velocity = velocity_map @ velocity_map.T
        force = np.linalg.pinv(velocity, rcond=1e-6)
        desired = np.diag([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
        self.assertAlmostEqual(
            metrics["recomputed_direct_raw"],
            cross_plot.normalized_frobenius_distance(force, desired),
        )
        self.assertAlmostEqual(
            metrics["recomputed_indirect_raw"],
            cross_plot.normalized_frobenius_distance(velocity, desired),
        )

    def test_four_rows_use_native_and_crossed_metric_routing(self):
        evaluated = {}
        for case_number in range(1, 7):
            modes = {}
            for mode_index, mode in enumerate(cross_plot.MODES, start=1):
                modes[mode] = pd.DataFrame(
                    {
                        "time": [0.0, 1.0],
                        "recomputed_direct_raw": [10.0 + mode_index] * 2,
                        "recomputed_direct_scaled": [20.0 + mode_index] * 2,
                        "recomputed_indirect_raw": [30.0 + mode_index] * 2,
                        "recomputed_indirect_scaled": [40.0 + mode_index] * 2,
                    }
                )
            evaluated[f"pose_{case_number}"] = modes

        style = Equation8PlotStyle(dpi=72)
        style.apply()
        figure = cross_plot.plot_cross_evaluation(
            evaluated,
            stage="6d_pick_place",
            metric_scale="raw",
            style=style,
        )
        self.addCleanup(plt.close, figure)

        expected = (
            (11.0, 12.0),
            (31.0, 33.0),
            (31.0, 32.0, 33.0),
            (11.0, 12.0, 13.0),
        )
        observed = tuple(
            tuple(
                line.get_ydata()[0] * np.sqrt(2.0)
                for line in figure.axes[row * 6].lines
            )
            for row in range(4)
        )
        for observed_row, expected_row in zip(observed, expected):
            np.testing.assert_allclose(observed_row, expected_row)

    def test_all_stage_summary_uses_row_four_metric_and_three_traces(self):
        evaluated_stages = {}
        for stage_index, stage in enumerate(cross_plot.STAGES):
            evaluated = {}
            for case_number in range(1, 7):
                modes = {}
                for mode_index, mode in enumerate(cross_plot.MODES, start=1):
                    value = 100.0 * stage_index + 10.0 + mode_index
                    modes[mode] = pd.DataFrame(
                        {
                            "time": [0.0, 1.0],
                            "recomputed_direct_raw": [value] * 2,
                            "recomputed_direct_scaled": [value + 20.0] * 2,
                            "recomputed_indirect_raw": [value + 40.0] * 2,
                            "recomputed_indirect_scaled": [value + 60.0] * 2,
                        }
                    )
                evaluated[f"case_{case_number}"] = modes
            evaluated_stages[stage] = evaluated

        style = Equation8PlotStyle(dpi=72)
        style.apply()
        figure = cross_plot.plot_all_stages_direct_metric(
            evaluated_stages,
            metric_scale="raw",
            style=style,
        )
        self.addCleanup(plt.close, figure)

        self.assertEqual(len(figure.axes), 18)
        for row in range(3):
            observed = tuple(
                line.get_ydata()[0] * np.sqrt(2.0)
                for line in figure.axes[row * 6].lines
            )
            expected = tuple(100.0 * row + 10.0 + index for index in range(1, 4))
            np.testing.assert_allclose(observed, expected)


if __name__ == "__main__":
    unittest.main()
