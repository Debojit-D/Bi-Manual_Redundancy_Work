"""Plot six-case Equation (8) optimization comparisons from one batch.

The default command creates one combined three-objective figure for each
stage. Each comparison draws the matching baseline trace behind the optimized
trace::

    .venv/bin/python -m MUJOCO.scripts.plot_main

Use ``--stage static``, ``--stage pick_place``, or ``--stage 6d_pick_place``
to plot only one stage.

Pass a different batch directory as the first argument to override
``DEFAULT_BATCH_DIR``.
"""

import argparse
from dataclasses import dataclass
from pathlib import Path
import re

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

from MUJOCO.plotting_scripts.equation8_plot_style import Equation8PlotStyle
from MUJOCO.utils.cli import run_cli


# Change this path to select the comparison batch used when no path is passed.
DEFAULT_BATCH_DIR = Path(
    "/home/debojit/debojit/iitgn/Bi-Manual_Redundancy_Work/outputs/"
    "equation8_comparison_batches/comparison_main_20260723_214914_681226"
)

STAGES = ("static", "pick_place", "6d_pick_place")
MODES = ("baseline", "velocity", "force", "directional_force")
MODE_LABELS = {
    "baseline": "Baseline",
    "velocity": "Velocity optimization",
    "force": "Force optimization",
    "directional_force": "Directional-force optimization",
}
TORQUE_COLUMNS = tuple(
    f"tau_act_{arm}{joint}"
    for arm in ("l", "r")
    for joint in range(1, 8)
)
TRACKING_ERROR_PLOTS = (
    (
        "position_error_norm",
        1000.0,
        "Position error (mm)",
        "05_position_error_six_cases",
    ),
    (
        "orientation_error_norm",
        180.0 / np.pi,
        "Orientation error (deg)",
        "06_orientation_error_six_cases",
    ),
)
CASE_PATTERN = re.compile(r"^(?:position|pose)_(\d+)$")


@dataclass(frozen=True)
class OptimizationPlot:
    mode: str
    label: str
    raw_column: str
    scaled_column: str
    raw_ylabel: str
    scaled_ylabel: str
    filename: str

    def column(self, metric_scale):
        return self.raw_column if metric_scale == "raw" else self.scaled_column

    def ylabel(self, metric_scale):
        return (
            self.raw_ylabel
            if metric_scale == "raw"
            else self.scaled_ylabel
        )


OPTIMIZATION_PLOTS = (
    OptimizationPlot(
        mode="velocity",
        label="Velocity optimization",
        raw_column="velocity_manipulability_raw",
        scaled_column="velocity_manipulability_scaled",
        raw_ylabel="Velocity manipulability",
        scaled_ylabel="Scaled velocity manipulability",
        filename="01_velocity_optimization_six_cases",
    ),
    OptimizationPlot(
        mode="force",
        label="Force optimization",
        raw_column="force_manipulability_raw",
        scaled_column="force_manipulability_scaled",
        raw_ylabel="Force manipulability",
        scaled_ylabel="Scaled force manipulability",
        filename="02_force_optimization_six_cases",
    ),
    OptimizationPlot(
        mode="directional_force",
        label="Directional-force optimization",
        raw_column="directional_force_cost_raw",
        scaled_column="directional_force_cost_scaled",
        raw_ylabel="Directional-Force Manipulability",
        scaled_ylabel="Scaled Directional-Force Manipulability",
        filename="03_directional_force_optimization_six_cases",
    ),
)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "batch_dir",
        nargs="?",
        type=Path,
        default=DEFAULT_BATCH_DIR,
        help=(
            "comparison_main timestamp directory containing data/ "
            f"(default: {DEFAULT_BATCH_DIR})"
        ),
    )
    parser.add_argument(
        "--stage",
        choices=(*STAGES, "all"),
        default="all",
        help="experiment stage to plot (default: %(default)s)",
    )
    parser.add_argument(
        "--metric-scale",
        choices=("raw", "scaled"),
        default="raw",
        help=(
            "plot raw optimization quantities or characteristic-length-scaled "
            "CSV diagnostics (default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="figure root (default: <batch_dir>/plots)",
    )
    parser.add_argument(
        "--format",
        choices=("png", "pdf", "both"),
        default="both",
        help="saved figure format (default: %(default)s)",
    )
    parser.add_argument(
        "--dpi",
        type=int,
        default=300,
        help="PNG resolution (default: %(default)s)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="open figures after saving",
    )
    arguments = parser.parse_args(argv)
    if arguments.dpi <= 0:
        parser.error("--dpi must be greater than zero")
    return arguments


def case_number(path):
    match = CASE_PATTERN.fullmatch(path.name)
    if match is None:
        raise ValueError(f"Not a recognized comparison case directory: {path}")
    return int(match.group(1))


def case_directories(path):
    return tuple(
        sorted(
            (
                child
                for child in path.iterdir()
                if child.is_dir() and CASE_PATTERN.fullmatch(child.name)
            ),
            key=case_number,
        )
    )


def discover_stage_run_directory(batch_dir, stage):
    """Find the timestamped CSV directory for one comparison stage."""
    batch_dir = Path(batch_dir).expanduser().resolve()
    stage_root = batch_dir / "data" / stage
    if not stage_root.is_dir():
        raise FileNotFoundError(f"Stage data directory not found: {stage_root}")

    if case_directories(stage_root):
        return stage_root
    candidates = tuple(
        child
        for child in sorted(stage_root.iterdir())
        if child.is_dir() and case_directories(child)
    )
    if not candidates:
        raise FileNotFoundError(
            f"No timestamped six-case CSV directory found in: {stage_root}"
        )
    if len(candidates) > 1:
        names = ", ".join(path.name for path in candidates)
        raise ValueError(
            f"Multiple stage runs found in {stage_root}: {names}. "
            "Supply a batch containing one run per stage."
        )
    return candidates[0]


def required_columns():
    return (
        "time",
        *TORQUE_COLUMNS,
        "position_error_norm",
        "orientation_error_norm",
        *(
            column
            for specification in OPTIMIZATION_PLOTS
            for column in (
                specification.raw_column,
                specification.scaled_column,
            )
        ),
    )


def load_csv(path):
    """Load and validate only the columns needed by the initial plots."""
    path = Path(path)
    frame = pd.read_csv(path, usecols=lambda name: name in required_columns())
    missing = set(required_columns()) - set(frame.columns)
    if missing:
        raise ValueError(f"Missing columns {sorted(missing)} in: {path}")
    if frame.empty:
        raise ValueError(f"CSV contains no samples: {path}")
    numeric = frame.to_numpy(dtype=float)
    if not np.all(np.isfinite(numeric)):
        raise ValueError(f"CSV contains non-finite plot values: {path}")
    return frame


def load_stage_runs(batch_dir, stage):
    """Return six ordered cases, each containing all four mode dataframes."""
    run_dir = discover_stage_run_directory(batch_dir, stage)
    cases = case_directories(run_dir)
    if len(cases) != 6:
        raise ValueError(
            f"Expected six cases in {run_dir}, found {len(cases)}"
        )

    loaded = {}
    for case_dir in cases:
        mode_runs = {}
        for mode in MODES:
            csv_path = case_dir / f"{mode}.csv"
            if not csv_path.is_file():
                raise FileNotFoundError(
                    f"Missing {mode!r} CSV for {case_dir.name}: {csv_path}"
                )
            mode_runs[mode] = load_csv(csv_path)
        loaded[case_dir.name] = mode_runs
    return loaded


def subplot_title(case_name):
    number = case_name.rsplit("_", maxsplit=1)[-1]
    label = "Pose" if case_name.startswith("pose_") else "Position"
    return f"{label} {number}"


def extend_final_value(time, values, end_time):
    """Hold the final sample constant through a later comparison horizon."""
    time = np.asarray(time, dtype=float)
    values = np.asarray(values, dtype=float)
    end_time = float(end_time)
    if end_time <= time[-1]:
        return time, values
    return (
        np.append(time, end_time),
        np.append(values, values[-1]),
    )


def plot_optimization_grid(
    stage_runs,
    specification,
    *,
    stage,
    metric_scale,
    style,
):
    """Plot one objective across six cases with baseline behind each trace."""
    column = specification.column(metric_scale)
    figure, axes = plt.subplots(
        1,
        6,
        figsize=style.SIX_PANEL_SIZE,
        sharex=True,
        sharey=True,
    )

    for axis, (case_name, runs) in zip(axes.flat, stage_runs.items()):
        baseline = runs["baseline"]
        optimized = runs[specification.mode]
        baseline_time = baseline["time"].to_numpy(dtype=float)
        baseline_values = baseline[column].to_numpy(dtype=float)
        optimized_time = optimized["time"].to_numpy(dtype=float)
        optimized_values = optimized[column].to_numpy(dtype=float)
        if stage == "static":
            baseline_time, baseline_values = extend_final_value(
                baseline_time,
                baseline_values,
                optimized_time[-1],
            )
        sns.lineplot(
            x=baseline_time,
            y=baseline_values,
            estimator=None,
            sort=False,
            legend=False,
            ax=axis,
            **style.baseline_line_kwargs(),
        )
        sns.lineplot(
            x=optimized_time,
            y=optimized_values,
            estimator=None,
            sort=False,
            legend=False,
            ax=axis,
            **style.optimized_line_kwargs(specification.mode),
        )
        axis.set_title(subplot_title(case_name))
        axis.set_xlabel("")
        axis.set_ylabel("")

    figure.supxlabel("Simulation time (s)", y=0.005)
    figure.supylabel(specification.ylabel(metric_scale), x=0.025)
    style.finish_six_panel_row_figure(
        figure,
        axes,
        mode=specification.mode,
        mode_label=specification.label,
    )
    return figure


def plot_velocity_optimization(stage_runs, **kwargs):
    return plot_optimization_grid(
        stage_runs,
        OPTIMIZATION_PLOTS[0],
        **kwargs,
    )


def plot_force_optimization(stage_runs, **kwargs):
    return plot_optimization_grid(
        stage_runs,
        OPTIMIZATION_PLOTS[1],
        **kwargs,
    )


def plot_directional_force_optimization(stage_runs, **kwargs):
    return plot_optimization_grid(
        stage_runs,
        OPTIMIZATION_PLOTS[2],
        **kwargs,
    )


def plot_combined_optimization_grid(
    stage_runs,
    *,
    stage,
    metric_scale,
    style,
):
    """Plot all three objectives as rows across the six comparison cases."""
    figure, axes = plt.subplots(
        3,
        6,
        figsize=style.THREE_BY_SIX_SIZE,
        sharex=False if stage == "static" else "col",
        sharey="row",
    )

    for row, specification in enumerate(OPTIMIZATION_PLOTS):
        column = specification.column(metric_scale)
        for axis, (case_name, runs) in zip(
            axes[row],
            stage_runs.items(),
        ):
            baseline = runs["baseline"]
            optimized = runs[specification.mode]
            baseline_time = baseline["time"].to_numpy(dtype=float)
            baseline_values = baseline[column].to_numpy(dtype=float)
            optimized_time = optimized["time"].to_numpy(dtype=float)
            optimized_values = optimized[column].to_numpy(dtype=float)
            if stage == "static":
                baseline_time, baseline_values = extend_final_value(
                    baseline_time,
                    baseline_values,
                    optimized_time[-1],
                )
            sns.lineplot(
                x=baseline_time,
                y=baseline_values,
                estimator=None,
                sort=False,
                legend=False,
                ax=axis,
                **style.baseline_line_kwargs(),
            )
            sns.lineplot(
                x=optimized_time,
                y=optimized_values,
                estimator=None,
                sort=False,
                legend=False,
                ax=axis,
                **style.optimized_line_kwargs(specification.mode),
            )
            axis.set_xlabel("")
            axis.set_ylabel("")
            if row == 0:
                axis.set_title(subplot_title(case_name))
        metric_label = {
            "velocity": "Velocity\nmanipulability",
            "force": "Force\nmanipulability",
            "directional_force": "Directional-Force\nManipulability",
        }[specification.mode]
        row_label = (
            f"{metric_label}\n(scaled)"
            if metric_scale == "scaled"
            else metric_label
        )
        axes[row, 0].set_ylabel(
            row_label,
            rotation=90,
            ha="center",
            va="center",
            labelpad=14,
        )

    figure.supxlabel("Simulation time (s)", y=0.005)
    figure.align_ylabels(axes[:, 0])
    style.finish_all_modes_six_panel_figure(
        figure,
        axes,
        mode_labels=MODE_LABELS,
    )
    return figure


def actuator_effort(run):
    """Return sqrt(tau tau^T), the combined norm of all 14 actuator torques."""
    torque = run.loc[:, TORQUE_COLUMNS].to_numpy(dtype=float)
    return np.sqrt(np.sum(torque * torque, axis=1))


def plot_actuator_effort(stage_runs, *, stage, style):
    """Plot all four modes' actuator effort across the six cases."""
    figure, axes = plt.subplots(
        2,
        3,
        figsize=style.SIX_PANEL_GRID_SIZE,
        sharex=False,
        sharey=False,
    )

    for axis, (case_name, runs) in zip(axes.flat, stage_runs.items()):
        comparison_end_time = max(
            float(run["time"].iloc[-1]) for run in runs.values()
        )
        for mode in MODES:
            run = runs[mode]
            time = run["time"].to_numpy(dtype=float)
            effort = actuator_effort(run)
            if stage == "static" and mode == "baseline":
                time, effort = extend_final_value(
                    time,
                    effort,
                    comparison_end_time,
                )
            line_kwargs = (
                style.baseline_line_kwargs()
                if mode == "baseline"
                else style.optimized_line_kwargs(mode)
            )
            sns.lineplot(
                x=time,
                y=effort,
                estimator=None,
                sort=False,
                legend=False,
                ax=axis,
                **line_kwargs,
            )
        axis.set_title(subplot_title(case_name))
        axis.set_xlabel("")
        axis.set_ylabel("")

    figure.supxlabel("Simulation time (s)", y=0.005)
    figure.supylabel("Actuator effort, L2 norm (N m)", x=0.025)
    style.finish_all_modes_six_panel_figure(
        figure,
        axes,
        mode_labels=MODE_LABELS,
    )
    return figure


def plot_tracking_error(
    stage_runs,
    *,
    error_column,
    scale,
    ylabel,
    style,
):
    """Plot one object-pose tracking error norm for all modes and poses."""
    figure, axes = plt.subplots(
        1,
        6,
        figsize=style.SIX_PANEL_SIZE,
        sharex=True,
        sharey=True,
    )

    for axis, (case_name, runs) in zip(axes.flat, stage_runs.items()):
        for mode in MODES:
            run = runs[mode]
            line_kwargs = (
                style.baseline_line_kwargs()
                if mode == "baseline"
                else style.optimized_line_kwargs(mode)
            )
            sns.lineplot(
                x=run["time"],
                y=scale * run[error_column],
                estimator=None,
                sort=False,
                legend=False,
                ax=axis,
                **line_kwargs,
            )
        axis.set_title(subplot_title(case_name))
        axis.set_xlabel("")
        axis.set_ylabel("")

    axes.flat[0].set_ylim(bottom=0.0)
    figure.supxlabel("Simulation time (s)", y=0.005)
    figure.supylabel(ylabel, x=0.025)
    style.finish_all_modes_six_panel_row_figure(
        figure,
        axes,
        mode_labels=MODE_LABELS,
    )
    return figure


def generate_stage_figures(
    batch_dir,
    stage,
    *,
    output_root,
    metric_scale,
    output_format,
    style,
):
    """Create and save only the combined three-by-six figure for one stage."""
    stage_runs = load_stage_runs(batch_dir, stage)
    output_dir = Path(output_root) / "combined_plots"
    combined_figure = plot_combined_optimization_grid(
        stage_runs,
        stage=stage,
        metric_scale=metric_scale,
        style=style,
    )
    written = style.save(
        combined_figure,
        output_dir,
        f"{stage}_combined_optimization_three_by_six",
        output_format,
    )
    return (combined_figure,), tuple(written)


def main(argv=None):
    arguments = parse_arguments(argv)
    batch_dir = arguments.batch_dir.expanduser().resolve()
    if not batch_dir.is_dir():
        raise FileNotFoundError(f"Batch directory not found: {batch_dir}")
    output_root = (
        arguments.output_dir.expanduser().resolve()
        if arguments.output_dir is not None
        else batch_dir / "plots"
    )
    stages = STAGES if arguments.stage == "all" else (arguments.stage,)

    style = Equation8PlotStyle(dpi=arguments.dpi)
    style.apply()
    all_figures = []
    all_written = []
    for stage in stages:
        figures, written = generate_stage_figures(
            batch_dir,
            stage,
            output_root=output_root,
            metric_scale=arguments.metric_scale,
            output_format=arguments.format,
            style=style,
        )
        all_figures.extend(figures)
        all_written.extend(written)

    print(f"Saved {len(all_written)} figure files to: {output_root}")
    for path in all_written:
        print(f"  {path}")
    if arguments.show:
        plt.show()
    else:
        plt.close("all")
    return tuple(all_written)


if __name__ == "__main__":
    run_cli(main)
