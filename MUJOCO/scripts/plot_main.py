"""Plot six-case Equation (8) optimization comparisons from one batch.

The default command creates three figures for the 6D stage. Each figure has
six subplots (one per pose) and draws the matching baseline trace behind the
optimized trace::

    .venv/bin/python -m MUJOCO.scripts.plot_main /path/to/comparison_main_BATCH

Static and ordinary pick/place stages use the same plotting functions and can
be selected with ``--stage static`` or ``--stage pick_place``.
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


STAGES = ("static", "pick_place", "6d_pick_place")
MODES = ("baseline", "velocity", "force", "directional_force")
CASE_PATTERN = re.compile(r"^(?:position|pose)_(\d+)$")


@dataclass(frozen=True)
class OptimizationPlot:
    mode: str
    label: str
    title: str
    raw_column: str
    scaled_column: str
    raw_ylabel: str
    scaled_ylabel: str
    direction: str
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
        title="Velocity manipulability",
        raw_column="velocity_manipulability_raw",
        scaled_column="velocity_manipulability_scaled",
        raw_ylabel="Raw velocity manipulability",
        scaled_ylabel="Scaled velocity manipulability",
        direction="higher is better",
        filename="01_velocity_optimization_six_cases",
    ),
    OptimizationPlot(
        mode="force",
        label="Force optimization",
        title="Force manipulability",
        raw_column="force_manipulability_raw",
        scaled_column="force_manipulability_scaled",
        raw_ylabel="Raw force manipulability",
        scaled_ylabel="Scaled force manipulability",
        direction="higher is better",
        filename="02_force_optimization_six_cases",
    ),
    OptimizationPlot(
        mode="directional_force",
        label="Directional-force optimization",
        title="Directional-force cost",
        raw_column="directional_force_cost_raw",
        scaled_column="directional_force_cost_scaled",
        raw_ylabel="Raw directional-force cost",
        scaled_ylabel="Scaled directional-force cost",
        direction="lower is better",
        filename="03_directional_force_optimization_six_cases",
    ),
)

STAGE_LABELS = {
    "static": "Static",
    "pick_place": "Pick-and-Place",
    "6d_pick_place": "6D Pick-and-Place",
}


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "batch_dir",
        type=Path,
        help="comparison_main timestamp directory containing data/",
    )
    parser.add_argument(
        "--stage",
        choices=(*STAGES, "all"),
        default="6d_pick_place",
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
        2,
        3,
        figsize=style.SIX_PANEL_SIZE,
        sharex=True,
        sharey=True,
    )

    for axis, (case_name, runs) in zip(axes.flat, stage_runs.items()):
        baseline = runs["baseline"]
        optimized = runs[specification.mode]
        sns.lineplot(
            data=baseline,
            x="time",
            y=column,
            estimator=None,
            sort=False,
            legend=False,
            ax=axis,
            **style.baseline_line_kwargs(),
        )
        sns.lineplot(
            data=optimized,
            x="time",
            y=column,
            estimator=None,
            sort=False,
            legend=False,
            ax=axis,
            **style.optimized_line_kwargs(specification.mode),
        )
        axis.set_title(subplot_title(case_name))
        axis.set_xlabel("")
        axis.set_ylabel("")

    figure.suptitle(
        f"{STAGE_LABELS[stage]} — {specification.title} "
        f"({specification.direction})"
    )
    figure.supxlabel("Simulation time (s)")
    figure.supylabel(specification.ylabel(metric_scale))
    style.finish_six_panel_figure(
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


def generate_stage_figures(
    batch_dir,
    stage,
    *,
    output_root,
    metric_scale,
    output_format,
    style,
):
    """Create and save the three initial six-panel figures for one stage."""
    stage_runs = load_stage_runs(batch_dir, stage)
    output_dir = Path(output_root) / stage / "optimization_overlays"
    plotting_functions = (
        plot_velocity_optimization,
        plot_force_optimization,
        plot_directional_force_optimization,
    )
    written = []
    figures = []
    for specification, plotting_function in zip(
        OPTIMIZATION_PLOTS,
        plotting_functions,
    ):
        figure = plotting_function(
            stage_runs,
            stage=stage,
            metric_scale=metric_scale,
            style=style,
        )
        figures.append(figure)
        written.extend(
            style.save(
                figure,
                output_dir,
                specification.filename,
                output_format,
            )
        )
    return tuple(figures), tuple(written)


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
