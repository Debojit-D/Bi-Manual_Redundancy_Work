"""Compare direct and indirect directional-force joint angles over time.

The default command creates one comprehensive figure for each comparison
stage. Each figure has one row per robot joint and one column per experiment
case, with only the direct and indirect directional-force runs plotted::

    .venv/bin/python -m \
        MUJOCO.scripts.plot_directional_joint_angle_comparison
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import MaxNLocator
import numpy as np
import pandas as pd
import seaborn as sns

from MUJOCO.plotting_scripts.equation8_plot_style import Equation8PlotStyle
from MUJOCO.scripts.plot_main import (
    DEFAULT_BATCH_DIR,
    STAGES,
    case_directories,
    discover_stage_run_directories,
    subplot_title,
)
from MUJOCO.utils.cli import run_cli


MODES = ("directional_force", "directional_force_indirect")
MODE_LABELS = {
    "directional_force": "Directional - Force Manipulability",
    "directional_force_indirect": (
        "Directional - Force Manipulability (Indirect)"
    ),
}
JOINT_COLUMNS = tuple(
    f"q_{arm}{joint}"
    for arm in ("l", "r")
    for joint in range(1, 8)
)
STAGE_LABELS = {
    "static": "Static",
    "pick_place": "Pick-and-place",
    "6d_pick_place": "6D pick-and-place",
}


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
        help="comparison batch containing the recorded joint-angle CSVs",
    )
    parser.add_argument(
        "--stage",
        choices=(*STAGES, "all"),
        default="all",
        help="experiment stage to plot (default: %(default)s)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help=(
            "output directory "
            "(default: <batch_dir>/plots/directional_joint_angles)"
        ),
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
        help="open the figures after saving",
    )
    arguments = parser.parse_args(argv)
    if arguments.dpi <= 0:
        parser.error("--dpi must be greater than zero")
    return arguments


def load_joint_angle_csv(path):
    """Load and validate time plus all fourteen recorded arm angles."""
    required = ("time", *JOINT_COLUMNS)
    frame = pd.read_csv(path, usecols=lambda column: column in required)
    missing = set(required) - set(frame.columns)
    if missing:
        raise ValueError(f"Missing columns {sorted(missing)} in: {path}")
    if frame.empty:
        raise ValueError(f"CSV contains no samples: {path}")
    if not np.all(np.isfinite(frame.to_numpy(dtype=float))):
        raise ValueError(f"CSV contains non-finite joint-angle data: {path}")
    return frame


def load_stage_runs(batch_dir, stage):
    """Load the direct and indirect run for every case in one stage."""
    run_directories = discover_stage_run_directories(batch_dir, stage)
    case_names = sorted(
        {
            case_directory.name
            for run_directory in run_directories
            for case_directory in case_directories(run_directory)
        },
        key=lambda name: int(name.rsplit("_", maxsplit=1)[-1]),
    )
    if len(case_names) != 6:
        raise ValueError(
            f"Expected six cases across {run_directories}, "
            f"found {len(case_names)}"
        )

    loaded = {}
    for case_name in case_names:
        loaded[case_name] = {}
        for mode in MODES:
            matches = tuple(
                run_directory / case_name / f"{mode}.csv"
                for run_directory in run_directories
                if (run_directory / case_name / f"{mode}.csv").is_file()
            )
            if len(matches) != 1:
                raise ValueError(
                    f"Expected one {mode!r} CSV for {case_name}; "
                    f"found {matches}"
                )
            loaded[case_name][mode] = load_joint_angle_csv(matches[0])
    return loaded


def joint_label(column):
    arm = "L" if column[2] == "l" else "R"
    return rf"$q_{{{arm}{column[3:]}}}$ (rad)"


def plot_stage_joint_angles(stage_runs, *, stage, style):
    """Plot 14 joints by six cases with two optimization traces per panel."""
    figure, axes = plt.subplots(
        len(JOINT_COLUMNS),
        len(stage_runs),
        figsize=(12.0, 18.0),
        sharex="col",
        sharey="row",
        squeeze=False,
    )

    for column_index, (case_name, runs) in enumerate(stage_runs.items()):
        for row_index, joint_column in enumerate(JOINT_COLUMNS):
            axis = axes[row_index, column_index]
            for mode in MODES:
                run = runs[mode]
                axis.plot(
                    run["time"],
                    run[joint_column],
                    label=MODE_LABELS[mode],
                    **style.optimized_line_kwargs(mode),
                )
            if row_index == 0:
                axis.set_title(subplot_title(case_name))
            if column_index == 0:
                axis.set_ylabel(
                    joint_label(joint_column),
                    rotation=0,
                    ha="right",
                    va="center",
                    labelpad=8,
                )
            axis.margins(x=0)
            axis.ticklabel_format(axis="y", style="plain", useOffset=False)
            axis.yaxis.set_major_locator(MaxNLocator(nbins=3))
            axis.xaxis.set_major_locator(MaxNLocator(nbins=4))

    handles = tuple(
        Line2D(
            [0],
            [0],
            label=MODE_LABELS[mode],
            **style.optimized_line_kwargs(mode),
        )
        for mode in MODES
    )
    figure.suptitle(
        f"{STAGE_LABELS[stage]} joint-angle evolution",
        y=0.997,
    )
    figure.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.982),
        ncol=2,
        frameon=True,
        fancybox=False,
        framealpha=1.0,
        facecolor="white",
        edgecolor="#68737D",
    )
    figure.supxlabel("Simulation time (s)", y=0.004)
    figure.align_ylabels(axes[:, 0])
    sns.despine(fig=figure, offset=2, trim=False)
    figure.tight_layout(
        rect=(0.025, 0.012, 1.0, 0.958),
        pad=0.35,
        h_pad=0.25,
        w_pad=0.45,
    )
    return figure


def main(argv=None):
    arguments = parse_arguments(argv)
    batch_dir = arguments.batch_dir.expanduser().resolve()
    if not batch_dir.is_dir():
        raise FileNotFoundError(f"Batch directory not found: {batch_dir}")
    output_dir = (
        arguments.output_dir.expanduser().resolve()
        if arguments.output_dir is not None
        else batch_dir / "plots" / "directional_joint_angles"
    )
    stages = STAGES if arguments.stage == "all" else (arguments.stage,)

    style = Equation8PlotStyle(dpi=arguments.dpi)
    style.apply()
    written = []
    for stage in stages:
        stage_runs = load_stage_runs(batch_dir, stage)
        figure = plot_stage_joint_angles(
            stage_runs,
            stage=stage,
            style=style,
        )
        written.extend(
            style.save(
                figure,
                output_dir,
                f"{stage}_directional_joint_angles",
                arguments.format,
            )
        )

    print(f"Saved {len(written)} joint-angle figure files to: {output_dir}")
    for path in written:
        print(f"  {path}")
    if arguments.show:
        plt.show()
    else:
        plt.close("all")
    return tuple(written)


if __name__ == "__main__":
    run_cli(main)
