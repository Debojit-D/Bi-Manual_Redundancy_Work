"""Plot final static joint-configuration differences.

For each of the six static cases, this script computes the Euclidean distance
between the final seven-joint configurations produced by the direct and
indirect directional-force optimizations. It writes one CSV and the matching
PNG/PDF bar chart to ``<batch_dir>/plots/directional_joint_angles``.
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

from MUJOCO.plotting_scripts.equation8_plot_style import Equation8PlotStyle
from MUJOCO.scripts.plot_main import (
    DEFAULT_BATCH_DIR,
    case_directories,
    discover_stage_run_directories,
)
from MUJOCO.utils.cli import run_cli


MODES = ("directional_force", "directional_force_indirect")
JOINT_COLUMNS = tuple(
    f"q_{arm}{joint}"
    for arm in ("l", "r")
    for joint in range(1, 8)
)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "batch_dir",
        nargs="?",
        type=Path,
        default=DEFAULT_BATCH_DIR,
        help="comparison batch containing the recorded joint-angle CSVs",
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
        help="open the figure after saving",
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


def load_static_runs(batch_dir):
    """Load the direct and indirect directional-force runs for each case."""
    run_directories = discover_stage_run_directories(batch_dir, "static")
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


def calculate_static_final_configuration_metrics(batch_dir):
    """Return final direct-indirect seven-joint distances for both arms."""
    stage_runs = load_static_runs(batch_dir)
    records = []
    for case_name, modes in stage_runs.items():
        direct_final = modes["directional_force"].iloc[-1]
        indirect_final = modes["directional_force_indirect"].iloc[-1]
        left_difference = np.array(
            [
                direct_final[f"q_l{joint}"]
                - indirect_final[f"q_l{joint}"]
                for joint in range(1, 8)
            ],
            dtype=float,
        )
        right_difference = np.array(
            [
                direct_final[f"q_r{joint}"]
                - indirect_final[f"q_r{joint}"]
                for joint in range(1, 8)
            ],
            dtype=float,
        )
        records.append(
            {
                "case": case_name,
                "left_arm_final_configuration_l2_rad": float(
                    np.linalg.norm(left_difference)
                ),
                "right_arm_final_configuration_l2_rad": float(
                    np.linalg.norm(right_difference)
                ),
                "direct_final_time_s": float(direct_final["time"]),
                "indirect_final_time_s": float(indirect_final["time"]),
            }
        )
    return pd.DataFrame.from_records(records)


def plot_static_final_configuration_metrics(metrics):
    """Plot one final-configuration distance per arm and static position."""
    figure, axis = plt.subplots(figsize=(7.16, 3.8))
    positions = np.arange(1, len(metrics) + 1)
    width = 0.36
    left_values = metrics[
        "left_arm_final_configuration_l2_rad"
    ].to_numpy(dtype=float)
    right_values = metrics[
        "right_arm_final_configuration_l2_rad"
    ].to_numpy(dtype=float)
    left_bars = axis.bar(
        positions - width / 2,
        left_values,
        width,
        label="Left arm",
        color="#0072B2",
    )
    right_bars = axis.bar(
        positions + width / 2,
        right_values,
        width,
        label="Right arm",
        color="#D55E00",
    )
    axis.bar_label(left_bars, fmt="%.2f", padding=2, fontsize=8)
    axis.bar_label(right_bars, fmt="%.2f", padding=2, fontsize=8)
    axis.set_title(
        "Static final joint-configuration difference: direct vs indirect"
    )
    axis.set_xlabel("Static table position")
    axis.set_ylabel(r"Final configuration distance $\|\Delta q\|_2$ (rad)")
    axis.set_xticks(positions)
    axis.set_ylim(
        0.0,
        1.13 * max(float(np.max(left_values)), float(np.max(right_values))),
    )
    axis.legend(ncol=2, loc="upper center", frameon=False)
    axis.margins(x=0.04)
    sns.despine(fig=figure, offset=2, trim=False)
    figure.text(
        0.5,
        0.01,
        "Distance between the two final seven-joint vectors for each arm.",
        ha="center",
        va="bottom",
        fontsize=8.5,
    )
    figure.tight_layout(rect=(0.0, 0.045, 1.0, 1.0))
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
    output_dir.mkdir(parents=True, exist_ok=True)

    style = Equation8PlotStyle(dpi=arguments.dpi)
    style.apply()
    metrics = calculate_static_final_configuration_metrics(batch_dir)
    figure = plot_static_final_configuration_metrics(metrics)
    written = list(
        style.save(
            figure,
            output_dir,
            "static_final_arm_configuration_difference",
            arguments.format,
        )
    )
    csv_path = output_dir / "static_final_arm_configuration_difference.csv"
    metrics.to_csv(csv_path, index=False)
    written.append(csv_path)

    print(f"Saved {len(written)} static joint-angle metric files to: {output_dir}")
    for path in written:
        print(f"  {path}")
    if arguments.show:
        plt.show()
    else:
        plt.close(figure)
    return tuple(written)


if __name__ == "__main__":
    run_cli(main)
