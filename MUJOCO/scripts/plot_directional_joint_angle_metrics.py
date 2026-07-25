"""Summarize direct-versus-indirect joint trajectory differences.

For each joint, the comparison metric is the time-aligned trajectory RMSE

    RMSE_j = sqrt((1 / T) integral (q_direct_j(t)-q_indirect_j(t))^2 dt).

The shorter run's final joint configuration is held through the longer run's
end time. Each combined-arm value is the RMS of its seven joint RMSE values.
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import seaborn as sns

from MUJOCO.plotting_scripts.equation8_plot_style import Equation8PlotStyle
from MUJOCO.scripts.plot_directional_joint_angle_comparison import (
    JOINT_COLUMNS,
    load_stage_runs,
)
from MUJOCO.scripts.plot_main import DEFAULT_BATCH_DIR, STAGES
from MUJOCO.utils.cli import run_cli


STAGE_LABELS = {
    "static": "Static",
    "pick_place": "Pick-and-place",
    "6d_pick_place": "6D pick-and-place",
}
DISPLAY_COLUMNS = (
    *(rf"$q_{{L{joint}}}$" for joint in range(1, 8)),
    "Left arm\ncombined",
    *(rf"$q_{{R{joint}}}$" for joint in range(1, 8)),
    "Right arm\ncombined",
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


def trajectory_rmse(direct, indirect, joint_column):
    """Return time-weighted RMSE after aligning two recorded trajectories."""
    direct_time = direct["time"].to_numpy(dtype=float)
    indirect_time = indirect["time"].to_numpy(dtype=float)
    if np.any(np.diff(direct_time) <= 0.0):
        raise ValueError("Direct-run timestamps must be strictly increasing")
    if np.any(np.diff(indirect_time) <= 0.0):
        raise ValueError("Indirect-run timestamps must be strictly increasing")

    common_time = np.union1d(direct_time, indirect_time)
    duration = float(common_time[-1] - common_time[0])
    if duration <= 0.0:
        raise ValueError("Joint trajectories must span a positive duration")
    direct_values = np.interp(
        common_time,
        direct_time,
        direct[joint_column].to_numpy(dtype=float),
    )
    indirect_values = np.interp(
        common_time,
        indirect_time,
        indirect[joint_column].to_numpy(dtype=float),
    )
    squared_difference = (direct_values - indirect_values) ** 2
    mean_squared_difference = (
        np.trapezoid(squared_difference, common_time) / duration
    )
    return float(np.sqrt(mean_squared_difference))


def calculate_metrics(batch_dir):
    """Return one row of per-joint and combined-arm metrics per case."""
    rows = []
    row_labels = []
    records = []
    for stage in STAGES:
        stage_runs = load_stage_runs(batch_dir, stage)
        for case_number, (case_name, modes) in enumerate(
            stage_runs.items(),
            start=1,
        ):
            direct = modes["directional_force"]
            indirect = modes["directional_force_indirect"]
            joint_values = np.array(
                [
                    trajectory_rmse(direct, indirect, joint_column)
                    for joint_column in JOINT_COLUMNS
                ]
            )
            left_combined = float(np.sqrt(np.mean(joint_values[:7] ** 2)))
            right_combined = float(np.sqrt(np.mean(joint_values[7:] ** 2)))
            display_values = np.concatenate(
                (
                    joint_values[:7],
                    [left_combined],
                    joint_values[7:],
                    [right_combined],
                )
            )
            rows.append(display_values)
            case_label = "Pose" if case_name.startswith("pose_") else "Position"
            row_labels.append(
                f"{STAGE_LABELS[stage]} — {case_label} {case_number}"
            )
            record = {
                "stage": stage,
                "case": case_name,
                **dict(zip(JOINT_COLUMNS, joint_values)),
                "left_arm_combined_rmse": left_combined,
                "right_arm_combined_rmse": right_combined,
            }
            records.append(record)

    heatmap_frame = pd.DataFrame(
        np.asarray(rows),
        index=row_labels,
        columns=DISPLAY_COLUMNS,
    )
    metrics_frame = pd.DataFrame.from_records(records)
    return heatmap_frame, metrics_frame


def plot_metric_heatmap(metrics):
    """Create one annotated heatmap for all stages, cases, and joints."""
    figure, axis = plt.subplots(figsize=(13.0, 8.2))
    sns.heatmap(
        metrics,
        ax=axis,
        cmap="mako",
        vmin=0.0,
        annot=True,
        fmt=".2f",
        linewidths=0.35,
        linecolor="white",
        cbar_kws={
            "label": "Direct–indirect trajectory RMSE (rad)",
            "shrink": 0.86,
        },
        annot_kws={"fontsize": 7.0},
    )
    axis.set_title(
        "Joint-angle differences: direct vs indirect directional-force "
        "optimization",
        pad=14,
    )
    axis.set_xlabel("")
    axis.set_ylabel("")
    axis.tick_params(axis="x", rotation=0)
    axis.tick_params(axis="y", rotation=0)

    for boundary in (6, 12):
        axis.axhline(boundary, color="#212529", linewidth=1.6)
    for boundary in (7, 8, 15):
        axis.axvline(boundary, color="#212529", linewidth=1.25)

    for index, tick_label in enumerate(axis.get_xticklabels()):
        if index in (7, 15):
            tick_label.set_fontweight("bold")

    figure.text(
        0.5,
        0.012,
        "Per-joint time-aligned RMSE; combined columns are the RMS across "
        "the seven joints of that arm.",
        ha="center",
        va="bottom",
        fontsize=9,
    )
    figure.tight_layout(rect=(0.0, 0.035, 1.0, 1.0))
    return figure


def calculate_static_final_configuration_metrics(batch_dir):
    """Return final direct–indirect seven-joint distances for both arms."""
    stage_runs = load_stage_runs(batch_dir, "static")
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
    heatmap_metrics, tabular_metrics = calculate_metrics(batch_dir)
    figure = plot_metric_heatmap(heatmap_metrics)
    written = list(
        style.save(
            figure,
            output_dir,
            "directional_joint_angle_rmse_summary",
            arguments.format,
        )
    )
    csv_path = output_dir / "directional_joint_angle_rmse_summary.csv"
    tabular_metrics.to_csv(csv_path, index=False)
    written.append(csv_path)

    final_metrics = calculate_static_final_configuration_metrics(batch_dir)
    final_figure = plot_static_final_configuration_metrics(final_metrics)
    written.extend(
        style.save(
            final_figure,
            output_dir,
            "static_final_arm_configuration_difference",
            arguments.format,
        )
    )
    final_csv_path = (
        output_dir / "static_final_arm_configuration_difference.csv"
    )
    final_metrics.to_csv(final_csv_path, index=False)
    written.append(final_csv_path)

    print(f"Saved {len(written)} joint-angle metric files to: {output_dir}")
    for path in written:
        print(f"  {path}")
    if arguments.show:
        plt.show()
    else:
        plt.close(figure)
        plt.close(final_figure)
    return tuple(written)


if __name__ == "__main__":
    run_cli(main)
