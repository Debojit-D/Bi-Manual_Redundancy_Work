"""Plot six-case Equation (8) optimization comparisons from one batch.

The default command creates the original combined three-objective figure and
the combined four-objective V2 figure for each stage. Every objective draws its
matching baseline trace behind the optimized trace. For older baseline CSVs
that predate the indirect metric, that trace is reconstructed from the recorded
joint and object poses::

    .venv/bin/python -m bimanual_redundancy.plotting.plot_main

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
from matplotlib.patches import Patch
import numpy as np
import pandas as pd
import seaborn as sns
from scipy.spatial.transform import Rotation

from bimanual_redundancy.plotting.equation8_plot_style import Equation8PlotStyle
from bimanual_redundancy.simulation.cli import run_cli
from bimanual_redundancy.core import (
    CooperativeManipulationKinematics,
)
from bimanual_redundancy.simulation import DualFrankaMuJoCoScene


# Change this path to select the comparison batch used when no path is passed.
DEFAULT_BATCH_DIR = Path(
    "/home/debojit/debojit/iitgn/Bi-Manual_Redundancy_Work/outputs/"
    "equation8_comparison_batches/comparison_main_20260723_214914_681226"
)

STAGES = ("static", "pick_place", "6d_pick_place")
MODES = (
    "baseline",
    "velocity",
    "force",
    "directional_force",
    "directional_force_indirect",
)
TWO_ROW_LEGEND_HANDLE_ORDER = (0, 3, 1, 4, 2)
MODE_LABELS = {
    "baseline": "Baseline",
    "velocity": "Velocity Manipulability",
    "force": "Force Manipulability",
    "directional_force": "Directional Force Manipulability",
    "directional_force_indirect": (
        "Directional Force Manipulability (Indirect)"
    ),
}
MODE_ABBREVIATIONS = {
    "baseline": "B",
    "velocity": "V",
    "force": "F",
    "directional_force": "DF",
    "directional_force_indirect": r"DF$_i$",
}
ACTUATOR_MODE_LABELS = {
    mode: f"{MODE_ABBREVIATIONS[mode]}: {MODE_LABELS[mode]}"
    for mode in MODES
}
TORQUE_COLUMNS = tuple(
    f"tau_act_{arm}{joint}"
    for arm in ("l", "r")
    for joint in range(1, 8)
)
ARM_JOINT_COLUMNS = tuple(
    f"q_{arm}{joint}"
    for arm in ("l", "r")
    for joint in range(1, 8)
)
OBJECT_POSE_COLUMNS = (
    "object_x",
    "object_y",
    "object_z",
    "object_qw",
    "object_qx",
    "object_qy",
    "object_qz",
)
INDIRECT_BASELINE_RAW_COLUMN = "directional_force_indirect_cost_raw"
INDIRECT_BASELINE_SCALED_COLUMN = "directional_force_indirect_cost_scaled"
ARM_BASE_OFFSET = 0.25
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
    baseline_raw_column: str | None = None
    baseline_scaled_column: str | None = None
    include_baseline: bool = True
    plot_divisor: float = 1.0

    def column(self, metric_scale):
        return self.raw_column if metric_scale == "raw" else self.scaled_column

    def baseline_column(self, metric_scale):
        if metric_scale == "raw":
            return self.baseline_raw_column or self.raw_column
        return self.baseline_scaled_column or self.scaled_column

    def ylabel(self, metric_scale):
        return (
            self.raw_ylabel
            if metric_scale == "raw"
            else self.scaled_ylabel
        )

    def plot_values(self, values):
        """Apply presentation-only normalization to a metric series."""
        return np.asarray(values, dtype=float) / self.plot_divisor


OPTIMIZATION_PLOTS = (
    OptimizationPlot(
        mode="velocity",
        label="Velocity Manipulability",
        raw_column="velocity_manipulability_raw",
        scaled_column="velocity_manipulability_scaled",
        raw_ylabel="Velocity manipulability",
        scaled_ylabel="Scaled velocity manipulability",
        filename="01_velocity_optimization_six_cases",
    ),
    OptimizationPlot(
        mode="force",
        label="Force Manipulability",
        raw_column="force_manipulability_raw",
        scaled_column="force_manipulability_scaled",
        raw_ylabel="Force manipulability",
        scaled_ylabel="Scaled force manipulability",
        filename="02_force_optimization_six_cases",
    ),
    OptimizationPlot(
        mode="directional_force",
        label="Directional Force Manipulability",
        raw_column="directional_force_cost_raw",
        scaled_column="directional_force_cost_scaled",
        raw_ylabel="Directional - Force Manipulability",
        scaled_ylabel="Scaled Directional - Force Manipulability",
        filename="03_directional_force_optimization_six_cases",
        plot_divisor=np.sqrt(2.0),
    ),
    OptimizationPlot(
        mode="directional_force_indirect",
        label="Directional Force Manipulability (Indirect)",
        raw_column="paper_objective_raw",
        scaled_column="paper_objective_scaled",
        raw_ylabel="Directional - Force Manipulability (Indirect)",
        scaled_ylabel="Scaled Directional - Force Manipulability (Indirect)",
        filename="04_directional_force_indirect_optimization_six_cases",
        baseline_raw_column=INDIRECT_BASELINE_RAW_COLUMN,
        baseline_scaled_column=INDIRECT_BASELINE_SCALED_COLUMN,
        plot_divisor=np.sqrt(2.0),
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


def discover_stage_run_directories(batch_dir, stage):
    """Find every timestamped CSV directory contributing modes to a stage."""
    batch_dir = Path(batch_dir).expanduser().resolve()
    stage_root = batch_dir / "data" / stage
    if not stage_root.is_dir():
        raise FileNotFoundError(f"Stage data directory not found: {stage_root}")
    if case_directories(stage_root):
        return (stage_root,)
    candidates = tuple(
        child
        for child in sorted(stage_root.iterdir())
        if child.is_dir() and case_directories(child)
    )
    if not candidates:
        raise FileNotFoundError(
            f"No timestamped six-case CSV directory found in: {stage_root}"
        )
    return candidates


def required_columns():
    return (
        "time",
        *TORQUE_COLUMNS,
        *ARM_JOINT_COLUMNS,
        *OBJECT_POSE_COLUMNS,
        "characteristic_length_m",
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


def normalized_frobenius_distance(capability, desired):
    """Return the trace-normalized Frobenius distance used by the optimizer."""
    capability_trace = float(np.trace(capability))
    desired_trace = float(np.trace(desired))
    if capability_trace <= 0.0 or desired_trace <= 0.0:
        raise ValueError("Capability and desired matrices need positive traces")
    return float(
        np.linalg.norm(
            capability / capability_trace - desired / desired_trace,
            ord="fro",
        )
    )


def add_indirect_baseline_metrics(loaded):
    """Reconstruct the indirect metric from each recorded baseline state."""
    scene = DualFrankaMuJoCoScene(
        control_hz=50.0,
        left_arm_base_position=np.array([0.0, -ARM_BASE_OFFSET, 0.0]),
        right_arm_base_position=np.array([0.0, ARM_BASE_OFFSET, 0.0]),
        left_arm_base_euler_xyz_degrees=np.zeros(3),
        right_arm_base_euler_xyz_degrees=np.zeros(3),
        show_mocap_targets=False,
        enable_bias_compensation=True,
    )
    kinematics = CooperativeManipulationKinematics(
        scene.model,
        scene.left_arm_dofs,
        scene.right_arm_dofs,
    )
    desired = np.diag([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

    for runs in loaded.values():
        baseline = runs["baseline"]
        arm_states = baseline.loc[:, ARM_JOINT_COLUMNS].to_numpy(dtype=float)
        object_poses = baseline.loc[:, OBJECT_POSE_COLUMNS].to_numpy(dtype=float)
        characteristic_lengths = baseline[
            "characteristic_length_m"
        ].to_numpy(dtype=float)
        raw_values = np.empty(len(baseline), dtype=float)
        scaled_values = np.empty(len(baseline), dtype=float)

        for sample, (arm_state, object_pose, characteristic_length) in enumerate(
            zip(arm_states, object_poses, characteristic_lengths)
        ):
            position = object_pose[:3]
            quaternion_xyzw = np.array(
                [
                    object_pose[4],
                    object_pose[5],
                    object_pose[6],
                    object_pose[3],
                ]
            )
            scene.data.qpos[scene.arm_qpos] = arm_state
            scene.set_table_reference_pose(
                position,
                Rotation.from_quat(quaternion_xyzw).as_matrix(),
            )

            velocity_map = kinematics.paper_object_velocity_map(scene.data)
            velocity_capability = velocity_map @ velocity_map.T
            raw_values[sample] = normalized_frobenius_distance(
                velocity_capability,
                desired,
            )

            spatial_scaling = np.diag(
                [1.0, 1.0, 1.0]
                + [float(characteristic_length)] * 3
            )
            scaled_velocity_map = spatial_scaling @ velocity_map
            scaled_velocity_capability = (
                scaled_velocity_map @ scaled_velocity_map.T
            )
            scaled_values[sample] = normalized_frobenius_distance(
                scaled_velocity_capability,
                desired,
            )

        baseline[INDIRECT_BASELINE_RAW_COLUMN] = raw_values
        baseline[INDIRECT_BASELINE_SCALED_COLUMN] = scaled_values


def load_stage_runs(batch_dir, stage):
    """Merge timestamped runs into six cases containing all five modes."""
    run_dirs = discover_stage_run_directories(batch_dir, stage)
    case_names = sorted(
        {
            case_dir.name
            for run_dir in run_dirs
            for case_dir in case_directories(run_dir)
        },
        key=lambda name: int(name.rsplit("_", maxsplit=1)[-1]),
    )
    if len(case_names) != 6:
        raise ValueError(
            f"Expected six cases across {run_dirs}, found {len(case_names)}"
        )

    loaded = {}
    for case_name in case_names:
        mode_runs = {}
        for mode in MODES:
            matches = tuple(
                run_dir / case_name / f"{mode}.csv"
                for run_dir in run_dirs
                if (run_dir / case_name / f"{mode}.csv").is_file()
            )
            if not matches:
                raise FileNotFoundError(
                    f"Missing {mode!r} CSV for {case_name} in: {run_dirs}"
                )
            if len(matches) > 1:
                raise ValueError(
                    f"Multiple {mode!r} CSVs found for {case_name}: {matches}"
                )
            mode_runs[mode] = load_csv(matches[0])
        loaded[case_name] = mode_runs
    add_indirect_baseline_metrics(loaded)
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
    baseline_column = specification.baseline_column(metric_scale)
    figure, axes = plt.subplots(
        1,
        6,
        figsize=style.SIX_PANEL_SIZE,
        sharex=True,
        sharey=True,
    )

    for axis, (case_name, runs) in zip(axes.flat, stage_runs.items()):
        optimized = runs[specification.mode]
        optimized_time = optimized["time"].to_numpy(dtype=float)
        optimized_values = specification.plot_values(optimized[column])
        if specification.include_baseline:
            baseline = runs["baseline"]
            baseline_time = baseline["time"].to_numpy(dtype=float)
            baseline_values = specification.plot_values(
                baseline[baseline_column]
            )
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


def plot_directional_force_indirect_optimization(stage_runs, **kwargs):
    return plot_optimization_grid(
        stage_runs,
        OPTIMIZATION_PLOTS[3],
        **kwargs,
    )


def plot_combined_optimization_grid(
    stage_runs,
    *,
    stage,
    metric_scale,
    style,
    specifications=OPTIMIZATION_PLOTS,
):
    """Plot the selected objectives as rows across all six comparison cases."""
    row_count = len(specifications)
    figure, axes = plt.subplots(
        row_count,
        6,
        figsize=(
            (
                style.REDUCED_FOUR_BY_SIX_SIZE
                if stage in ("static", "6d_pick_place")
                else style.FOUR_BY_SIX_SIZE
            )
            if row_count == 4
            else style.THREE_BY_SIX_SIZE
        ),
        sharex=False if stage == "static" else "col",
        sharey="row",
    )

    for row, specification in enumerate(specifications):
        column = specification.column(metric_scale)
        baseline_column = specification.baseline_column(metric_scale)
        for axis, (case_name, runs) in zip(
            axes[row],
            stage_runs.items(),
        ):
            optimized = runs[specification.mode]
            optimized_time = optimized["time"].to_numpy(dtype=float)
            optimized_values = specification.plot_values(optimized[column])
            if specification.include_baseline:
                baseline = runs["baseline"]
                baseline_time = baseline["time"].to_numpy(dtype=float)
                baseline_values = specification.plot_values(
                    baseline[baseline_column]
                )
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
                if stage in ("static", "6d_pick_place"):
                    axis.set_title(subplot_title(case_name), pad=8.0)
                else:
                    axis.set_title(subplot_title(case_name))
        metric_label = {
            "velocity": "Velocity\nmanipulability",
            "force": "Force\nmanipulability",
            "directional_force": "Directional - Force\nmanipulability",
            "directional_force_indirect": (
                "Directional - Force\nmanipulability (Indirect)"
            ),
        }[specification.mode]
        if row_count == 3:
            direction = {
                "velocity": r"$\uparrow$",
                "force": r"$\uparrow$",
                "directional_force": r"$\downarrow$",
            }[specification.mode]
            metric_label = f"{metric_label} ({direction})"
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
            fontsize=8.0,
        )

    figure.supxlabel(
        "Simulation time (s)",
        y=0.018 if stage in ("static", "6d_pick_place") else 0.005,
    )
    figure.align_ylabels(axes[:, 0])
    mode_labels = {
        "baseline": MODE_LABELS["baseline"],
        **{
            specification.mode: MODE_LABELS[specification.mode]
            for specification in specifications
        },
    }
    two_row_legend = len(mode_labels) == len(MODES)
    style.finish_all_modes_six_panel_figure(
        figure,
        axes,
        mode_labels=mode_labels,
        legend_columns=3 if two_row_legend else None,
        legend_handle_order=(
            TWO_ROW_LEGEND_HANDLE_ORDER if two_row_legend else None
        ),
        legend_y=0.99 if two_row_legend else 0.97,
        layout_top=(
            0.89
            if stage in ("static", "6d_pick_place") and two_row_legend
            else (0.91 if two_row_legend else 0.89)
        ),
    )
    return figure


def actuator_effort(run):
    """Return sqrt(tau tau^T), the combined norm of all 14 actuator torques."""
    torque = run.loc[:, TORQUE_COLUMNS].to_numpy(dtype=float)
    return np.sqrt(np.sum(torque * torque, axis=1))


def total_actuator_effort(run, *, end_time=None):
    """Integrate the 14-actuator torque norm over a comparison horizon."""
    time = run["time"].to_numpy(dtype=float)
    if np.any(np.diff(time) <= 0.0):
        raise ValueError("Actuator-effort timestamps must be strictly increasing")
    effort = actuator_effort(run)
    if end_time is not None:
        time, effort = extend_final_value(time, effort, end_time)
    return float(np.trapezoid(effort, time))


def plot_actuator_effort(stage_runs, *, stage, style):
    """Plot baseline and all four modes' effort across the six cases."""
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
        mode_labels=ACTUATOR_MODE_LABELS,
        legend_columns=3,
        legend_handle_order=TWO_ROW_LEGEND_HANDLE_ORDER,
        legend_y=0.99,
        layout_top=0.78,
    )
    return figure


def plot_total_actuator_effort(stage_runs, *, style):
    """Plot time-integrated actuator effort for all modes and six cases."""
    figure, axes = plt.subplots(
        2,
        3,
        figsize=style.SIX_PANEL_GRID_SIZE,
        sharex=True,
        sharey=False,
    )
    positions = np.arange(len(MODES))
    colors = (
        style.BASELINE_COLOR,
        *(style.mode_color(mode) for mode in MODES if mode != "baseline"),
    )
    tick_labels = tuple(MODE_ABBREVIATIONS[mode] for mode in MODES)
    legend_handles = tuple(
        Patch(
            facecolor=color,
            edgecolor="white",
            linewidth=0.5,
            label=ACTUATOR_MODE_LABELS[mode],
        )
        for mode, color in zip(MODES, colors)
    )
    legend_handles = tuple(
        legend_handles[index] for index in TWO_ROW_LEGEND_HANDLE_ORDER
    )

    for axis, (case_name, runs) in zip(axes.flat, stage_runs.items()):
        comparison_end_time = max(
            float(run["time"].iloc[-1]) for run in runs.values()
        )
        totals = tuple(
            total_actuator_effort(
                runs[mode],
                end_time=comparison_end_time,
            )
            for mode in MODES
        )
        axis.bar(
            positions,
            totals,
            color=colors,
            width=0.72,
            edgecolor="white",
            linewidth=0.5,
        )
        axis.set_title(subplot_title(case_name))
        axis.set_xlabel("")
        axis.set_ylabel("")
        axis.set_xticks(positions, tick_labels, rotation=25, ha="right")
        axis.set_ylim(bottom=0.0)

    sns.despine(fig=figure, offset=2, trim=False)
    figure.tight_layout(
        rect=(0.035, 0.04, 1.0, 0.78),
        pad=0.35,
        h_pad=0.5,
        w_pad=0.45,
    )
    figure.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.99),
        ncol=3,
        handlelength=1.2,
        frameon=True,
        fancybox=False,
        framealpha=1.0,
        facecolor="white",
        edgecolor="#68737D",
        borderpad=0.4,
    )
    figure.supxlabel("Optimization mode", y=0.005)
    figure.supylabel(
        r"Total actuator effort $\int \|\tau(t)\|_2\,dt$ (N m s)",
        x=0.018,
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
    """Create combined-objective and actuator-effort figures for one stage."""
    stage_runs = load_stage_runs(batch_dir, stage)
    legacy_output_dir = Path(output_root) / "combined_plots"
    output_dir = Path(output_root) / "combined_plotsV2"
    effort_output_dir = Path(output_root) / "actuator_effort"
    legacy_combined_figure = plot_combined_optimization_grid(
        stage_runs,
        stage=stage,
        metric_scale=metric_scale,
        style=style,
        specifications=OPTIMIZATION_PLOTS[:3],
    )
    combined_figure = plot_combined_optimization_grid(
        stage_runs,
        stage=stage,
        metric_scale=metric_scale,
        style=style,
    )
    written = list(
        style.save(
            legacy_combined_figure,
            legacy_output_dir,
            f"{stage}_combined_optimization_three_by_six",
            output_format,
        )
    )
    written.extend(style.save(
        combined_figure,
        output_dir,
        f"{stage}_combined_optimization_four_by_six",
        output_format,
    ))
    effort_figure = plot_actuator_effort(
        stage_runs,
        stage=stage,
        style=style,
    )
    written.extend(
        style.save(
            effort_figure,
            effort_output_dir,
            f"{stage}_actuator_effort_six_cases",
            output_format,
        )
    )
    total_effort_figure = plot_total_actuator_effort(
        stage_runs,
        style=style,
    )
    written.extend(
        style.save(
            total_effort_figure,
            effort_output_dir,
            f"{stage}_total_actuator_effort_six_cases",
            output_format,
        )
    )
    return (
        legacy_combined_figure,
        combined_figure,
        effort_figure,
        total_effort_figure,
    ), tuple(written)


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
