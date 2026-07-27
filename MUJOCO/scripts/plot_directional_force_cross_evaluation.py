"""Cross-evaluate direct and indirect directional-force optimizations.

The plotted quantities are recomputed from the recorded arm configurations and
object poses. The direct and indirect metric columns already present in the
CSV files are deliberately not loaded, so the result is an independent
kinematic verification rather than a replot of recorded diagnostics.
"""

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation
import seaborn as sns

from MUJOCO.plotting_scripts.equation8_plot_style import Equation8PlotStyle
from MUJOCO.scripts.plot_main import (
    ARM_BASE_OFFSET,
    ARM_JOINT_COLUMNS,
    DEFAULT_BATCH_DIR,
    OBJECT_POSE_COLUMNS,
    STAGES,
    case_directories,
    discover_stage_run_directories,
    extend_final_value,
    subplot_title,
)
from MUJOCO.utils.cli import run_cli
from MUJOCO.utils.grasping_kinematics import (
    CooperativeManipulationKinematics,
)
from MUJOCO.utils.scene_builder import DualFrankaMuJoCoScene


OPTIMIZATION_MODES = (
    "directional_force",
    "directional_force_indirect",
)
MODES = ("baseline", *OPTIMIZATION_MODES)
MODE_LABELS = {
    "baseline": "Baseline",
    "directional_force": "Direct directional-force optimization",
    "directional_force_indirect": "Indirect directional-force optimization",
}
STAGE_LABELS = {
    "static": "Static",
    "pick_place": "Pick-and-place",
    "6d_pick_place": "6D pick-and-place",
}
REQUIRED_COLUMNS = (
    "time",
    *ARM_JOINT_COLUMNS,
    *OBJECT_POSE_COLUMNS,
    "characteristic_length_m",
)
METRIC_COLUMNS = {
    ("direct", "raw"): "recomputed_direct_raw",
    ("direct", "scaled"): "recomputed_direct_scaled",
    ("indirect", "raw"): "recomputed_indirect_raw",
    ("indirect", "scaled"): "recomputed_indirect_scaled",
}
ROW_SPECIFICATIONS = (
    (
        "direct",
        ("directional_force",),
        "Direct metric on\ndirect optimization ↓",
    ),
    (
        "indirect",
        ("directional_force_indirect",),
        "Indirect metric on\nindirect optimization ↑",
    ),
    (
        "indirect",
        OPTIMIZATION_MODES,
        "Indirect metric on\nboth optimizations ↑",
    ),
    (
        "direct",
        OPTIMIZATION_MODES,
        "Direct metric on\nboth optimizations ↓",
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
        help="comparison_main batch containing the recorded run CSVs",
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
        help="spatial metric scale to recompute (default: %(default)s)",
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
        help="open figures after saving",
    )
    arguments = parser.parse_args(argv)
    if arguments.dpi <= 0:
        parser.error("--dpi must be greater than zero")
    return arguments


def load_state_csv(path):
    """Load only state needed for independent objective reconstruction."""
    path = Path(path)
    frame = pd.read_csv(path, usecols=lambda name: name in REQUIRED_COLUMNS)
    missing = set(REQUIRED_COLUMNS) - set(frame.columns)
    if missing:
        raise ValueError(f"Missing columns {sorted(missing)} in: {path}")
    if frame.empty:
        raise ValueError(f"CSV contains no samples: {path}")
    if not np.all(np.isfinite(frame.to_numpy(dtype=float))):
        raise ValueError(f"CSV contains non-finite state data: {path}")
    return frame


def load_stage_states(batch_dir, stage):
    """Load baseline, direct, and indirect states for all six stage cases."""
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
            loaded[case_name][mode] = load_state_csv(matches[0])
    return loaded


def normalized_frobenius_distance(capability, desired):
    """Return the trace-normalized matrix distance used by both objectives."""
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


def directional_metrics_from_velocity_map(
    velocity_map,
    characteristic_length,
    *,
    pinv_rcond,
):
    """Independently compute direct/indirect raw and scaled objectives."""
    velocity_map = np.asarray(velocity_map, dtype=float)
    characteristic_length = float(characteristic_length)
    if characteristic_length <= 0.0:
        raise ValueError("characteristic_length must be positive")

    scaling = np.diag(
        [1.0, 1.0, 1.0] + [characteristic_length] * 3
    )
    scaled_map = scaling @ velocity_map
    velocity_raw = velocity_map @ velocity_map.T
    velocity_scaled = scaled_map @ scaled_map.T
    force_raw = np.linalg.pinv(velocity_raw, rcond=pinv_rcond)
    force_scaled = np.linalg.pinv(velocity_scaled, rcond=pinv_rcond)

    desired_weights_raw = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    desired_weights_scaled = desired_weights_raw.copy()
    desired_weights_scaled[3:] /= characteristic_length
    desired_raw = np.diag(desired_weights_raw)
    desired_scaled = np.diag(desired_weights_scaled)
    return {
        METRIC_COLUMNS[("direct", "raw")]: normalized_frobenius_distance(
            force_raw,
            desired_raw,
        ),
        METRIC_COLUMNS[("direct", "scaled")]: normalized_frobenius_distance(
            force_scaled,
            desired_scaled,
        ),
        METRIC_COLUMNS[("indirect", "raw")]: normalized_frobenius_distance(
            velocity_raw,
            desired_raw,
        ),
        METRIC_COLUMNS[("indirect", "scaled")]: normalized_frobenius_distance(
            velocity_scaled,
            desired_scaled,
        ),
    }


def make_kinematic_recomputer():
    """Build the same robot geometry used to record the comparison batch."""
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
    return scene, kinematics


def recompute_run_metrics(run, scene, kinematics):
    """Reconstruct both objectives from every recorded configuration."""
    arm_states = run.loc[:, ARM_JOINT_COLUMNS].to_numpy(dtype=float)
    object_poses = run.loc[:, OBJECT_POSE_COLUMNS].to_numpy(dtype=float)
    characteristic_lengths = run["characteristic_length_m"].to_numpy(
        dtype=float
    )
    recomputed = {
        column: np.empty(len(run), dtype=float)
        for column in METRIC_COLUMNS.values()
    }

    for sample, (arm_state, object_pose, characteristic_length) in enumerate(
        zip(arm_states, object_poses, characteristic_lengths)
    ):
        scene.data.qpos[scene.arm_qpos] = arm_state
        quaternion_xyzw = np.array(
            [
                object_pose[4],
                object_pose[5],
                object_pose[6],
                object_pose[3],
            ]
        )
        scene.set_table_reference_pose(
            object_pose[:3],
            Rotation.from_quat(quaternion_xyzw).as_matrix(),
        )
        velocity_map = kinematics.paper_object_velocity_map(scene.data)
        values = directional_metrics_from_velocity_map(
            velocity_map,
            characteristic_length,
            pinv_rcond=kinematics.pinv_rcond,
        )
        for column, value in values.items():
            recomputed[column][sample] = value

    return pd.DataFrame(
        {"time": run["time"].to_numpy(dtype=float), **recomputed}
    )


def recompute_stage_metrics(stage_states):
    """Recompute both metrics for baseline and both optimizers."""
    scene, kinematics = make_kinematic_recomputer()
    evaluated = {}
    for case_name, runs in stage_states.items():
        evaluated[case_name] = {}
        for mode, run in runs.items():
            evaluated[case_name][mode] = recompute_run_metrics(
                run,
                scene,
                kinematics,
            )
    return evaluated


def synchronize_objective_limits(axes):
    """Match scales for rows that evaluate the same objective."""
    for row_pair in ((0, 3), (1, 2)):
        paired_axes = tuple(
            axes[row, column]
            for row in row_pair
            for column in range(6)
        )
        lower = min(axis.get_ylim()[0] for axis in paired_axes)
        upper = max(axis.get_ylim()[1] for axis in paired_axes)
        for axis in paired_axes:
            axis.set_ylim(lower, upper)


def plot_cross_evaluation(evaluated, *, stage, metric_scale, style):
    """Plot four native/crossed objective evaluations across six cases."""
    figure, axes = plt.subplots(
        4,
        6,
        figsize=style.FOUR_BY_SIX_SIZE,
        sharex=False if stage == "static" else "col",
        squeeze=False,
    )
    for row, (objective, plotted_modes, row_label) in enumerate(
        ROW_SPECIFICATIONS
    ):
        metric_column = METRIC_COLUMNS[(objective, metric_scale)]
        for axis, (case_name, modes) in zip(axes[row], evaluated.items()):
            comparison_end_time = max(
                float(modes[mode]["time"].iloc[-1])
                for mode in plotted_modes
            )
            baseline = modes["baseline"]
            baseline_time = baseline["time"].to_numpy(dtype=float)
            baseline_values = (
                baseline[metric_column].to_numpy(dtype=float)
                / np.sqrt(2.0)
            )
            if stage == "static":
                baseline_time, baseline_values = extend_final_value(
                    baseline_time,
                    baseline_values,
                    comparison_end_time,
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
            for mode in plotted_modes:
                run = modes[mode]
                sns.lineplot(
                    x=run["time"],
                    y=run[metric_column] / np.sqrt(2.0),
                    estimator=None,
                    sort=False,
                    legend=False,
                    ax=axis,
                    **style.optimized_line_kwargs(mode),
                )
            axis.set_xlabel("")
            axis.set_ylabel("")
            if row == 0:
                axis.set_title(subplot_title(case_name))
        scale_suffix = "\n(scaled)" if metric_scale == "scaled" else ""
        axes[row, 0].set_ylabel(
            row_label + scale_suffix,
            rotation=90,
            ha="center",
            va="center",
            labelpad=18,
            fontsize=7.5,
        )

    synchronize_objective_limits(axes)
    for axis in axes.flat:
        axis.margins(x=0)
        axis.ticklabel_format(axis="y", style="plain", useOffset=False)
    sns.despine(fig=figure, offset=2, trim=False)
    figure.suptitle(
        f"{STAGE_LABELS[stage]} directional-force objective cross-evaluation",
        y=0.995,
    )
    figure.supxlabel("Simulation time (s)", y=0.005)
    figure.align_ylabels(axes[:, 0])
    handles = (
        Line2D(
            [0],
            [0],
            label=MODE_LABELS["baseline"],
            **style.baseline_line_kwargs(),
        ),
        *(
            Line2D(
                [0],
                [0],
                label=MODE_LABELS[mode],
                **style.optimized_line_kwargs(mode),
            )
            for mode in OPTIMIZATION_MODES
        ),
    )
    figure.tight_layout(
        rect=(0.015, 0.025, 1.0, 0.87),
        pad=0.35,
        h_pad=0.5,
        w_pad=0.45,
    )
    figure.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.955),
        ncol=3,
        handlelength=2.2,
        frameon=True,
        fancybox=False,
        framealpha=1.0,
        facecolor="white",
        edgecolor="#68737D",
        borderpad=0.4,
    )
    return figure


def plot_all_stages_direct_metric(evaluated_stages, *, metric_scale, style):
    """Combine row four from every stage into one three-by-six figure."""
    figure, axes = plt.subplots(
        len(STAGES),
        6,
        figsize=(style.DOUBLE_COLUMN_WIDTH, 4.8),
        sharex=False,
        sharey=True,
        squeeze=False,
    )
    metric_column = METRIC_COLUMNS[("direct", metric_scale)]
    for row, stage in enumerate(STAGES):
        evaluated = evaluated_stages[stage]
        for column, (case_name, modes) in enumerate(evaluated.items()):
            axis = axes[row, column]
            comparison_end_time = max(
                float(modes[mode]["time"].iloc[-1])
                for mode in OPTIMIZATION_MODES
            )
            baseline = modes["baseline"]
            baseline_time = baseline["time"].to_numpy(dtype=float)
            baseline_values = (
                baseline[metric_column].to_numpy(dtype=float)
                / np.sqrt(2.0)
            )
            if stage == "static":
                baseline_time, baseline_values = extend_final_value(
                    baseline_time,
                    baseline_values,
                    comparison_end_time,
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
            for mode in OPTIMIZATION_MODES:
                run = modes[mode]
                sns.lineplot(
                    x=run["time"],
                    y=run[metric_column] / np.sqrt(2.0),
                    estimator=None,
                    sort=False,
                    legend=False,
                    ax=axis,
                    **style.optimized_line_kwargs(mode),
                )
            axis.set_xlabel("")
            axis.set_ylabel("")
            axis.set_title(f"Case {column + 1}" if row == 0 else "")

        scale_suffix = "\n(scaled)" if metric_scale == "scaled" else ""
        axes[row, 0].set_ylabel(
            f"{STAGE_LABELS[stage]}\nDirect metric ↓{scale_suffix}",
            rotation=90,
            ha="center",
            va="center",
            labelpad=18,
            fontsize=7.5,
        )

    for axis in axes.flat:
        axis.margins(x=0)
        axis.ticklabel_format(axis="y", style="plain", useOffset=False)
    sns.despine(fig=figure, offset=2, trim=False)
    figure.suptitle(
        "Direct directional-force metric on baseline and both optimizations",
        y=0.995,
    )
    figure.supxlabel("Simulation time (s)", y=0.005)
    figure.align_ylabels(axes[:, 0])
    handles = (
        Line2D(
            [0],
            [0],
            label=MODE_LABELS["baseline"],
            **style.baseline_line_kwargs(),
        ),
        *(
            Line2D(
                [0],
                [0],
                label=MODE_LABELS[mode],
                **style.optimized_line_kwargs(mode),
            )
            for mode in OPTIMIZATION_MODES
        ),
    )
    figure.tight_layout(
        rect=(0.015, 0.035, 1.0, 0.82),
        pad=0.35,
        h_pad=0.5,
        w_pad=0.45,
    )
    figure.legend(
        handles=handles,
        loc="upper center",
        bbox_to_anchor=(0.5, 0.945),
        ncol=3,
        handlelength=2.2,
        frameon=True,
        fancybox=False,
        framealpha=1.0,
        facecolor="white",
        edgecolor="#68737D",
        borderpad=0.4,
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
    figures = []
    written = []
    evaluated_stages = {}
    for stage in stages:
        print(f"Recomputing directional metrics for stage: {stage}")
        stage_states = load_stage_states(batch_dir, stage)
        evaluated = recompute_stage_metrics(stage_states)
        evaluated_stages[stage] = evaluated
        figure = plot_cross_evaluation(
            evaluated,
            stage=stage,
            metric_scale=arguments.metric_scale,
            style=style,
        )
        figures.append(figure)
        written.extend(
            style.save(
                figure,
                output_dir,
                f"{stage}_directional_force_cross_evaluation_four_by_six",
                arguments.format,
            )
        )

    if stages == STAGES:
        summary_figure = plot_all_stages_direct_metric(
            evaluated_stages,
            metric_scale=arguments.metric_scale,
            style=style,
        )
        figures.append(summary_figure)
        written.extend(
            style.save(
                summary_figure,
                output_dir,
                "all_stages_direct_metric_both_optimizations_three_by_six",
                arguments.format,
            )
        )

    print(f"Saved {len(written)} cross-evaluation figure files to: {output_dir}")
    for path in written:
        print(f"  {path}")
    if arguments.show:
        plt.show()
    else:
        for figure in figures:
            plt.close(figure)
    return tuple(written)


if __name__ == "__main__":
    run_cli(main)
