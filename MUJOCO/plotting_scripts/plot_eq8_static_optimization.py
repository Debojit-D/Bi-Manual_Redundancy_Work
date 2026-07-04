"""Create publication-oriented plots for a static Equation (8) CSV run.

Example::

    # Plot the default recording configured below.
    python -m MUJOCO.plotting_scripts.plot_eq8_static_optimization

    # Or override it with another recording.
    python -m MUJOCO.plotting_scripts.plot_eq8_static_optimization path/to/run.csv

The script treats one CSV as one representative run. It does not add error
bands or make trial-to-trial claims; those require multiple matched runs.
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from MUJOCO.plotting_scripts.publication_style import (
    COLORS,
    PAPER_WIDTH_IN,
    configure_publication_style,
    finish_figure,
)


# Change this value when you want Run/Debug in the IDE to use another CSV.
REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_INPUT_CSV = REPOSITORY_ROOT / (
    "outputs/mujoco_data/"
    "dual_franka_eq8_static_optimization_fitted_spheres_"
    "20260704_155110_197252.csv"
)

OBJECTIVE_LABELS = {
    "velocity": "Velocity manipulability",
    "force": "Force manipulability",
    "directional_force": "Directional-force alignment distance",
}
MODE_LABELS = {
    "baseline": "Baseline (null-space optimization disabled)",
    "velocity": "Velocity-manipulability optimization",
    "force": "Force-manipulability optimization",
    "directional_force": "Directional-force alignment optimization",
}


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input_csv",
        nargs="?",
        type=Path,
        default=DEFAULT_INPUT_CSV,
        help=f"Equation (8) recording (default: {DEFAULT_INPUT_CSV})",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help=(
            "figure directory (default: <CSV directory>/<CSV stem>_figures)"
        ),
    )
    parser.add_argument(
        "--format",
        choices=("pdf", "png", "both"),
        default="both",
        help="output format (default: both)",
    )
    parser.add_argument(
        "--dpi", type=int, default=300, help="PNG resolution (default: 300)"
    )
    parser.add_argument(
        "--safety-margin",
        type=float,
        default=0.05,
        help="collision safety margin in metres (default: 0.05)",
    )
    parser.add_argument(
        "--show", action="store_true", help="show figures after saving"
    )
    return parser.parse_args()


def configure_style():
    configure_publication_style()
    plt.rcParams["figure.figsize"] = (PAPER_WIDTH_IN, 4.3)


def load_recording(path):
    if not path.is_file():
        raise FileNotFoundError(f"Recording not found: {path}")
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"Recording contains no samples: {path}")

    def values(column):
        if column not in rows[0]:
            raise ValueError(f"Required CSV column is missing: {column}")
        try:
            result = np.asarray([float(row[column]) for row in rows])
        except (TypeError, ValueError) as error:
            raise ValueError(f"Column is not numeric: {column}") from error
        if not np.all(np.isfinite(result)):
            raise ValueError(f"Column contains non-finite values: {column}")
        return result

    return rows, values


def save_figure(fig, output_dir, stem, output_format, dpi):
    finish_figure(fig)
    suffixes = ("pdf", "png") if output_format == "both" else (output_format,)
    written = []
    for suffix in suffixes:
        path = output_dir / f"{stem}.{suffix}"
        options = {"dpi": dpi} if suffix == "png" else {}
        fig.savefig(path, **options)
        written.append(path)
    return written


def label_panels(axes):
    for index, axis in enumerate(np.ravel(axes)):
        axis.text(
            0.01,
            0.98,
            f"({chr(ord('a') + index)})",
            transform=axis.transAxes,
            ha="left",
            va="top",
            fontweight="bold",
        )


def objective_series(rows, values):
    """Return the physically meaningful objective curve for this CSV.

    The optimizer now treats every objective as a maximization score.  For the
    directional-force case, the score is ``-D_dir``.  For plotting and paper
    reporting, however, the raw alignment distance ``D_dir`` is clearer, so
    this helper returns that distance and marks it as lower-is-better.
    """
    objective_key = rows[0].get("objective", "unknown")
    if objective_key == "velocity":
        return {
            "key": objective_key,
            "metric": values("velocity_manipulability"),
            "ylabel": "Velocity manipulability index",
            "title": "Velocity-manipulability objective",
            "lower_is_better": False,
        }
    if objective_key == "force":
        return {
            "key": objective_key,
            "metric": values("force_manipulability"),
            "ylabel": "Force manipulability index",
            "title": "Force-manipulability objective",
            "lower_is_better": False,
        }
    if objective_key == "directional_force":
        return {
            "key": objective_key,
            "metric": values("directional_force_cost"),
            "ylabel": "Directional-force alignment distance",
            "title": "Directional-force alignment objective",
            "lower_is_better": True,
        }
    return {
        "key": objective_key,
        "metric": values("objective_value"),
        "ylabel": "Objective value",
        "title": "Objective evolution",
        "lower_is_better": False,
    }


def improvement_percent(metric, lower_is_better):
    initial = metric[0]
    if np.isclose(initial, 0.0):
        return np.zeros_like(metric)
    if lower_is_better:
        return 100.0 * (1.0 - metric / initial)
    return 100.0 * (metric / initial - 1.0)


def plot_objective(time, rows, values):
    info = objective_series(rows, values)
    metric = info["metric"]
    relative = improvement_percent(metric, info["lower_is_better"])
    direction_note = "lower is better" if info["lower_is_better"] else "higher is better"

    fig, axes = plt.subplots(1, 2, figsize=(PAPER_WIDTH_IN, 2.75))

    axes[0].plot(time, metric, color=COLORS[0])
    axes[0].scatter(time[[0, -1]], metric[[0, -1]], color=COLORS[1], zorder=3)
    axes[0].set(xlabel="Time (s)", ylabel=info["ylabel"])
    axes[0].set_title(f"{info['title']} ({direction_note})")

    axes[1].plot(time, relative, color=COLORS[2])
    axes[1].axhline(0.0, color="0.35", linewidth=0.8)
    axes[1].set(xlabel="Time (s)", ylabel="Improvement from initial value (%)")
    axes[1].set_title(f"Net improvement: {relative[-1]:.1f}%")
    label_panels(axes)
    fig.tight_layout()
    return fig


def plot_pose_tracking(time, values):
    object_position = np.column_stack(
        [values("object_x"), values("object_y"), values("object_z")]
    )
    displacement_mm = 1000.0 * (object_position - object_position[0])
    position_error_mm = 1000.0 * np.column_stack(
        [
            values("position_error_x"),
            values("position_error_y"),
            values("position_error_z"),
        ]
    )
    orientation_error_deg = np.rad2deg(
        np.column_stack(
            [
                values("orientation_error_x"),
                values("orientation_error_y"),
                values("orientation_error_z"),
            ]
        )
    )

    fig, axes = plt.subplots(2, 2, figsize=(PAPER_WIDTH_IN, 5.1), sharex=True)
    components = ("x", "y", "z")
    for index, component in enumerate(components):
        axes[0, 0].plot(
            time, displacement_mm[:, index], color=COLORS[index], label=component
        )
        axes[0, 1].plot(
            time, position_error_mm[:, index], color=COLORS[index], label=component
        )
        axes[1, 0].plot(
            time,
            orientation_error_deg[:, index],
            color=COLORS[index],
            label=component,
        )

    axes[0, 0].set(ylabel="Displacement from initial pose (mm)")
    axes[0, 0].set_title("Measured object translation")
    axes[0, 1].plot(
        time,
        1000.0 * values("position_error_norm"),
        color="#222222",
        linestyle="--",
        label="norm",
    )
    axes[0, 1].set(ylabel="Position error (mm)")
    axes[0, 1].set_title("Position tracking error")
    axes[1, 0].plot(
        time,
        np.rad2deg(values("orientation_error_norm")),
        color="#222222",
        linestyle="--",
        label="norm",
    )
    axes[1, 0].set(xlabel="Time (s)", ylabel="Orientation error (deg)")
    axes[1, 0].set_title("Orientation tracking error")
    axes[1, 1].plot(
        time, values("grasp_error_norm"), color=COLORS[3], label="grasp error"
    )
    axes[1, 1].set(xlabel="Time (s)", ylabel="Grasp-pose error norm")
    axes[1, 1].set_title("Closed-chain grasp consistency")
    for axis in np.ravel(axes):
        axis.legend(ncol=4, loc="best")
    label_panels(axes)
    fig.tight_layout()
    return fig


def plot_controller(time, values):
    fig, axes = plt.subplots(2, 2, figsize=(PAPER_WIDTH_IN, 5.1), sharex=True)
    axes[0, 0].plot(
        time, values("primary_speed_max"), color=COLORS[0], label="primary"
    )
    axes[0, 0].plot(
        time, values("null_speed_max"), color=COLORS[1], label="null space"
    )
    axes[0, 0].plot(
        time, values("command_speed_max"), color=COLORS[2], label="commanded"
    )
    axes[0, 0].set(ylabel="Maximum joint speed (rad/s)")
    axes[0, 0].set_title("Equation (8) velocity terms")

    axes[0, 1].plot(
        time, values("phi_dot_opt_max"), color=COLORS[1], label=r"$\dot{\phi}_{opt}$"
    )
    axes[0, 1].set(ylabel="Maximum requested speed (rad/s)")
    axes[0, 1].set_title("Raw optimization request")
    gradient_axis = axes[0, 1].twinx()
    gradient_axis.grid(False)
    gradient_axis.plot(
        time,
        values("gradient_norm"),
        color=COLORS[3],
        linestyle="--",
        label="gradient norm",
    )
    gradient_axis.set_ylabel("Gradient norm")

    axes[1, 0].plot(
        time, values("null_space_scale"), color=COLORS[2], label="scale"
    )
    axes[1, 0].set(xlabel="Time (s)", ylabel="Scale factor")
    axes[1, 0].set_ylim(-0.05, 1.05)
    axes[1, 0].set_title("Joint-limit null-space scaling")

    axes[1, 1].semilogy(
        time,
        np.maximum(values("unscaled_null_space_leakage"), np.finfo(float).tiny),
        color=COLORS[1],
        label="unscaled",
    )
    axes[1, 1].semilogy(
        time,
        np.maximum(values("scaled_null_space_leakage"), np.finfo(float).tiny),
        color=COLORS[0],
        linestyle="--",
        label="scaled",
    )
    axes[1, 1].set(xlabel="Time (s)", ylabel="Leakage norm")
    axes[1, 1].set_title("Primary-task null-space leakage")
    for axis in np.ravel(axes):
        axis.legend(loc="best")
    handles, labels = gradient_axis.get_legend_handles_labels()
    gradient_axis.legend(handles, labels, loc="lower right")
    label_panels(axes)
    fig.tight_layout()
    return fig


def plot_safety(time, values, safety_margin):
    clearance = values("min_inter_arm_clearance")
    joint_margin = values("min_joint_limit_distance")
    collision_cost = values("collision_cost")
    fig, axes = plt.subplots(1, 3, figsize=(PAPER_WIDTH_IN, 2.8))

    axes[0].plot(time, 100.0 * clearance, color=COLORS[0])
    axes[0].axhline(
        100.0 * safety_margin,
        color=COLORS[1],
        linestyle="--",
        label=f"soft target {100.0 * safety_margin:.1f} cm",
    )
    axes[0].set(xlabel="Time (s)", ylabel="Minimum clearance (cm)")
    axes[0].set_title("Inter-arm clearance")
    axes[0].legend(loc="best")

    axes[1].semilogy(
        time,
        np.maximum(collision_cost, np.finfo(float).tiny),
        color=COLORS[1],
    )
    axes[1].set(xlabel="Time (s)", ylabel="Collision penalty")
    axes[1].set_title("Soft collision cost")

    axes[2].plot(time, joint_margin, color=COLORS[2])
    axes[2].set(xlabel="Time (s)", ylabel="Minimum limit distance (rad)")
    axes[2].set_title("Joint-limit margin")
    label_panels(axes)
    fig.tight_layout()
    return fig


def plot_joint_posture(time, values):
    fig, axes = plt.subplots(2, 1, figsize=(PAPER_WIDTH_IN, 5.2), sharex=True)
    for joint in range(1, 8):
        axes[0].plot(
            time,
            values(f"q_l{joint}"),
            color=COLORS[joint - 1],
            label=rf"$q_{{L{joint}}}$",
        )
        axes[1].plot(
            time,
            values(f"q_r{joint}"),
            color=COLORS[joint - 1],
            label=rf"$q_{{R{joint}}}$",
        )
    axes[0].set(ylabel="Joint angle (rad)")
    axes[0].set_title("Left-arm posture evolution")
    axes[1].set(xlabel="Time (s)", ylabel="Joint angle (rad)")
    axes[1].set_title("Right-arm posture evolution")
    for axis in axes:
        axis.legend(ncol=7, loc="best", columnspacing=0.8, handlelength=1.4)
    label_panels(axes)
    fig.tight_layout()
    return fig


def plot_actuator_effort(time, values):
    left_torque = np.column_stack([values(f"tau_act_l{i}") for i in range(1, 8)])
    right_torque = np.column_stack([values(f"tau_act_r{i}") for i in range(1, 8)])
    left_norm = np.linalg.norm(left_torque, axis=1)
    right_norm = np.linalg.norm(right_torque, axis=1)
    rms = np.sqrt(np.mean(np.column_stack([left_torque, right_torque]) ** 2, axis=0))

    fig, axes = plt.subplots(1, 3, figsize=(PAPER_WIDTH_IN, 2.9))
    axes[0].plot(time, values("tau_actuator_norm"), color=COLORS[0], label="actuator")
    axes[0].plot(
        time, values("tau_total_est_norm"), color=COLORS[1], label="estimated total"
    )
    axes[0].set(xlabel="Time (s)", ylabel="Torque-vector norm (N m)")
    axes[0].set_title("System effort")
    axes[0].legend(loc="best")

    axes[1].plot(time, left_norm, color=COLORS[0], label="left")
    axes[1].plot(time, right_norm, color=COLORS[1], label="right")
    axes[1].set(xlabel="Time (s)", ylabel="Actuator torque norm (N m)")
    axes[1].set_title("Effort distribution")
    axes[1].legend(loc="best")

    x = np.arange(14)
    axes[2].bar(x[:7], rms[:7], color=COLORS[0], label="left")
    axes[2].bar(x[7:], rms[7:], color=COLORS[1], label="right")
    axes[2].set_xticks(x, [str(i) for i in range(1, 8)] * 2, fontsize=7)
    axes[2].axvline(6.5, color="0.4", linewidth=0.8)
    axes[2].set(xlabel="Joint number (left | right)", ylabel="RMS torque (N m)")
    axes[2].set_title("Per-joint RMS effort")
    axes[2].legend(loc="best")
    label_panels(axes)
    fig.tight_layout()
    return fig


def write_summary(path, rows, values, safety_margin):
    time = values("time")
    objective_info = objective_series(rows, values)
    objective = objective_info["metric"]
    objective_change = improvement_percent(
        objective,
        objective_info["lower_is_better"],
    )
    position_mm = 1000.0 * values("position_error_norm")
    orientation_deg = np.rad2deg(values("orientation_error_norm"))
    clearance_cm = 100.0 * values("min_inter_arm_clearance")
    duration = time[-1] - time[0]
    frequency = (len(time) - 1) / duration if duration > 0.0 else float("nan")
    below_margin = np.mean(clearance_cm < 100.0 * safety_margin) * 100.0
    objective_key = rows[0].get("objective", "unknown")
    mode_key = rows[0].get("optimization_mode") or objective_key
    direction_note = (
        "lower is better"
        if objective_info["lower_is_better"]
        else "higher is better"
    )
    lines = [
        "Static Equation (8) representative-run summary",
        "================================================",
        f"Optimization mode: {MODE_LABELS.get(mode_key, mode_key)}",
        f"Monitored objective: {OBJECTIVE_LABELS.get(objective_key, objective_key)}",
        f"Objective direction: {direction_note}",
        f"Mode identifier: {mode_key}",
        f"Collision model: {rows[0].get('collision_version', 'unknown')}",
        f"Samples: {len(time)}",
        f"Duration: {duration:.3f} s",
        f"Nominal sample rate: {frequency:.3f} Hz",
        "",
        f"Objective metric: {objective[0]:.6g} -> {objective[-1]:.6g}",
        f"Net objective improvement: {objective_change[-1]:.3f} %",
        f"Maximum position error: {np.max(position_mm):.3f} mm",
        f"RMS position error: {np.sqrt(np.mean(position_mm**2)):.3f} mm",
        f"Maximum orientation error: {np.max(orientation_deg):.4f} deg",
        f"RMS orientation error: {np.sqrt(np.mean(orientation_deg**2)):.4f} deg",
        f"Maximum grasp-error norm: {np.max(values('grasp_error_norm')):.6g}",
        f"Minimum inter-arm clearance: {np.min(clearance_cm):.3f} cm",
        f"Samples below {100.0 * safety_margin:.1f} cm soft target: {below_margin:.2f} %",
        f"Minimum joint-limit distance: {np.min(values('min_joint_limit_distance')):.4f} rad",
        f"Maximum commanded joint speed: {np.max(values('command_speed_max')):.4f} rad/s",
        f"Maximum actuator torque norm: {np.max(values('tau_actuator_norm')):.4f} N m",
        f"RMS actuator torque norm: {np.sqrt(np.mean(values('tau_actuator_norm')**2)):.4f} N m",
        "",
        "Interpretation note: these values describe one representative run.",
        "Matched repeated and baseline trials are required for variability and comparative claims.",
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main():
    arguments = parse_arguments()
    input_csv = arguments.input_csv.expanduser().resolve()
    output_dir = (
        arguments.output_dir.expanduser().resolve()
        if arguments.output_dir is not None
        else input_csv.parent / f"{input_csv.stem}_figures"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    rows, values = load_recording(input_csv)
    time = values("time")
    configure_style()

    figures = [
        ("01_objective_improvement", plot_objective(time, rows, values)),
        ("02_pose_tracking", plot_pose_tracking(time, values)),
        ("03_controller_behavior", plot_controller(time, values)),
        ("04_safety_metrics", plot_safety(time, values, arguments.safety_margin)),
        ("05_joint_posture", plot_joint_posture(time, values)),
        ("06_actuator_effort", plot_actuator_effort(time, values)),
    ]
    written = []
    for stem, figure in figures:
        written.extend(
            save_figure(
                figure, output_dir, stem, arguments.format, arguments.dpi
            )
        )
    summary_path = output_dir / "run_summary.txt"
    write_summary(summary_path, rows, values, arguments.safety_margin)

    print(f"Loaded {len(rows)} samples from: {input_csv}")
    print(f"Saved {len(written)} figures and summary to: {output_dir}")
    if arguments.show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()
