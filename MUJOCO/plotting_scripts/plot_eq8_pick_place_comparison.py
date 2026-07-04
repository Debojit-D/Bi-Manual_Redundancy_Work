"""Plot the four-mode Equation (8) pick-and-place comparison.

Without positional arguments, the newest baseline, velocity, force, and
directional-force pick-and-place CSVs are discovered automatically::

    .venv/bin/python -m \
        MUJOCO.plotting_scripts.plot_eq8_pick_place_comparison

Use ``--experiment 6d`` for the spatial pick-and-place CSVs, or pass four CSV
paths explicitly. Figures are written as both PDF and PNG by default::

    .venv/bin/python -m \
        MUJOCO.plotting_scripts.plot_eq8_pick_place_comparison \
        --experiment 6d --format png --show
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

from MUJOCO.plotting_scripts.publication_style import (
    COLORS,
    configure_publication_style,
    finish_figure,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA_DIR = REPOSITORY_ROOT / "outputs" / "mujoco_data"

MODE_ORDER = ("baseline", "velocity", "force", "directional_force")
MODE_LABELS = {
    "baseline": "Baseline",
    "velocity": "Velocity optimization",
    "force": "Force optimization",
    "directional_force": "Directional-force optimization",
}
MODE_COLORS = {
    "baseline": "#4D4D4D",
    "velocity": COLORS[0],
    "force": COLORS[1],
    "directional_force": COLORS[2],
}


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "csv_files",
        nargs="*",
        type=Path,
        help="four comparison CSVs; omit to discover the newest set",
    )
    parser.add_argument(
        "--experiment",
        choices=("simple", "6d"),
        default="simple",
        help="CSV family to discover automatically (default: simple)",
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=DEFAULT_DATA_DIR,
        help="automatic-discovery directory (default: %(default)s)",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="figure directory; defaults beside the selected CSV family",
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
    parser.add_argument("--show", action="store_true", help="show after saving")
    return parser.parse_args()


def experiment_prefix(experiment):
    return (
        "dual_franka_eq8_pick_place"
        if experiment == "simple"
        else "dual_franka_eq8_6d_pick_place"
    )


def discover_files(data_dir, experiment):
    prefix = experiment_prefix(experiment)
    files = []
    for mode in MODE_ORDER:
        matches = sorted(
            data_dir.glob(f"{prefix}_{mode}_fitted_spheres_*.csv")
        )
        if not matches:
            raise FileNotFoundError(
                f"No {mode!r} {experiment} pick-and-place CSV in: {data_dir}"
            )
        files.append(matches[-1])
    return files


def load_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"CSV contains no samples: {path}")
    mode = rows[0].get("optimization_mode") or rows[0].get("objective")
    if mode not in MODE_ORDER:
        raise ValueError(f"Unknown optimization mode {mode!r} in: {path}")

    def values(column):
        if column not in rows[0]:
            raise KeyError(f"Missing column {column!r} in: {path}")
        result = np.asarray([float(row[column]) for row in rows])
        if not np.all(np.isfinite(result)):
            raise ValueError(f"Non-finite values in {column!r}: {path}")
        return result

    return {"path": path, "mode": mode, "rows": rows, "values": values}


def load_runs(paths):
    if len(paths) != len(MODE_ORDER):
        raise ValueError("Supply exactly four CSVs, or omit all CSV paths")
    runs = {}
    for path in paths:
        run = load_csv(path.expanduser().resolve())
        mode = run["mode"]
        if mode in runs:
            raise ValueError(f"Duplicate optimization mode supplied: {mode}")
        runs[mode] = run
    missing = set(MODE_ORDER) - set(runs)
    if missing:
        raise ValueError(f"Missing comparison modes: {sorted(missing)}")
    return runs


def plot_metric(runs, column, ylabel, title, log_scale=False):
    fig, axis = plt.subplots(figsize=(7.1, 3.5))
    for mode in MODE_ORDER:
        run = runs[mode]
        sns.lineplot(
            x=run["values"]("time"),
            y=run["values"](column),
            ax=axis,
            color=MODE_COLORS[mode],
            label=MODE_LABELS[mode],
            errorbar=None,
        )
    axis.set(xlabel="Time (s)", ylabel=ylabel, title=title)
    if log_scale:
        axis.set_yscale("log")
    axis.legend(loc="best", ncol=2)
    return fig


def plot_tracking_errors(runs):
    fig, axes = plt.subplots(3, 1, figsize=(7.1, 7.2), sharex=True)
    specifications = (
        ("position_error_norm", "Position error (m)"),
        ("orientation_error_norm", "Orientation error (rad)"),
        ("grasp_error_norm", "Grasp error norm"),
    )
    for mode in MODE_ORDER:
        run = runs[mode]
        time = run["values"]("time")
        for axis, (column, ylabel) in zip(axes, specifications):
            sns.lineplot(
                x=time,
                y=run["values"](column),
                ax=axis,
                color=MODE_COLORS[mode],
                label=MODE_LABELS[mode],
                errorbar=None,
            )
            axis.set_ylabel(ylabel)
    axes[0].set_title("Object tracking and cooperative-grasp errors")
    axes[-1].set_xlabel("Time (s)")
    for axis in axes:
        legend = axis.get_legend()
        if legend is not None:
            legend.remove()
    axes[0].legend(loc="best", ncol=2)
    return fig


def plot_object_path(runs):
    fig = plt.figure(figsize=(7.1, 5.2))
    axis = fig.add_subplot(111, projection="3d")
    for mode in MODE_ORDER:
        run = runs[mode]
        x = run["values"]("object_x")
        y = run["values"]("object_y")
        z = run["values"]("object_z")
        axis.plot(x, y, z, color=MODE_COLORS[mode], label=MODE_LABELS[mode])
        axis.scatter(x[0], y[0], z[0], color=MODE_COLORS[mode], marker="o", s=18)
        axis.scatter(x[-1], y[-1], z[-1], color=MODE_COLORS[mode], marker="x", s=28)
    axis.set(
        xlabel="World x (m)",
        ylabel="World y (m)",
        zlabel="World z (m)",
        title="Measured object path (circle: start, cross: finish)",
    )
    axis.legend(loc="best")
    return fig


def actuator_effort(run):
    values = run["values"]
    torque = np.column_stack(
        [values(f"tau_act_l{joint}") for joint in range(1, 8)]
        + [values(f"tau_act_r{joint}") for joint in range(1, 8)]
    )
    return np.linalg.norm(torque, axis=1)


def plot_actuator_effort(runs):
    fig, axis = plt.subplots(figsize=(7.1, 3.5))
    for mode in MODE_ORDER:
        run = runs[mode]
        sns.lineplot(
            x=run["values"]("time"),
            y=actuator_effort(run),
            ax=axis,
            color=MODE_COLORS[mode],
            label=MODE_LABELS[mode],
            errorbar=None,
        )
    axis.set(
        xlabel="Time (s)",
        ylabel=r"Actuator effort $\|\tau\|_2$ (N m)",
        title="Combined actuator torque effort",
    )
    axis.legend(loc="best", ncol=2)
    return fig


def save_figure(fig, output_dir, stem, output_format, dpi):
    finish_figure(fig)
    suffixes = ("pdf", "png") if output_format == "both" else (output_format,)
    paths = []
    for suffix in suffixes:
        path = output_dir / f"{stem}.{suffix}"
        fig.savefig(path, dpi=dpi if suffix == "png" else None)
        paths.append(path)
    return paths


def main():
    arguments = parse_arguments()
    data_dir = arguments.data_dir.expanduser().resolve()
    paths = (
        arguments.csv_files
        if arguments.csv_files
        else discover_files(data_dir, arguments.experiment)
    )
    runs = load_runs(paths)
    output_dir = (
        arguments.output_dir.expanduser().resolve()
        if arguments.output_dir is not None
        else data_dir / f"{arguments.experiment}_pick_place_comparison_figures"
    )
    output_dir.mkdir(parents=True, exist_ok=True)
    configure_publication_style()

    figures = (
        ("01_object_path_3d", plot_object_path(runs)),
        (
            "02_velocity_manipulability",
            plot_metric(
                runs,
                "velocity_manipulability",
                "Velocity manipulability index",
                "Velocity manipulability (higher is better)",
            ),
        ),
        (
            "03_force_manipulability",
            plot_metric(
                runs,
                "force_manipulability",
                "Force manipulability index",
                "Force manipulability (higher is better)",
            ),
        ),
        (
            "04_directional_force_cost",
            plot_metric(
                runs,
                "directional_force_cost",
                "Directional-force cost",
                "Directional-force cost (lower is better)",
            ),
        ),
        ("05_tracking_errors", plot_tracking_errors(runs)),
        (
            "06_null_space_speed",
            plot_metric(
                runs,
                "null_speed_max",
                "Maximum joint speed (rad/s)",
                "Projected null-space joint speed",
            ),
        ),
        ("07_actuator_effort", plot_actuator_effort(runs)),
        (
            "08_minimum_clearance",
            plot_metric(
                runs,
                "min_inter_arm_clearance",
                "Minimum sphere-surface clearance (m)",
                "Minimum inter-arm clearance",
            ),
        ),
        (
            "09_joint_limit_margin",
            plot_metric(
                runs,
                "min_joint_limit_distance",
                "Minimum joint-limit distance (rad)",
                "Closest joint-limit margin",
            ),
        ),
    )

    written = []
    for stem, figure in figures:
        written.extend(
            save_figure(
                figure,
                output_dir,
                stem,
                arguments.format,
                arguments.dpi,
            )
        )

    print("Input CSVs:")
    for mode in MODE_ORDER:
        print(f"  {MODE_LABELS[mode]}: {runs[mode]['path']}")
    print(f"Saved {len(written)} figure files to: {output_dir}")

    if arguments.show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()
