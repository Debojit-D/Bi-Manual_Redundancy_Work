"""Plot static four-mode manipulability metrics and actuator effort.

Without positional arguments, the newest baseline, velocity, force, and
directional-force CSVs are discovered in ``outputs/mujoco_data``.
"""

import argparse
import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from MUJOCO.plotting_scripts.publication_style import (
    COLORS,
    configure_publication_style,
    finish_figure,
)


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA_DIR = REPOSITORY_ROOT / "outputs" / "mujoco_data"
DEFAULT_OUTPUT_DIR = DEFAULT_DATA_DIR / "static_comparison_figures"

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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv_files",
        nargs="*",
        type=Path,
        help="four comparison CSVs; omit to discover the newest set",
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
        default=DEFAULT_OUTPUT_DIR,
        help="figure directory (default: %(default)s)",
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


def configure_style():
    configure_publication_style()


def discover_files(data_dir):
    files = []
    for mode in MODE_ORDER:
        matches = sorted(
            data_dir.glob(
                f"dual_franka_eq8_static_{mode}_fitted_spheres_*.csv"
            )
        )
        if not matches:
            raise FileNotFoundError(
                f"No {mode!r} comparison CSV found in: {data_dir}"
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
            raise KeyError(column)
        result = np.asarray([float(row[column]) for row in rows])
        if not np.all(np.isfinite(result)):
            raise ValueError(f"Non-finite values in {column!r}: {path}")
        return result

    return {"path": path, "mode": mode, "rows": rows, "values": values}


def load_runs(paths):
    runs = {}
    for path in paths:
        run = load_csv(path.expanduser().resolve())
        if run["mode"] in runs:
            raise ValueError(f"Duplicate mode supplied: {run['mode']}")
        runs[run["mode"]] = run
    missing = set(MODE_ORDER) - set(runs)
    if missing:
        raise ValueError(f"Missing comparison modes: {sorted(missing)}")
    return runs


def selected_metric(run, metric):
    """Use the pure metric when available, otherwise support legacy CSVs."""
    if metric in run["rows"][0]:
        return run["values"](metric)
    return run["values"]("objective_value")


def plot_line(axis, x, y, *, color, label=None, **kwargs):
    axis.plot(x, y, color=color, label=label, **kwargs)


def plot_single_metric(run, metric, ylabel, title, direction):
    fig, axis = plt.subplots(figsize=(7.1, 3.4))
    plot_line(
        axis,
        run["values"]("time"),
        selected_metric(run, metric),
        color=MODE_COLORS[run["mode"]],
    )
    axis.set(xlabel="Time (s)", ylabel=ylabel, title=f"{title} ({direction})")
    axis.ticklabel_format(axis="y", style="plain", useOffset=False)
    fig.tight_layout()
    return fig


def normalize_progress(values, lower_is_better=False):
    """Normalize progress from the initial value.

    Positive values always mean improvement.  This avoids the earlier
    directional-force ambiguity where a decreasing distance was plotted as if
    it were an increasing manipulability index.
    """
    start = values[0]
    progress = start - values if lower_is_better else values - start
    scale = np.max(np.abs(progress))
    if np.isclose(scale, 0.0):
        return np.zeros_like(values)
    return progress / scale


def plot_normalized_progress(runs):
    fig, axis = plt.subplots(figsize=(7.1, 3.4))
    selections = (
        ("velocity", "velocity_manipulability", False),
        ("force", "force_manipulability", False),
        ("directional_force", "directional_force_cost", True),
    )
    for mode, metric, lower_is_better in selections:
        run = runs[mode]
        progress = normalize_progress(
            selected_metric(run, metric),
            lower_is_better=lower_is_better,
        )
        plot_line(
            axis,
            run["values"]("time"),
            progress,
            color=MODE_COLORS[mode],
            label=MODE_LABELS[mode],
        )
    axis.set(
        xlabel="Time (s)",
        ylabel="Normalized optimization progress",
        title="Normalized task-objective progress",
    )
    axis.axhline(0.0, color="0.35", linewidth=0.8)
    axis.legend(loc="best")
    return fig


def actuator_effort(run):
    values = run["values"]
    torque = np.column_stack(
        [values(f"tau_act_l{joint}") for joint in range(1, 8)]
        + [values(f"tau_act_r{joint}") for joint in range(1, 8)]
    )
    return np.sqrt(np.sum(torque * torque, axis=1))


def plot_effort(runs):
    fig, axis = plt.subplots(figsize=(7.1, 3.4))
    for mode in MODE_ORDER:
        run = runs[mode]
        plot_line(
            axis,
            run["values"]("time"),
            actuator_effort(run),
            color=MODE_COLORS[mode],
            label=MODE_LABELS[mode],
        )
    axis.set(
        xlabel="Time (s)",
        ylabel=r"Actuator effort $\sqrt{\tau\tau^\mathsf{T}}=\|\tau\|_2$ (N m)",
        title="Actuator torque effort",
    )
    axis.legend(loc="best", ncol=2)
    fig.tight_layout()
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
    paths = (
        arguments.csv_files
        if arguments.csv_files
        else discover_files(arguments.data_dir.expanduser().resolve())
    )
    runs = load_runs(paths)
    output_dir = arguments.output_dir.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()

    velocity_figure = plot_single_metric(
        runs["velocity"],
        "velocity_manipulability",
        "Velocity manipulability index",
        "Velocity-manipulability optimization",
        "higher is better",
    )
    force_figure = plot_single_metric(
        runs["force"],
        "force_manipulability",
        "Force manipulability index",
        "Force-manipulability optimization",
        "higher is better",
    )
    directional_figure = plot_single_metric(
        runs["directional_force"],
        "directional_force_cost",
        "Directional-force alignment distance",
        "Directional-force optimization",
        "lower is better",
    )
    normalized_figure = plot_normalized_progress(runs)
    effort_figure = plot_effort(runs)

    written = []
    figures = (
        ("01_velocity_manipulability", velocity_figure),
        ("02_force_manipulability", force_figure),
        ("03_directional_force_alignment_distance", directional_figure),
        ("04_normalized_task_objective_progress", normalized_figure),
        ("05_actuator_effort", effort_figure),
    )
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
    print(f"Saved {len(written)} files to: {output_dir}")
    if arguments.show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()
