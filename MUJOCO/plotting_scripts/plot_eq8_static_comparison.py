"""Plot static four-mode manipulability metrics and actuator effort.

Without positional arguments, the newest baseline, velocity, force, and
directional-force CSVs are discovered in ``outputs/mujoco_data``.
"""

import argparse
import csv
import os
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

try:
    from MUJOCO.plotting_scripts.publication_style import (
        COLORS,
        configure_publication_style,
        finish_figure,
    )
except ModuleNotFoundError as error:
    # Support direct execution by absolute path from outside the repository.
    if error.name != "MUJOCO":
        raise
    from publication_style import (  # type: ignore[no-redef]
        COLORS,
        configure_publication_style,
        finish_figure,
    )


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR_ENVIRONMENT_VARIABLE = "BIMANUAL_MUJOCO_DATA_DIR"


def default_data_dir():
    """Return a portable default, optionally overridden by the environment."""
    configured = os.environ.get(DATA_DIR_ENVIRONMENT_VARIABLE)
    if configured:
        return Path(configured).expanduser()
    return REPOSITORY_ROOT / "outputs" / "mujoco_data"


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
        default=default_data_dir(),
        help=(
            "automatic-discovery directory; defaults to the repository's "
            "outputs/mujoco_data directory, or $"
            f"{DATA_DIR_ENVIRONMENT_VARIABLE} when set"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help=(
            "figure directory "
            "(default: <data directory>/static_comparison_figures)"
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
    parser.add_argument("--show", action="store_true", help="show after saving")
    return parser.parse_args()


def configure_style():
    configure_publication_style()


def discover_files(data_dir):
    files = []
    missing = []
    for mode in MODE_ORDER:
        matches = sorted(
            data_dir.glob(
                f"dual_franka_eq8_static_{mode}_fitted_spheres_*.csv"
            )
        )
        if not matches:
            missing.append(mode)
        else:
            files.append(matches[-1])
    if missing:
        patterns = ", ".join(f"{mode!r}" for mode in missing)
        raise FileNotFoundError(
            "Static comparison data is incomplete.\n"
            f"Data directory: {data_dir}\n"
            f"Missing modes: {patterns}\n"
            "Generate them with:\n"
            "  .venv/bin/python -m "
            "MUJOCO.scripts.dual_franka_eq8_static_comparison --record-data\n"
            "Alternatively, pass four CSV paths explicitly or select another "
            f"directory with --data-dir or ${DATA_DIR_ENVIRONMENT_VARIABLE}."
        )
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


def plot_single_metric(run, metric, ylabel, title, direction):
    fig, axis = plt.subplots(figsize=(7.1, 3.4))
    sns.lineplot(
        x=run["values"]("time"),
        y=selected_metric(run, metric),
        ax=axis,
        color=MODE_COLORS[run["mode"]],
        errorbar=None,
    )
    axis.set(xlabel="Time (s)", ylabel=ylabel, title=f"{title} ({direction})")
    axis.ticklabel_format(axis="y", style="plain", useOffset=False)
    return fig


def normalize_progress(values, lower_is_better=False):
    """Min-max normalize achieved optimization progress to [0, 1]."""
    minimum = np.min(values)
    maximum = np.max(values)
    span = maximum - minimum
    if np.isclose(span, 0.0):
        return np.zeros_like(values)
    normalized = (values - minimum) / span
    return 1.0 - normalized if lower_is_better else normalized


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
        sns.lineplot(
            x=run["values"]("time"),
            y=progress,
            ax=axis,
            color=MODE_COLORS[mode],
            label=MODE_LABELS[mode],
            errorbar=None,
        )
    axis.set(
        xlabel="Time (s)",
        ylabel="Normalized optimization progress",
        title="Normalized manipulability optimization",
        ylim=(-0.02, 1.02),
    )
    axis.set_yticks(np.linspace(0.0, 1.0, 6))
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
    data_dir = arguments.data_dir.expanduser().resolve()
    paths = (
        arguments.csv_files
        if arguments.csv_files
        else discover_files(data_dir)
    )
    runs = load_runs(paths)
    output_dir = (
        arguments.output_dir.expanduser().resolve()
        if arguments.output_dir is not None
        else data_dir / "static_comparison_figures"
    )
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
        "Directional-force cost",
        "Directional-force optimization",
        "lower is better",
    )
    normalized_figure = plot_normalized_progress(runs)
    effort_figure = plot_effort(runs)

    written = []
    figures = (
        ("01_velocity_manipulability", velocity_figure),
        ("02_force_manipulability", force_figure),
        ("03_directional_force_cost", directional_figure),
        ("04_normalized_manipulability", normalized_figure),
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
    try:
        main()
    except FileNotFoundError as error:
        raise SystemExit(str(error)) from error
