"""Plot the four static directional-distance permutation experiments.

Without positional arguments, the newest CSV for each permutation is found in
``outputs/mujoco_data``. Raw distances are kept in separate figures because
the force- and velocity-capability matrices define different quantities.
"""

import argparse
import csv
from pathlib import Path

from bimanual_redundancy import paths

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

from bimanual_redundancy.plotting.publication_style import (
    COLORS,
    configure_publication_style,
    finish_figure,
)


REPOSITORY_ROOT = paths.REPO_ROOT
DEFAULT_DATA_DIR = REPOSITORY_ROOT / "outputs" / "mujoco_data"
DEFAULT_OUTPUT_DIR = (
    DEFAULT_DATA_DIR / "directional_distance_comparison_figures"
)

CASE_ORDER = (
    "force_minimize",
    "force_maximize",
    "velocity_minimize",
    "velocity_maximize",
)
CASE_LABELS = {
    "force_minimize": "Force matrix: minimize distance",
    "force_maximize": "Force matrix: maximize distance",
    "velocity_minimize": "Velocity matrix: minimize distance",
    "velocity_maximize": "Velocity matrix: maximize distance",
}
CASE_TITLES = {
    "force_minimize": "Force-capability alignment (minimize)",
    "force_maximize": "Force-capability negative control (maximize)",
    "velocity_minimize": "Velocity-capability alignment (minimize)",
    "velocity_maximize": "Velocity-capability avoidance (maximize)",
}
CASE_COLORS = {
    "force_minimize": COLORS[0],
    "force_maximize": COLORS[1],
    "velocity_minimize": COLORS[2],
    "velocity_maximize": COLORS[4],
}


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "csv_files",
        nargs="*",
        type=Path,
        help="four permutation CSVs; omit to discover the newest set",
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


def discover_files(data_dir):
    files = []
    for case in CASE_ORDER:
        matches = sorted(
            data_dir.glob(
                "dual_franka_eq8_directional_distance_"
                f"{case}_fitted_spheres_*.csv"
            )
        )
        if not matches:
            raise FileNotFoundError(
                f"No {case!r} directional-distance CSV found in: {data_dir}"
            )
        files.append(matches[-1])
    return files


def load_csv(path):
    with path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    if not rows:
        raise ValueError(f"CSV contains no samples: {path}")
    case = rows[0].get("optimization_mode") or rows[0].get("objective")
    if case not in CASE_ORDER:
        raise ValueError(f"Unknown directional-distance case {case!r}: {path}")

    def values(column):
        if column not in rows[0]:
            raise KeyError(f"Missing column {column!r} in: {path}")
        result = np.asarray([float(row[column]) for row in rows])
        if not np.all(np.isfinite(result)):
            raise ValueError(f"Non-finite values in {column!r}: {path}")
        return result

    return {"path": path, "case": case, "rows": rows, "values": values}


def load_runs(paths):
    runs = {}
    for path in paths:
        run = load_csv(path.expanduser().resolve())
        case = run["case"]
        if case in runs:
            raise ValueError(f"Duplicate case supplied: {case}")
        runs[case] = run
    missing = set(CASE_ORDER) - set(runs)
    if missing:
        raise ValueError(f"Missing directional-distance cases: {sorted(missing)}")
    return runs


def plot_raw_distance(run):
    case = run["case"]
    matrix_label = (
        r"Force capability $C_f=(AA^\mathsf{T})^\dagger$"
        if case.startswith("force_")
        else r"Velocity capability $C_v=AA^\mathsf{T}$"
    )
    direction = "lower is better" if case.endswith("_minimize") else "higher is better"
    fig, axis = plt.subplots(figsize=(7.1, 3.4))
    sns.lineplot(
        x=run["values"]("time"),
        y=run["values"]("objective_value"),
        ax=axis,
        color=CASE_COLORS[case],
        errorbar=None,
    )
    axis.set(
        xlabel="Time (s)",
        ylabel=r"Normalized Frobenius distance $D(q)$",
        title=f"{CASE_TITLES[case]} — {direction}",
    )
    axis.text(
        0.99,
        0.04,
        matrix_label,
        transform=axis.transAxes,
        ha="right",
        va="bottom",
        fontsize=8,
    )
    axis.ticklabel_format(axis="y", style="plain", useOffset=False)
    return fig


def normalize_progress(values, minimize):
    """Map achieved progress to [0, 1], with 1 always meaning better."""
    minimum = np.min(values)
    maximum = np.max(values)
    span = maximum - minimum
    if np.isclose(span, 0.0):
        return np.zeros_like(values)
    normalized_value = (values - minimum) / span
    return 1.0 - normalized_value if minimize else normalized_value


def plot_normalized_progress(runs):
    fig, axis = plt.subplots(figsize=(7.1, 3.6))
    for case in CASE_ORDER:
        run = runs[case]
        progress = normalize_progress(
            run["values"]("objective_value"),
            minimize=case.endswith("_minimize"),
        )
        sns.lineplot(
            x=run["values"]("time"),
            y=progress,
            ax=axis,
            color=CASE_COLORS[case],
            label=CASE_LABELS[case],
            errorbar=None,
        )
    axis.set(
        xlabel="Time (s)",
        ylabel="Normalized optimization progress",
        title="Directional-distance permutation progress",
        ylim=(-0.02, 1.02),
    )
    axis.set_yticks(np.linspace(0.0, 1.0, 6))
    axis.legend(loc="best", ncol=2)
    return fig


def actuator_effort(run):
    values = run["values"]
    torque = np.column_stack(
        [values(f"tau_act_l{joint}") for joint in range(1, 8)]
        + [values(f"tau_act_r{joint}") for joint in range(1, 8)]
    )
    return np.sqrt(np.sum(torque * torque, axis=1))


def plot_effort(runs):
    fig, axis = plt.subplots(figsize=(7.1, 3.6))
    for case in CASE_ORDER:
        run = runs[case]
        sns.lineplot(
            x=run["values"]("time"),
            y=actuator_effort(run),
            ax=axis,
            color=CASE_COLORS[case],
            label=CASE_LABELS[case],
            errorbar=None,
        )
    axis.set(
        xlabel="Time (s)",
        ylabel=r"Actuator effort $\sqrt{\tau\tau^\mathsf{T}}=\|\tau\|_2$ (N m)",
        title="Actuator torque effort",
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


def write_summary(path, runs):
    lines = [
        "Directional-distance 2x2 permutation summary",
        "================================================",
    ]
    for case in CASE_ORDER:
        run = runs[case]
        distance = run["values"]("objective_value")
        effort = actuator_effort(run)
        lines.extend(
            [
                "",
                CASE_LABELS[case],
                f"  Source: {run['path'].name}",
                f"  Duration: {run['values']('time')[-1]:.3f} s",
                f"  Distance: {distance[0]:.6g} -> {distance[-1]:.6g}",
                f"  Minimum distance: {np.min(distance):.6g}",
                f"  Maximum distance: {np.max(distance):.6g}",
                f"  Peak actuator effort: {np.max(effort):.6g} N m",
                f"  RMS actuator effort: {np.sqrt(np.mean(effort**2)):.6g} N m",
            ]
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


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
    configure_publication_style()

    raw_figures = [plot_raw_distance(runs[case]) for case in CASE_ORDER]
    normalized_figure = plot_normalized_progress(runs)
    effort_figure = plot_effort(runs)
    figures = [
        (f"0{index}_{case}_distance", figure)
        for index, (case, figure) in enumerate(
            zip(CASE_ORDER, raw_figures),
            start=1,
        )
    ]
    figures.extend(
        [
            ("05_normalized_progress", normalized_figure),
            ("06_actuator_effort", effort_figure),
        ]
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
    summary_path = output_dir / "run_summary.txt"
    write_summary(summary_path, runs)
    print(
        f"Saved {len(written)} figures and summary to: {output_dir}"
    )
    if arguments.show:
        plt.show()
    else:
        plt.close("all")


if __name__ == "__main__":
    main()
