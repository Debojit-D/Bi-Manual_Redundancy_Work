"""Summarize object-tracking errors and controller latency from one batch.

The output is a compact CSV suitable for the simulation-results table in the
paper. Each row aggregates the six matched runs for one stage and control mode.

Example::

    .venv/bin/python -m MUJOCO.scripts.summarize_tracking_latency \
        outputs/equation8_comparison_batches/comparison_main_20260723_214914_681226

By default, the summary is written to
``<batch_dir>/tracking_latency_summary.csv``.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import re

import numpy as np
import pandas as pd

from MUJOCO.utils.cli import run_cli


STAGES = ("static", "pick_place", "6d_pick_place")
MODES = ("baseline", "velocity", "force", "directional_force")
CASE_PATTERN = re.compile(r"^(?:position|pose)_(\d+)$")
REQUIRED_COLUMNS = (
    "position_error_norm",
    "orientation_error_norm",
    "control_compute_time_ms",
)
OPTIONAL_TIMING_COLUMNS = (
    "controller_update_time_ms",
    "optimizer_time_ms",
)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "batch_dir",
        type=Path,
        help="comparison_main batch directory containing data/<stage>/",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="output CSV path (default: <batch_dir>/tracking_latency_summary.csv)",
    )
    return parser.parse_args(argv)


def case_number(path: Path) -> int:
    match = CASE_PATTERN.fullmatch(path.name)
    if match is None:
        raise ValueError(f"Unrecognized case directory: {path}")
    return int(match.group(1))


def case_directories(path: Path) -> tuple[Path, ...]:
    if not path.is_dir():
        return ()
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


def discover_stage_run_directory(batch_dir: Path, stage: str) -> Path:
    """Return the directory containing position_*/pose_* case folders."""
    stage_root = batch_dir / "data" / stage
    direct_cases = case_directories(stage_root)
    if direct_cases:
        return stage_root

    candidates = tuple(
        child
        for child in sorted(stage_root.iterdir())
        if child.is_dir() and case_directories(child)
    ) if stage_root.is_dir() else ()

    if not candidates:
        raise FileNotFoundError(
            f"No six-case CSV directory found for stage {stage!r} in {stage_root}"
        )
    if len(candidates) > 1:
        names = ", ".join(path.name for path in candidates)
        raise ValueError(
            f"Multiple run directories found for stage {stage!r}: {names}"
        )
    return candidates[0]


def load_run(path: Path) -> pd.DataFrame:
    frame = pd.read_csv(path)
    missing = set(REQUIRED_COLUMNS) - set(frame.columns)
    if missing:
        raise ValueError(f"Missing columns {sorted(missing)} in {path}")
    if frame.empty:
        raise ValueError(f"CSV contains no samples: {path}")

    checked_columns = list(REQUIRED_COLUMNS)
    checked_columns.extend(
        column for column in OPTIONAL_TIMING_COLUMNS if column in frame.columns
    )
    values = frame.loc[:, checked_columns].to_numpy(dtype=float)
    if not np.all(np.isfinite(values)):
        raise ValueError(f"CSV contains non-finite summary values: {path}")
    return frame


def root_mean_square(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=float)
    return float(np.sqrt(np.mean(values * values)))


def timing_statistics(frames: list[pd.DataFrame], column: str) -> dict[str, float]:
    available = [
        frame[column].to_numpy(dtype=float)
        for frame in frames
        if column in frame.columns
    ]
    if not available:
        return {
            f"{column}_mean": float("nan"),
            f"{column}_p95": float("nan"),
            f"{column}_max": float("nan"),
        }
    values = np.concatenate(available)
    return {
        f"{column}_mean": float(np.mean(values)),
        f"{column}_p95": float(np.percentile(values, 95.0)),
        f"{column}_max": float(np.max(values)),
    }


def summarize_mode(stage: str, mode: str, run_paths: list[Path]) -> dict[str, float | int | str]:
    frames = [load_run(path) for path in run_paths]

    position_rmse = np.array(
        [root_mean_square(frame["position_error_norm"]) for frame in frames]
    )
    orientation_rmse = np.array(
        [root_mean_square(frame["orientation_error_norm"]) for frame in frames]
    )
    position_all = np.concatenate(
        [frame["position_error_norm"].to_numpy(dtype=float) for frame in frames]
    )
    orientation_all = np.concatenate(
        [frame["orientation_error_norm"].to_numpy(dtype=float) for frame in frames]
    )

    row: dict[str, float | int | str] = {
        "stage": stage,
        "mode": mode,
        "run_count": len(frames),
        "sample_count": int(sum(len(frame) for frame in frames)),
        "position_rmse_mean_m": float(np.mean(position_rmse)),
        "position_rmse_worst_m": float(np.max(position_rmse)),
        "position_error_max_m": float(np.max(position_all)),
        "orientation_rmse_mean_rad": float(np.mean(orientation_rmse)),
        "orientation_rmse_worst_rad": float(np.max(orientation_rmse)),
        "orientation_error_max_rad": float(np.max(orientation_all)),
    }
    row.update(timing_statistics(frames, "control_compute_time_ms"))
    row.update(timing_statistics(frames, "controller_update_time_ms"))
    row.update(timing_statistics(frames, "optimizer_time_ms"))
    return row


def build_summary(batch_dir: Path) -> pd.DataFrame:
    rows = []
    for stage in STAGES:
        run_dir = discover_stage_run_directory(batch_dir, stage)
        cases = case_directories(run_dir)
        if len(cases) != 6:
            raise ValueError(
                f"Expected six cases for stage {stage!r}, found {len(cases)} in {run_dir}"
            )

        for mode in MODES:
            run_paths = [case_dir / f"{mode}.csv" for case_dir in cases]
            missing = [path for path in run_paths if not path.is_file()]
            if missing:
                formatted = "\n".join(str(path) for path in missing)
                raise FileNotFoundError(
                    f"Missing {mode!r} CSV files for stage {stage!r}:\n{formatted}"
                )
            rows.append(summarize_mode(stage, mode, run_paths))

    return pd.DataFrame(rows)


def main(argv=None):
    arguments = parse_arguments(argv)
    batch_dir = arguments.batch_dir.expanduser().resolve()
    if not batch_dir.is_dir():
        raise FileNotFoundError(f"Batch directory not found: {batch_dir}")

    output_path = (
        arguments.output.expanduser().resolve()
        if arguments.output is not None
        else batch_dir / "tracking_latency_summary.csv"
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)

    summary = build_summary(batch_dir)
    summary.to_csv(output_path, index=False, float_format="%.9g")
    print(summary.to_string(index=False))
    print(f"\nSaved tracking and latency summary to: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(run_cli(main))
