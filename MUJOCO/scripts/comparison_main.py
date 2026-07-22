"""Run the complete Equation (8) comparison and recording campaign.

The stages always run in this order:

1. static optimization,
2. ordinary pick-and-place, and
3. 6D pick-and-place.

Every stage runs baseline, velocity, force, and directional-force modes,
records the CSV data, and records perspective, top, and front MP4 files. By
default all six shared pickup positions/poses are processed, producing 72
runs when all six 6D paths are complete.

Run from the repository root::

    .venv/bin/python -m MUJOCO.scripts.comparison_main

Preview the exact three child commands without starting MuJoCo::

    .venv/bin/python -m MUJOCO.scripts.comparison_main --dry-run

Use ``--sweep-option 2`` for optimization-first ordering within each stage.
Press Ctrl+C to stop the active child cleanly and abort the remaining stages.
"""

import argparse
from datetime import datetime
from pathlib import Path
import re
import shlex
import subprocess
import sys
import time

from tqdm.auto import tqdm

from MUJOCO.scripts.table_spawn_comparison_positions import (
    SIX_D_TRAJECTORY_CASES,
    SWEEP_OPTIONS,
    TABLE_SPAWN_CASES,
)
from MUJOCO.utils import camera_presets
from MUJOCO.utils.cli import run_cli
from MUJOCO.utils.scene_builder import DualFrankaMuJoCoScene


STAGES = (
    ("static", "MUJOCO.scripts.dual_franka_eq8_static_comparison"),
    (
        "pick_place",
        "MUJOCO.scripts.dual_franka_eq8_pick_place_comparison",
    ),
    (
        "6d_pick_place",
        "MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison",
    ),
)
OPTIMIZATION_MODES = (
    "baseline",
    "velocity",
    "force",
    "directional_force",
)
DEFAULT_OUTPUT_ROOT = (
    Path(__file__).resolve().parents[2]
    / "outputs"
    / "equation8_comparison_batches"
)
RUN_BANNER_PATTERN = re.compile(r"(?:^|\r)Run\s+\d+/\d+:")


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--sweep-option",
        type=int,
        choices=SWEEP_OPTIONS,
        default=1,
        help=(
            "ordering within every stage: 1 = position/pose first; "
            "2 = optimization first (default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help="root for the combined timestamped data/video batch",
    )
    parser.add_argument(
        "--static-duration",
        type=float,
        default=None,
        help=(
            "fixed seconds per static run; by default use the static "
            "comparison's convergence stopping rule"
        ),
    )
    parser.add_argument(
        "--hold-duration",
        type=float,
        default=2.0,
        help="final hold seconds for pick-and-place stages",
    )
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--video-height", type=int, default=720)
    parser.add_argument("--video-fps", type=int, default=30)
    parser.add_argument(
        "--characteristic-length",
        type=float,
        default=None,
        help=(
            "optional manual spatial characteristic length in metres; "
            "default is automatic grasp-geometry calculation"
        ),
    )
    parser.add_argument(
        "--disable-collision-penalty",
        action="store_true",
        help="forward collision-penalty disabling to every stage",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the commands and configuration audit without running",
    )
    arguments = parser.parse_args(argv)
    if arguments.static_duration is not None and arguments.static_duration <= 0:
        parser.error("--static-duration must be greater than zero")
    if arguments.hold_duration < 0:
        parser.error("--hold-duration cannot be negative")
    if arguments.video_width <= 0 or arguments.video_width % 2:
        parser.error("--video-width must be a positive even number")
    if arguments.video_height <= 0 or arguments.video_height % 2:
        parser.error("--video-height must be a positive even number")
    if arguments.video_fps <= 0:
        parser.error("--video-fps must be greater than zero")
    if (
        arguments.characteristic_length is not None
        and arguments.characteristic_length <= 0
    ):
        parser.error("--characteristic-length must be greater than zero")
    return arguments


def validate_uniform_configuration():
    """Confirm the shared pickup positions and camera defaults are aligned."""
    table_pickups = tuple(position for _, position in TABLE_SPAWN_CASES)
    six_d_pickups = tuple(case[1] for case in SIX_D_TRAJECTORY_CASES)
    if table_pickups != six_d_pickups:
        raise RuntimeError(
            "TABLE_SPAWN_CASES and SIX_D_TRAJECTORY_CASES pickup positions "
            "are not aligned"
        )

    shared_distances = (
        camera_presets.PERSPECTIVE_CAMERA_DISTANCE,
        camera_presets.TOP_CAMERA_DISTANCE,
        camera_presets.FRONT_CAMERA_DISTANCE,
    )
    scene_distances = (
        DualFrankaMuJoCoScene.PERSPECTIVE_CAMERA_DISTANCE,
        DualFrankaMuJoCoScene.TOP_CAMERA_DISTANCE,
        DualFrankaMuJoCoScene.FRONT_CAMERA_DISTANCE,
    )
    if scene_distances != shared_distances:
        raise RuntimeError("Dual-Franka scene camera distances are not uniform")
    return table_pickups, shared_distances


def expected_run_counts():
    """Return per-stage run counts for the currently configured cases."""
    complete_6d_cases = sum(
        all(component is not None for component in case[1:])
        for case in SIX_D_TRAJECTORY_CASES
    )
    mode_count = len(OPTIMIZATION_MODES)
    return {
        "static": len(TABLE_SPAWN_CASES) * mode_count,
        "pick_place": len(TABLE_SPAWN_CASES) * mode_count,
        "6d_pick_place": complete_6d_cases * mode_count,
    }


def build_stage_commands(arguments, batch_dir):
    """Build static, pick/place, and 6D commands in required order."""
    commands = []
    for stage_name, module_name in STAGES:
        command = [
            sys.executable,
            "-u",
            "-m",
            module_name,
            "--sweep-option",
            str(arguments.sweep_option),
            "--record-data",
            "--record-video",
            "--video-view",
            "all",
            "--data-output-dir",
            str(batch_dir / "data" / stage_name),
            "--video-output-dir",
            str(batch_dir / "videos" / stage_name),
            "--video-width",
            str(arguments.video_width),
            "--video-height",
            str(arguments.video_height),
            "--video-fps",
            str(arguments.video_fps),
        ]
        if stage_name == "static" and arguments.static_duration is not None:
            command.extend(("--duration", str(arguments.static_duration)))
        if stage_name != "static":
            command.extend(("--hold-duration", str(arguments.hold_duration)))
        if stage_name == "6d_pick_place":
            command.extend(("--mode", "all"))
        if arguments.characteristic_length is not None:
            command.extend(
                (
                    "--characteristic-length",
                    str(arguments.characteristic_length),
                )
            )
        if arguments.disable_collision_penalty:
            command.append("--disable-collision-penalty")
        commands.append((stage_name, tuple(command)))
    return tuple(commands)


def run_stage(command, stage_name, progress):
    """Stream one child process while advancing completed-run progress."""
    stage_start = time.perf_counter()
    active_run_start = None
    runs_started = 0
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    try:
        if process.stdout is None:
            raise RuntimeError("comparison child stdout pipe is unavailable")
        for line in process.stdout:
            print(line, end="", flush=True)
            # Each comparison child prints exactly one "Run i/N:" banner at
            # the start of a case. Reaching the next banner proves that the
            # previous case completed successfully.
            if RUN_BANNER_PATTERN.search(line):
                now = time.perf_counter()
                if active_run_start is not None:
                    progress.update(1)
                runs_started += 1
                active_run_start = now
                progress.set_postfix_str(
                    f"{stage_name}: run {runs_started}",
                    refresh=True,
                )
        return_code = process.wait()
    except KeyboardInterrupt:
        if process.poll() is None:
            process.terminate()
        process.wait()
        raise
    finally:
        if process.stdout is not None:
            process.stdout.close()

    if return_code != 0:
        raise subprocess.CalledProcessError(return_code, command)
    if active_run_start is not None:
        progress.update(1)
    return time.perf_counter() - stage_start, runs_started


def main(argv=None):
    arguments = parse_arguments(argv)
    pickup_positions, camera_distances = validate_uniform_configuration()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    batch_dir = (
        arguments.output_dir.expanduser().resolve()
        / f"comparison_main_{timestamp}"
    )
    commands = build_stage_commands(arguments, batch_dir)

    print("Unified Equation (8) comparison campaign")
    print("Stage order: static -> pick_place -> 6d_pick_place")
    print("Modes per position/pose: baseline, velocity, force, directional_force")
    print(f"Shared pickup positions verified: {len(pickup_positions)}")
    print(
        "Universal camera distances (perspective, top, front): "
        f"{camera_distances} m"
    )
    print(f"Combined output root: {batch_dir}")

    run_counts = expected_run_counts()
    total_runs = sum(run_counts.values())
    print(
        f"Expected runs: {total_runs} total "
        f"({run_counts['static']} static, "
        f"{run_counts['pick_place']} pick/place, "
        f"{run_counts['6d_pick_place']} 6D)."
    )
    print(
        "Wall-clock ETA will appear after the first completed run and "
        "will continuously adapt to measured performance."
    )

    campaign_start = time.perf_counter()
    if arguments.dry_run:
        for index, (stage_name, command) in enumerate(commands, start=1):
            print(f"\nStage {index}/{len(commands)}: {stage_name}")
            print(shlex.join(command))
    else:
        with tqdm(
            total=total_runs,
            desc="Equation (8) campaign",
            unit="run",
            dynamic_ncols=True,
        ) as progress:
            for index, (stage_name, command) in enumerate(commands, start=1):
                print(f"\nStage {index}/{len(commands)}: {stage_name}")
                print(shlex.join(command))
                stage_seconds, observed_runs = run_stage(
                    command,
                    stage_name,
                    progress,
                )
                print(
                    f"Stage {stage_name} completed: {observed_runs} runs in "
                    f"{stage_seconds / 60.0:.1f} minutes."
                )
            progress.set_postfix_str("complete", refresh=True)

    if arguments.dry_run:
        print("\nDry run complete; no simulation or output files were created.")
    else:
        campaign_seconds = time.perf_counter() - campaign_start
        print(
            "\nAll comparison stages completed in "
            f"{campaign_seconds / 3600.0:.2f} hours. Output: {batch_dir}"
        )
    return batch_dir


if __name__ == "__main__":
    run_cli(main)
