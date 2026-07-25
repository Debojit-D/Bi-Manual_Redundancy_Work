"""Run the complete Equation (8) comparison and recording campaign.

The campaign contains three independent stages:

1. static optimization,
2. ordinary pick-and-place, and
3. 6D pick-and-place.

By default the three stages run concurrently in separate processes. Every
individual simulation and its optimization loop remain sequential. Use
``--workers 1`` to reproduce the former stage-by-stage execution.

Every stage runs baseline, velocity, force, directional-force, and indirect
directional-force modes, records the CSV data, and records perspective, top,
and front MP4 files. By default all six shared pickup positions/poses are
processed, producing 90 runs when all six 6D paths are complete.

Run from the repository root::

    .venv/bin/python -m MUJOCO.scripts.comparison_main

Preview the exact three child commands without starting MuJoCo::

    .venv/bin/python -m MUJOCO.scripts.comparison_main --dry-run

Use ``--sweep-option 2`` for optimization-first ordering within each stage.
Press Ctrl+C to stop all active children and abort the remaining stages.
"""

import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor, as_completed
from datetime import datetime
from pathlib import Path
import re
import shlex
import subprocess
import sys
import threading
import time

from tqdm.auto import tqdm

from MUJOCO.scripts.table_spawn_comparison_positions import (
    SIX_D_TRAJECTORY_CASES,
    SWEEP_OPTIONS,
    TABLE_SPAWN_CASES,
)
from MUJOCO.utils import camera_presets
from MUJOCO.utils.cli import run_cli
from MUJOCO.utils.comparison_run_safety import (
    INCOMPLETE_SWEEP_MARKER,
    RUN_FAILURE_MARKER,
)
from MUJOCO.utils.scene_builder import DualFrankaMuJoCoScene
from MUJOCO.utils.video_recording import VIDEO_ENCODER_CHOICES


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
    "directional_force_indirect",
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
    parser.add_argument("--video-fps", type=int, default=50)
    parser.add_argument(
        "--video-encoder",
        choices=VIDEO_ENCODER_CHOICES,
        default="nvenc",
        help=(
            "H.264 encoder: GPU NVENC or software x264 fallback "
            "(default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--nvenc-session-limit",
        type=int,
        default=8,
        help=(
            "maximum simultaneous NVENC streams; excess camera views use "
            "x264 (default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=3,
        help=(
            "maximum independent stage processes to run concurrently "
            f"(1-{len(STAGES)}; default: %(default)s)"
        ),
    )
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
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="show child-process logs and generated commands while running",
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
    if arguments.nvenc_session_limit <= 0:
        parser.error("--nvenc-session-limit must be greater than zero")
    if not 1 <= arguments.workers <= len(STAGES):
        parser.error(f"--workers must be between 1 and {len(STAGES)}")
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
    return {
        "static": len(TABLE_SPAWN_CASES) * len(OPTIMIZATION_MODES),
        "pick_place": len(TABLE_SPAWN_CASES) * len(OPTIMIZATION_MODES),
        "6d_pick_place": complete_6d_cases * len(OPTIMIZATION_MODES),
    }


def stage_nvenc_view_limits(arguments):
    """Allocate hardware streams without exceeding the concurrent limit."""
    if arguments.video_encoder != "nvenc":
        return {}

    stage_count = len(STAGES)
    views_per_stage = 3
    active_stages = min(arguments.workers, stage_count)
    if active_stages < stage_count:
        per_stage = min(
            views_per_stage,
            arguments.nvenc_session_limit // active_stages,
        )
        return {stage_name: per_stage for stage_name, _ in STAGES}

    usable_sessions = min(
        arguments.nvenc_session_limit,
        stage_count * views_per_stage,
    )
    base, extra = divmod(usable_sessions, stage_count)
    return {
        stage_name: min(views_per_stage, base + (index < extra))
        for index, (stage_name, _) in enumerate(STAGES)
    }


def build_stage_commands(arguments, batch_dir):
    """Build static, pick/place, and 6D commands in required order."""
    commands = []
    nvenc_limits = stage_nvenc_view_limits(arguments)
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
            "--video-encoder",
            arguments.video_encoder,
        ]
        if stage_name in nvenc_limits:
            command.extend(
                ("--nvenc-max-views", str(nvenc_limits[stage_name]))
            )
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


def run_stage(
    command,
    stage_name,
    progress,
    *,
    active_processes=None,
    process_lock=None,
    output_lock=None,
    verbose=False,
):
    """Stream one child process while advancing completed-run progress."""
    stage_start = time.perf_counter()
    active_run_start = None
    runs_started = 0
    recent_output = deque(maxlen=80)
    process = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
    )
    if active_processes is not None:
        with process_lock:
            active_processes[stage_name] = process
    try:
        if process.stdout is None:
            raise RuntimeError("comparison child stdout pipe is unavailable")
        for line in process.stdout:
            recent_output.append(line)
            important_warning = (
                RUN_FAILURE_MARKER in line
                or INCOMPLETE_SWEEP_MARKER in line
            )
            if verbose or important_warning:
                if output_lock is None:
                    print(f"[{stage_name}] {line}", end="", flush=True)
                else:
                    with output_lock:
                        print(f"[{stage_name}] {line}", end="", flush=True)
            # Each comparison child prints exactly one "Run i/N:" banner at
            # the start of a case. Reaching the next banner proves that the
            # previous case completed successfully.
            if RUN_BANNER_PATTERN.search(line):
                now = time.perf_counter()
                runs_started += 1
                if output_lock is None:
                    if active_run_start is not None:
                        progress.update(1)
                    progress.set_postfix_str(
                        f"{stage_name}: run {runs_started}",
                        refresh=True,
                    )
                else:
                    with output_lock:
                        if active_run_start is not None:
                            progress.update(1)
                        progress.set_postfix_str(
                            f"{stage_name}: run {runs_started}",
                            refresh=True,
                        )
                active_run_start = now
        return_code = process.wait()
    except KeyboardInterrupt:
        if process.poll() is None:
            process.terminate()
        process.wait()
        raise
    finally:
        if active_processes is not None:
            with process_lock:
                active_processes.pop(stage_name, None)
        if process.stdout is not None:
            process.stdout.close()

    if return_code != 0:
        if not verbose:
            diagnostic = "".join(recent_output)
            message = (
                f"\nStage {stage_name} failed with status {return_code}. "
                "Recent output follows:\n"
                f"{diagnostic}"
            )
            if output_lock is None:
                print(message, file=sys.stderr, flush=True)
            else:
                with output_lock:
                    print(message, file=sys.stderr, flush=True)
        raise subprocess.CalledProcessError(return_code, command)
    if active_run_start is not None:
        if output_lock is None:
            progress.update(1)
        else:
            with output_lock:
                progress.update(1)
    return time.perf_counter() - stage_start, runs_started


def terminate_active_processes(active_processes, process_lock):
    """Stop campaign children that are still running after an interruption."""
    with process_lock:
        processes = tuple(active_processes.values())
    for process in processes:
        if process.poll() is None:
            process.terminate()
    for process in processes:
        if process.poll() is not None:
            continue
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()


def run_stages(commands, progress, workers, *, verbose=False):
    """Run independent stages concurrently without changing their internals."""
    active_processes = {}
    process_lock = threading.Lock()
    output_lock = threading.Lock()
    results = {}

    executor = ThreadPoolExecutor(
        max_workers=workers,
        thread_name_prefix="equation8-stage",
    )
    futures = {}
    try:
        for stage_name, command in commands:
            future = executor.submit(
                run_stage,
                command,
                stage_name,
                progress,
                active_processes=active_processes,
                process_lock=process_lock,
                output_lock=output_lock,
                verbose=verbose,
            )
            futures[future] = stage_name

        for future in as_completed(futures):
            stage_name = futures[future]
            results[stage_name] = future.result()
            stage_seconds, observed_runs = results[stage_name]
            with output_lock:
                if verbose:
                    print(
                        f"Stage {stage_name} completed: {observed_runs} runs "
                        f"in {stage_seconds / 60.0:.1f} minutes."
                    )
                progress.set_postfix_str(
                    f"{stage_name}: {observed_runs} runs complete",
                    refresh=True,
                )
    except BaseException:
        terminate_active_processes(active_processes, process_lock)
        for future in futures:
            future.cancel()
        raise
    finally:
        executor.shutdown(wait=True, cancel_futures=True)

    return tuple(
        (stage_name, *results[stage_name])
        for stage_name, _ in commands
    )


def main(argv=None):
    arguments = parse_arguments(argv)
    pickup_positions, camera_distances = validate_uniform_configuration()
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    batch_dir = (
        arguments.output_dir.expanduser().resolve()
        / f"comparison_main_{timestamp}"
    )
    commands = build_stage_commands(arguments, batch_dir)

    nvenc_limits = stage_nvenc_view_limits(arguments)
    show_audit = arguments.verbose or arguments.dry_run
    if show_audit:
        print("Unified Equation (8) comparison campaign")
        print(
            f"Stage workers: {arguments.workers} "
            "(use --workers 1 for sequential stages)"
        )
        print(f"Video encoder: {arguments.video_encoder}")
    if show_audit and nvenc_limits:
        print(
            "NVENC views per stage: "
            + ", ".join(
                f"{stage_name}={limit}"
                for stage_name, limit in nvenc_limits.items()
            )
            + "; remaining views use x264."
        )

    run_counts = expected_run_counts()
    total_runs = sum(run_counts.values())
    if show_audit:
        print(
            "Modes per position/pose: "
            "baseline, velocity, force, directional_force, "
            "directional_force_indirect"
        )
        print(f"Shared pickup positions verified: {len(pickup_positions)}")
        print(
            "Universal camera distances (perspective, top, front): "
            f"{camera_distances} m"
        )
        print(f"Combined output root: {batch_dir}")
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
            if arguments.verbose:
                for index, (stage_name, command) in enumerate(
                    commands,
                    start=1,
                ):
                    print(f"\nStage {index}/{len(commands)} queued: {stage_name}")
                    print(shlex.join(command))
            run_stages(
                commands,
                progress,
                arguments.workers,
                verbose=arguments.verbose,
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
