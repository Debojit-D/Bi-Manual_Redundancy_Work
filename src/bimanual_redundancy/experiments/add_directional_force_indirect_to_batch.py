"""Add only directional-force-indirect runs to an existing comparison batch.

This add-on is intended for a batch that already contains the original four
Equation (8) modes. It records the new ``directional_force_indirect`` mode for
all six static positions, all six ordinary pick-and-place positions, and all
six complete 6D trajectories. Existing files are never overwritten: each
comparison child creates a new timestamped directory below the batch's
existing ``data/<stage>`` and ``videos/<stage>`` roots.

The default target is the existing July 23 comparison batch::

    .venv/bin/python -m \
        bimanual_redundancy.experiments.add_directional_force_indirect_to_batch

Inspect the exact three commands without running MuJoCo::

    .venv/bin/python -m \
        bimanual_redundancy.experiments.add_directional_force_indirect_to_batch --dry-run

Supply another compatible batch directory when needed::

    .venv/bin/python -m \
        bimanual_redundancy.experiments.add_directional_force_indirect_to_batch \
        --batch-dir outputs/equation8_comparison_batches/comparison_main_...
"""

import argparse
from pathlib import Path

from bimanual_redundancy import paths
import shlex
import sys
import time

from tqdm.auto import tqdm

from bimanual_redundancy.experiments import comparison_main
from bimanual_redundancy.experiments.table_spawn_comparison_positions import (
    SIX_D_TRAJECTORY_CASES,
    SWEEP_OPTIONS,
    TABLE_SPAWN_CASES,
)
from bimanual_redundancy.simulation.cli import run_cli
from bimanual_redundancy.simulation.recording import VIDEO_ENCODER_CHOICES


OBJECTIVE_MODE = "directional_force_indirect"
DEFAULT_BATCH_DIR = (
    paths.OUTPUT_COMPARISON_BATCHES_DIR / "comparison_main_20260723_214914_681226"
)


def parse_arguments(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--batch-dir",
        type=Path,
        default=DEFAULT_BATCH_DIR,
        help="existing comparison batch to extend (default: %(default)s)",
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
        "--static-duration",
        type=float,
        default=None,
        help=(
            "fixed seconds per static run; by default use convergence-based "
            "stopping"
        ),
    )
    parser.add_argument(
        "--hold-duration",
        type=float,
        default=2.0,
        help="final hold seconds for both moving stages (default: %(default)s)",
    )
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--video-height", type=int, default=720)
    parser.add_argument("--video-fps", type=int, default=50)
    parser.add_argument(
        "--video-encoder",
        choices=VIDEO_ENCODER_CHOICES,
        default="nvenc",
        help="H.264 encoder for recorded videos (default: %(default)s)",
    )
    parser.add_argument(
        "--nvenc-session-limit",
        type=int,
        default=8,
        help=(
            "maximum simultaneous NVENC streams; excess views use x264 "
            "(default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=3,
        help=(
            "maximum independent stage processes to run concurrently "
            f"(1-{len(comparison_main.STAGES)}; default: %(default)s)"
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
        help="disable collision penalties in every indirect run",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="print the commands without starting simulations",
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="show child-process logs and generated commands",
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
    if not 1 <= arguments.workers <= len(comparison_main.STAGES):
        parser.error(
            f"--workers must be between 1 and {len(comparison_main.STAGES)}"
        )
    if (
        arguments.characteristic_length is not None
        and arguments.characteristic_length <= 0
    ):
        parser.error("--characteristic-length must be greater than zero")
    return arguments


def validate_batch_directory(batch_dir):
    """Resolve a compatible existing batch without modifying it."""
    batch_dir = batch_dir.expanduser().resolve()
    if not batch_dir.is_dir():
        raise FileNotFoundError(f"Comparison batch does not exist: {batch_dir}")
    for output_kind in ("data", "videos"):
        for stage_name, _ in comparison_main.STAGES:
            stage_root = batch_dir / output_kind / stage_name
            if not stage_root.is_dir():
                raise FileNotFoundError(
                    f"Expected existing batch directory: {stage_root}"
                )
    return batch_dir


def expected_run_counts():
    """Return the six indirect-only runs expected from each stage."""
    complete_6d_cases = sum(
        all(component is not None for component in case[1:])
        for case in SIX_D_TRAJECTORY_CASES
    )
    return {
        "static": len(TABLE_SPAWN_CASES),
        "pick_place": len(TABLE_SPAWN_CASES),
        "6d_pick_place": complete_6d_cases,
    }


def build_stage_commands(arguments, batch_dir):
    """Build one indirect-only command for each comparison stage."""
    commands = []
    nvenc_limits = comparison_main.stage_nvenc_view_limits(arguments)
    for stage_name, module_name in comparison_main.STAGES:
        command = [
            sys.executable,
            "-u",
            "-m",
            module_name,
            "--sweep-option",
            str(arguments.sweep_option),
            "--mode",
            OBJECTIVE_MODE,
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


def main(argv=None):
    arguments = parse_arguments(argv)
    comparison_main.validate_uniform_configuration()
    batch_dir = validate_batch_directory(arguments.batch_dir)
    commands = build_stage_commands(arguments, batch_dir)
    run_counts = expected_run_counts()
    total_runs = sum(run_counts.values())

    print("Directional-force-indirect comparison add-on")
    print(f"Existing batch: {batch_dir}")
    print(
        f"Expected new runs: {total_runs} total "
        f"({run_counts['static']} static, "
        f"{run_counts['pick_place']} pick/place, "
        f"{run_counts['6d_pick_place']} 6D)."
    )
    print(
        "Expected new artifacts: 18 CSV files and 54 MP4 files "
        "(perspective, top, and front)."
    )

    if arguments.dry_run:
        for index, (stage_name, command) in enumerate(commands, start=1):
            print(f"\nStage {index}/{len(commands)}: {stage_name}")
            print(shlex.join(command))
        print("\nDry run complete; no files were created.")
        return batch_dir

    start = time.perf_counter()
    with tqdm(
        total=total_runs,
        desc="Indirect objective add-on",
        unit="run",
        dynamic_ncols=True,
    ) as progress:
        comparison_main.run_stages(
            commands,
            progress,
            arguments.workers,
            verbose=arguments.verbose,
        )
        progress.set_postfix_str("complete", refresh=True)

    elapsed = time.perf_counter() - start
    print(
        "\nDirectional-force-indirect add-on completed in "
        f"{elapsed / 3600.0:.2f} hours. Updated batch: {batch_dir}"
    )
    return batch_dir


if __name__ == "__main__":
    run_cli(main)
