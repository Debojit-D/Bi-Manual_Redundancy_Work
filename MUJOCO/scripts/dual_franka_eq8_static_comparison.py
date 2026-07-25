"""Run all static Equation (8) modes at six table positions.

By default (sweep option 1), each table position runs five independent MuJoCo
scenes in this order:

1. baseline (no null-space optimization),
2. velocity manipulability,
3. force manipulability, and
4. directional-force manipulability, and
5. indirect directional-force manipulability (velocity-distance maximize).

The six positions therefore produce 30 independent runs.

Use ``--sweep-option 2`` to run each optimization mode at all six positions
before advancing to the next optimization mode.

Each viewer remains visible until the null-space motion has converged, then
closes before the next mode starts. A run is converged when its maximum
null-space joint speed stays below the selected threshold for a sustained
window. Supplying ``--duration`` instead runs every mode for the same fixed
number of seconds and disables convergence-based stopping.

Run position-first (option 1, the default)::

    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_static_comparison \
        --sweep-option 1

Run optimization-first (option 2)::

    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_static_comparison \
        --sweep-option 2

Use a straight-on front camera for every interactive run::

    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_static_comparison \
        --sweep-option 1 --front-view

Use the overhead camera instead::

    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_static_comparison \
        --sweep-option 1 --top-view

Record perspective, top, and front views for all 30 runs without opening a
viewer::

    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_static_comparison \
        --record-video --video-view all

Press Ctrl+C to close the active run and stop the entire sweep cleanly.
"""

import argparse
from datetime import datetime
from pathlib import Path

from MUJOCO.scripts.dual_franka_eq8_static_optimization import (
    main as run_static_experiment,
)
from MUJOCO.scripts.table_spawn_comparison_positions import (
    SWEEP_OPTIONS,
    TABLE_SPAWN_CASES,
    ordered_comparison_cases,
)
from MUJOCO.utils.camera_presets import (
    VIDEO_VIEW_CHOICES,
    video_views_for_choice,
)
from MUJOCO.utils.cli import add_camera_view_arguments, run_cli
from MUJOCO.utils.comparison_run_safety import (
    print_run_failure,
    print_sweep_summary,
)
from MUJOCO.utils.redundancy_optimization import ManipulabilityObjective
from MUJOCO.utils.video_recording import VIDEO_ENCODER_CHOICES


DEFAULT_CONVERGENCE_SPEED = 0.005
DEFAULT_CONVERGENCE_HOLD = 0.5
DEFAULT_MINIMUM_RUN_TIME = 1.0
EXPERIMENTS = (
    ("baseline", ManipulabilityObjective.FORCE, False),
    ("velocity", ManipulabilityObjective.VELOCITY, True),
    ("force", ManipulabilityObjective.FORCE, True),
    ("directional_force", ManipulabilityObjective.DIRECTIONAL_FORCE, True),
    (
        "directional_force_indirect",
        ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT,
        True,
    ),
)
EXPERIMENT_MODES = ("all",) + tuple(case[0] for case in EXPERIMENTS)
DEFAULT_VIDEO_ROOT = (
    Path(__file__).resolve().parents[2] / "outputs" / "mujoco_videos"
)
DEFAULT_DATA_ROOT = (
    Path(__file__).resolve().parents[2] / "outputs" / "mujoco_data"
)


def experiments_for_mode(mode):
    """Return all comparison modes or one explicitly selected mode."""
    if mode == "all":
        return EXPERIMENTS
    selected = tuple(case for case in EXPERIMENTS if case[0] == mode)
    if not selected:
        raise ValueError(f"mode must be one of {EXPERIMENT_MODES}")
    return selected


def parse_arguments():
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
            "run order: 1 = five modes at each position; "
            "2 = all six positions for each mode (default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--mode",
        choices=EXPERIMENT_MODES,
        default="all",
        help="run all optimization modes or one selected mode",
    )
    parser.add_argument(
        "--duration",
        type=float,
        help=(
            "fixed duration of each run in seconds; overrides convergence "
            "stopping"
        ),
    )
    parser.add_argument(
        "--convergence-speed",
        type=float,
        default=DEFAULT_CONVERGENCE_SPEED,
        help=(
            "maximum null-space joint speed considered converged in rad/s "
            "(default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--convergence-hold",
        type=float,
        default=DEFAULT_CONVERGENCE_HOLD,
        help="time the speed must remain below threshold (default: %(default)s)",
    )
    parser.add_argument(
        "--minimum-run-time",
        type=float,
        default=DEFAULT_MINIMUM_RUN_TIME,
        help="earliest convergence check in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--record-data",
        action="store_true",
        help="record every position/mode run to a labeled CSV",
    )
    parser.add_argument(
        "--data-output-dir",
        type=Path,
        default=DEFAULT_DATA_ROOT,
        help="root directory for timestamped comparison CSVs",
    )
    parser.add_argument(
        "--show-collision-spheres",
        action="store_true",
        help="draw the fitted collision spheres in every viewer",
    )
    add_camera_view_arguments(parser, scope="comparison runs")
    parser.add_argument(
        "--record-video",
        action="store_true",
        help="run headlessly and record the selected video view(s)",
    )
    parser.add_argument(
        "--video-view",
        choices=VIDEO_VIEW_CHOICES,
        default="both",
        help=(
            "record all, the legacy perspective/top pair, or one camera "
            "when using --record-video (default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--video-output-dir",
        type=Path,
        default=DEFAULT_VIDEO_ROOT,
        help="root directory for timestamped video runs (default: %(default)s)",
    )
    parser.add_argument("--video-width", type=int, default=1280)
    parser.add_argument("--video-height", type=int, default=720)
    parser.add_argument("--video-fps", type=int, default=30)
    parser.add_argument(
        "--video-encoder",
        choices=VIDEO_ENCODER_CHOICES,
        default="nvenc",
        help="H.264 encoder used for recorded videos (default: %(default)s)",
    )
    parser.add_argument(
        "--nvenc-max-views",
        type=int,
        choices=range(4),
        default=3,
        help="maximum selected views encoded with NVENC (default: %(default)s)",
    )
    parser.add_argument(
        "--disable-collision-penalty",
        action="store_true",
        help="disable the soft collision term for all optimized modes",
    )
    parser.add_argument(
        "--characteristic-length",
        type=float,
        default=None,
        help=(
            "manual spatial characteristic length in metres for every run; "
            "by default each run computes it from rigid object contact sites"
        ),
    )
    return parser.parse_args()


def main():
    arguments = parse_arguments()
    selected_experiments = experiments_for_mode(arguments.mode)
    selected_video_views = video_views_for_choice(arguments.video_view)
    if arguments.duration is not None and arguments.duration <= 0.0:
        raise ValueError("--duration must be greater than zero")
    if arguments.convergence_speed < 0.0:
        raise ValueError("--convergence-speed cannot be negative")
    if arguments.convergence_hold <= 0.0:
        raise ValueError("--convergence-hold must be greater than zero")
    if arguments.minimum_run_time < 0.0:
        raise ValueError("--minimum-run-time cannot be negative")
    if arguments.record_video and arguments.show_collision_spheres:
        raise ValueError(
            "--show-collision-spheres cannot be used with --record-video"
        )
    if arguments.video_width <= 0 or arguments.video_width % 2:
        raise ValueError("--video-width must be a positive even number")
    if arguments.video_height <= 0 or arguments.video_height % 2:
        raise ValueError("--video-height must be a positive even number")
    if arguments.video_fps <= 0:
        raise ValueError("--video-fps must be greater than zero")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    video_run_dir = None
    if arguments.record_video:
        video_run_dir = (
            arguments.video_output_dir.expanduser().resolve()
            / f"static_comparison_{timestamp}"
        )
        video_run_dir.mkdir(parents=True, exist_ok=False)
        print(f"Headless dual-view video output: {video_run_dir}")

    data_run_dir = None
    if arguments.record_data:
        data_run_dir = (
            arguments.data_output_dir.expanduser().resolve()
            / f"static_comparison_{timestamp}"
        )
        data_run_dir.mkdir(parents=True, exist_ok=False)
        print(f"Comparison CSV output: {data_run_dir}")

    total_runs = len(TABLE_SPAWN_CASES) * len(selected_experiments)
    print(
        "Static Equation (8) comparison: "
        f"{len(TABLE_SPAWN_CASES)} positions x "
        f"{len(selected_experiments)} mode(s) "
        f"= {total_runs} runs"
    )
    print(
        f"Sweep option {arguments.sweep_option}: "
        + (
            "position-first "
            f"({len(selected_experiments)} mode(s) at each position)."
            if arguments.sweep_option == 1
            else "optimization-first (all positions for each mode)."
        )
    )
    if arguments.record_video:
        print(
            "Running headlessly; recording "
            + ", ".join(selected_video_views)
            + "."
        )
    else:
        print("Close a viewer early to advance immediately to the next run.")
    if arguments.duration is None:
        print(
            "Stopping rule: convergence at null-space speed <= "
            f"{arguments.convergence_speed:g} rad/s for "
            f"{arguments.convergence_hold:g} s."
        )
    else:
        print(
            f"Stopping rule: fixed {arguments.duration:g} s per mode "
            "(convergence stopping disabled)."
        )
    run_cases = ordered_comparison_cases(
        TABLE_SPAWN_CASES,
        selected_experiments,
        arguments.sweep_option,
    )
    failed_runs = []
    for run_index, case in enumerate(run_cases, start=1):
        position_name, table_position, name, objective, enabled = case
        print("\n" + "=" * 72)
        print(
            f"Run {run_index}/{total_runs}: {position_name} / {name}; "
            f"table site_top_middle = {table_position} m"
        )
        print("=" * 72)
        run_label = f"{position_name} / {name}"
        try:
            run_static_experiment(
                objective=objective,
                characteristic_length=arguments.characteristic_length,
                enable_redundancy_optimization=enabled,
                duration=arguments.duration,
                convergence_speed_threshold=(
                    arguments.convergence_speed
                    if arguments.duration is None
                    else None
                ),
                convergence_hold_duration=arguments.convergence_hold,
                minimum_convergence_time=arguments.minimum_run_time,
                table_spawn_position=table_position,
                record_data=arguments.record_data,
                output_csv=(
                    data_run_dir / position_name / f"{name}.csv"
                    if data_run_dir is not None
                    else None
                ),
                show_collision_spheres=arguments.show_collision_spheres,
                enable_collision_penalty=(
                    not arguments.disable_collision_penalty
                ),
                top_view=arguments.top_view,
                front_view=arguments.front_view,
                video_output_dir=(
                    video_run_dir / position_name / name
                    if video_run_dir is not None
                    else None
                ),
                video_width=arguments.video_width,
                video_height=arguments.video_height,
                video_fps=arguments.video_fps,
                video_views=selected_video_views,
                video_encoder=arguments.video_encoder,
                video_nvenc_view_limit=arguments.nvenc_max_views,
            )
        except Exception as error:
            failed_runs.append(run_label)
            print_run_failure(run_label, error)

    print_sweep_summary(total_runs, failed_runs)


if __name__ == "__main__":
    run_cli(main)
