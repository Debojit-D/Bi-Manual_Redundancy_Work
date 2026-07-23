"""Run the simple Equation (8) pick-and-place comparison at six positions.

With sweep option 1 (the default), each configured table position runs four
independent MuJoCo scenes sequentially: baseline, velocity manipulability,
force manipulability, and directional-force manipulability. Sweep option 2
runs each mode at all six positions before advancing to the next mode. Both
options produce the same 24 runs. Each scene performs the same lift-and-return
trajectory and then holds for the configured interval. Close a viewer early to
advance to the next run.

Run from the repository root with the project virtual environment::

    # Option 1: run all four modes at each position.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --sweep-option 1

    # Option 2: run each optimization mode across all six positions.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --sweep-option 2

    # Change the final hold duration for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --hold-duration 5

    # Record every control step to a labeled CSV per position and mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --record-data

    # Display the fitted inter-arm collision spheres in every viewer.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --show-collision-spheres

    # Use a full overhead camera for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --top-view

    # Use a straight-on front camera for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --front-view

    # Run headlessly and record all three camera views for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --record-video --video-view all

    # Compare the objectives without the soft collision penalty.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --disable-collision-penalty

    # Override the shared optimization gain and joint-speed limit.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --optimization-gain 3000 --max-joint-speed 3

Options can be combined. Press Ctrl+C to close the active run and stop the
entire sweep cleanly. Run with ``--help`` for the complete option list.
"""

import argparse
from datetime import datetime
from pathlib import Path

from MUJOCO.scripts import dual_franka_eq8_optimized_pick_place as experiment
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


EXPERIMENTS = (
    ("baseline", ManipulabilityObjective.FORCE, False),
    ("velocity", ManipulabilityObjective.VELOCITY, True),
    ("force", ManipulabilityObjective.FORCE, True),
    ("directional_force", ManipulabilityObjective.DIRECTIONAL_FORCE, True),
)
DEFAULT_VIDEO_ROOT = (
    Path(__file__).resolve().parents[2] / "outputs" / "mujoco_videos"
)
DEFAULT_DATA_ROOT = (
    Path(__file__).resolve().parents[2] / "outputs" / "mujoco_data"
)


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
            "run order: 1 = four modes at each position; "
            "2 = all six positions for each mode (default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--hold-duration",
        type=float,
        default=2.0,
        help="final hold time for each run in seconds (default: %(default)s)",
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
        help="draw fitted collision spheres in every viewer",
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
        help="disable the soft collision term for optimized modes",
    )
    parser.add_argument(
        "--optimization-gain",
        type=float,
        default=experiment.OPTIMIZATION_GAIN,
        help="null-space gradient gain (default: %(default)s)",
    )
    parser.add_argument(
        "--max-joint-speed",
        type=float,
        default=experiment.MAXIMUM_OPTIMIZATION_JOINT_SPEED,
        help="symmetric joint-speed bound in rad/s (default: %(default)s)",
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


def validate_arguments(arguments):
    if arguments.hold_duration < 0.0:
        raise ValueError("--hold-duration cannot be negative")
    if arguments.optimization_gain <= 0.0:
        raise ValueError("--optimization-gain must be greater than zero")
    if arguments.max_joint_speed <= 0.0:
        raise ValueError("--max-joint-speed must be greater than zero")
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


def main():
    arguments = parse_arguments()
    validate_arguments(arguments)
    selected_video_views = video_views_for_choice(arguments.video_view)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    video_run_dir = None
    if arguments.record_video:
        video_run_dir = (
            arguments.video_output_dir.expanduser().resolve()
            / f"pick_place_comparison_{timestamp}"
        )
        video_run_dir.mkdir(parents=True, exist_ok=False)
        print(f"Headless dual-view video output: {video_run_dir}")

    data_run_dir = None
    if arguments.record_data:
        data_run_dir = (
            arguments.data_output_dir.expanduser().resolve()
            / f"pick_place_comparison_{timestamp}"
        )
        data_run_dir.mkdir(parents=True, exist_ok=False)
        print(f"Comparison CSV output: {data_run_dir}")

    total_runs = len(TABLE_SPAWN_CASES) * len(EXPERIMENTS)
    print(
        "Equation (8) pick-and-place comparison: "
        f"{len(TABLE_SPAWN_CASES)} positions x {len(EXPERIMENTS)} modes "
        f"= {total_runs} runs"
    )
    print(
        f"Sweep option {arguments.sweep_option}: "
        + (
            "position-first (four modes at each position)."
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
    run_cases = ordered_comparison_cases(
        TABLE_SPAWN_CASES,
        EXPERIMENTS,
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
            experiment.main(
                objective=objective,
                characteristic_length=arguments.characteristic_length,
                enable_redundancy_optimization=enabled,
                hold_duration=arguments.hold_duration,
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
                optimization_gain=arguments.optimization_gain,
                maximum_joint_speed=arguments.max_joint_speed,
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
