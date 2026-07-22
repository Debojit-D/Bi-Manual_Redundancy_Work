"""Run the spatial Equation (8) comparison from six pickup positions.

With sweep option 1 (the default), each shared pickup position runs baseline,
velocity manipulability, force manipulability, and directional-force
manipulability. Sweep option 2 runs each mode at all six pickup positions
before advancing to the next mode. Both orders produce 24 independent scenes.
Every run uses the same user-configurable intermediate and goal poses. Close a
viewer early to advance immediately to the next run.

Run from the repository root with the project virtual environment::

    # Option 1: run all four modes at each pickup position (24 runs).
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --sweep-option 1

    # Option 2: run each optimization mode across all six pickup positions.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --sweep-option 2

    # Run only pickup position 2 x all four modes, with your place locations.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --position 2 \\
        --intermediate-position 0.40 0.00 0.52 \\
        --goal-position 0.20 0.45 0.269

    # Position map [x, y, z] metres (pickup/start position only):
    # 1=(0.30, 0.20, 0.28), 2=(0.60, 0.20, 0.28)
    # 3=(0.30, 0.00, 0.28), 4=(0.60, 0.00, 0.28)
    # 5=(0.30,-0.20, 0.28), 6=(0.60,-0.20, 0.28)

    # Change the final hold duration for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --hold-duration 5

    # Record each mode and display the fitted collision spheres.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --record-data --show-collision-spheres

    # Use a full overhead camera for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --top-view

    # Use a straight-on front camera for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --front-view

    # Run headlessly and record both camera views for every mode.
    .venv/bin/python -m \\
        MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --record-video

    # Compare the objectives without the soft collision penalty.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --disable-collision-penalty

    # Use one custom pickup plus custom intermediate and goal positions.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --start-position 0.40 -0.15 0.269 \\
        --intermediate-position 0.40 0.00 0.52 \\
        --goal-position 0.20 0.45 0.269

    # Override the corresponding extrinsic XYZ orientations (radians).
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --start-euler-xyz 0 0 1.5708 \\
        --intermediate-euler-xyz 0 0 1.9708 \\
        --goal-euler-xyz 0 0 2.3708

    # Change both path-segment durations and optimizer limits.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --start-to-intermediate-duration 12 \\
        --intermediate-to-goal-duration 12 \\
        --optimization-gain 3000 --max-joint-speed 3

Options can be combined. Press Ctrl+C to close the active run and stop the
entire sweep cleanly. Run with ``--help`` for the complete option list.
"""

import argparse
from datetime import datetime
from pathlib import Path

from MUJOCO.scripts import dual_franka_eq8_optimized_6d_pick_place as experiment
from MUJOCO.scripts.table_spawn_comparison_positions import (
    SWEEP_OPTIONS,
    TABLE_SPAWN_CASES,
    ordered_comparison_cases,
    table_spawn_position_for_number,
)
from MUJOCO.utils.cli import add_camera_view_arguments, run_cli
from MUJOCO.utils.redundancy_optimization import ManipulabilityObjective


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
            "run order: 1 = four modes at each pickup position; "
            "2 = all pickup positions for each mode (default: %(default)s)"
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
        help="record every pickup/mode run to a labeled CSV",
    )
    parser.add_argument(
        "--data-output-dir",
        type=Path,
        default=DEFAULT_DATA_ROOT,
        help="root directory for timestamped comparison CSVs",
    )
    parser.add_argument("--show-collision-spheres", action="store_true")
    add_camera_view_arguments(parser, scope="comparison runs")
    parser.add_argument(
        "--record-video",
        action="store_true",
        help="run headlessly and record perspective plus top-view MP4s",
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
    parser.add_argument("--disable-collision-penalty", action="store_true")
    parser.add_argument(
        "--optimization-gain",
        type=float,
        default=experiment.OPTIMIZATION_GAIN,
    )
    parser.add_argument(
        "--max-joint-speed",
        type=float,
        default=experiment.MAXIMUM_OPTIMIZATION_JOINT_SPEED,
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
    pickup_group = parser.add_mutually_exclusive_group()
    pickup_group.add_argument(
        "--position",
        type=int,
        choices=range(1, len(TABLE_SPAWN_CASES) + 1),
        help="run only predefined pickup position 1 through 6",
    )
    pickup_group.add_argument(
        "--start-position",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="run one custom pickup/start position",
    )
    parser.add_argument(
        "--start-euler-xyz",
        type=float,
        nargs=3,
        default=experiment.TABLE_START_EULER_XYZ,
        metavar=("RX", "RY", "RZ"),
    )
    parser.add_argument(
        "--intermediate-position",
        type=float,
        nargs=3,
        default=experiment.TABLE_INTERMEDIATE_POSITION,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--intermediate-euler-xyz",
        type=float,
        nargs=3,
        default=experiment.TABLE_INTERMEDIATE_EULER_XYZ,
        metavar=("RX", "RY", "RZ"),
    )
    parser.add_argument(
        "--goal-position",
        type=float,
        nargs=3,
        default=experiment.TABLE_GOAL_POSITION,
        metavar=("X", "Y", "Z"),
    )
    parser.add_argument(
        "--goal-euler-xyz",
        type=float,
        nargs=3,
        default=experiment.TABLE_GOAL_EULER_XYZ,
        metavar=("RX", "RY", "RZ"),
    )
    parser.add_argument(
        "--start-to-intermediate-duration",
        type=float,
        default=experiment.START_TO_INTERMEDIATE_DURATION,
    )
    parser.add_argument(
        "--intermediate-to-goal-duration",
        type=float,
        default=experiment.INTERMEDIATE_TO_GOAL_DURATION,
    )
    return parser.parse_args()


def validate_arguments(arguments):
    if arguments.hold_duration < 0.0:
        raise ValueError("--hold-duration cannot be negative")
    if arguments.optimization_gain <= 0.0:
        raise ValueError("--optimization-gain must be greater than zero")
    if arguments.max_joint_speed <= 0.0:
        raise ValueError("--max-joint-speed must be greater than zero")
    if arguments.start_to_intermediate_duration <= 0.0:
        raise ValueError("--start-to-intermediate-duration must be positive")
    if arguments.intermediate_to_goal_duration <= 0.0:
        raise ValueError("--intermediate-to-goal-duration must be positive")
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

    if arguments.position is not None:
        pickup_cases = (
            (
                f"position_{arguments.position}",
                table_spawn_position_for_number(arguments.position),
            ),
        )
    elif arguments.start_position is not None:
        pickup_cases = (("custom_position", arguments.start_position),)
    else:
        pickup_cases = TABLE_SPAWN_CASES

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    video_run_dir = None
    if arguments.record_video:
        video_run_dir = (
            arguments.video_output_dir.expanduser().resolve()
            / f"6d_pick_place_comparison_{timestamp}"
        )
        video_run_dir.mkdir(parents=True, exist_ok=False)
        print(f"Headless dual-view video output: {video_run_dir}")

    data_run_dir = None
    if arguments.record_data:
        data_run_dir = (
            arguments.data_output_dir.expanduser().resolve()
            / f"6d_pick_place_comparison_{timestamp}"
        )
        data_run_dir.mkdir(parents=True, exist_ok=False)
        print(f"Comparison CSV output: {data_run_dir}")

    total_runs = len(pickup_cases) * len(EXPERIMENTS)
    print(
        "Equation (8) 6D pick-and-place comparison: "
        f"{len(pickup_cases)} pickup positions x {len(EXPERIMENTS)} modes "
        f"= {total_runs} runs"
    )
    print(
        f"Sweep option {arguments.sweep_option}: "
        + (
            "position-first (four modes at each pickup position)."
            if arguments.sweep_option == 1
            else "optimization-first (all pickup positions for each mode)."
        )
    )
    if arguments.record_video:
        print("Running headlessly; recording perspective and top views.")
    else:
        print("Close a viewer early to advance immediately to the next run.")

    run_cases = ordered_comparison_cases(
        pickup_cases,
        EXPERIMENTS,
        arguments.sweep_option,
    )
    for run_index, case in enumerate(run_cases, start=1):
        position_name, start_position, name, objective, enabled = case
        print("\n" + "=" * 72)
        print(
            f"Run {run_index}/{total_runs}: {position_name} / {name}; "
            f"pickup site_top_middle = {start_position} m"
        )
        print("=" * 72)
        experiment.main(
            objective=objective,
            characteristic_length=arguments.characteristic_length,
            enable_redundancy_optimization=enabled,
            hold_duration=arguments.hold_duration,
            record_data=arguments.record_data,
            output_csv=(
                data_run_dir / position_name / f"{name}.csv"
                if data_run_dir is not None
                else None
            ),
            show_collision_spheres=arguments.show_collision_spheres,
            enable_collision_penalty=(not arguments.disable_collision_penalty),
            optimization_gain=arguments.optimization_gain,
            maximum_joint_speed=arguments.max_joint_speed,
            start_position=start_position,
            start_euler_xyz=arguments.start_euler_xyz,
            intermediate_position=arguments.intermediate_position,
            intermediate_euler_xyz=arguments.intermediate_euler_xyz,
            goal_position=arguments.goal_position,
            goal_euler_xyz=arguments.goal_euler_xyz,
            start_to_intermediate_duration=(
                arguments.start_to_intermediate_duration
            ),
            intermediate_to_goal_duration=(
                arguments.intermediate_to_goal_duration
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
        )

    print(f"\nCompleted all {total_runs} 6D comparison runs.")


if __name__ == "__main__":
    run_cli(main)
