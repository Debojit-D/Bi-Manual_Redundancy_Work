"""Run the spatial Equation (8) pick-and-place task in all four modes.

Four independent MuJoCo scenes run sequentially: baseline, velocity
manipulability, force manipulability, and directional-force manipulability.
Every mode follows the same configurable start, waypoint, and goal poses.
Close a viewer early to advance immediately to the next mode.

Run from the repository root with the project virtual environment::

    # Run all four modes with the default poses and trajectory durations.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison

    # Change the final hold duration for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \
        --hold-duration 5

    # Record each mode and display the fitted collision spheres.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \
        --record-data --show-collision-spheres

    # Use a full overhead camera for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \
        --top-view

    # Run headlessly and record both camera views for every mode.
    .venv/bin/python -m \
        MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \
        --record-video

    # Compare the objectives without the soft collision penalty.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \
        --disable-collision-penalty

    # Override the common start, intermediate, and goal positions (metres).
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \
        --start-position 0.40 -0.15 0.269 \
        --intermediate-position 0.40 0.00 0.52 \
        --goal-position 0.20 0.45 0.269

    # Override the corresponding extrinsic XYZ orientations (radians).
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \
        --start-euler-xyz 0 0 1.5708 \
        --intermediate-euler-xyz 0 0 1.9708 \
        --goal-euler-xyz 0 0 2.3708

    # Change both path-segment durations and optimizer limits.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \
        --start-to-intermediate-duration 12 \
        --intermediate-to-goal-duration 12 \
        --optimization-gain 3000 --max-joint-speed 3

Options can be combined. Run with ``--help`` for the complete option list.
"""

import argparse
from datetime import datetime
from pathlib import Path

from MUJOCO.scripts import dual_franka_eq8_optimized_6d_pick_place as experiment
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


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--hold-duration",
        type=float,
        default=2.0,
        help="final hold time for each run in seconds (default: %(default)s)",
    )
    parser.add_argument("--record-data", action="store_true")
    parser.add_argument("--show-collision-spheres", action="store_true")
    parser.add_argument(
        "--top-view",
        action="store_true",
        help="use a full overhead camera for all four runs",
    )
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
        "--start-position",
        type=float,
        nargs=3,
        default=experiment.TABLE_START_POSITION,
        metavar=("X", "Y", "Z"),
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

    video_run_dir = None
    if arguments.record_video:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        video_run_dir = (
            arguments.video_output_dir.expanduser().resolve()
            / f"6d_pick_place_comparison_{timestamp}"
        )
        video_run_dir.mkdir(parents=True, exist_ok=False)
        print(f"Headless dual-view video output: {video_run_dir}")

    print("Equation (8) 6D pick-and-place four-mode comparison")
    if arguments.record_video:
        print("Running headlessly; recording perspective and top views.")
    else:
        print("Close a viewer early to advance immediately to the next mode.")
    for index, (name, objective, enabled) in enumerate(EXPERIMENTS, start=1):
        print("\n" + "=" * 72)
        print(f"Run {index}/{len(EXPERIMENTS)}: {name}")
        print("=" * 72)
        experiment.main(
            objective=objective,
            enable_redundancy_optimization=enabled,
            hold_duration=arguments.hold_duration,
            record_data=arguments.record_data,
            show_collision_spheres=arguments.show_collision_spheres,
            enable_collision_penalty=(
                not arguments.disable_collision_penalty
            ),
            optimization_gain=arguments.optimization_gain,
            maximum_joint_speed=arguments.max_joint_speed,
            start_position=arguments.start_position,
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
            video_output_dir=(
                video_run_dir / name if video_run_dir is not None else None
            ),
            video_width=arguments.video_width,
            video_height=arguments.video_height,
            video_fps=arguments.video_fps,
        )

    print("\nCompleted all four 6D pick-and-place comparison runs.")


if __name__ == "__main__":
    main()
