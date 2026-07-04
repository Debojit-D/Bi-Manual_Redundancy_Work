"""Run the simple Equation (8) pick-and-place task in all four modes.

Four independent MuJoCo scenes run sequentially: baseline, velocity
manipulability, force manipulability, and directional-force manipulability.
Each scene performs the same lift-and-return trajectory and then holds for the
configured interval. Close a viewer early to advance to the next mode.

Run from the repository root with the project virtual environment::

    # Run all four modes and hold the returned pose for 2 seconds per mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison

    # Change the final hold duration for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --hold-duration 5

    # Record every control step to a separate timestamped CSV per mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --record-data

    # Display the fitted inter-arm collision spheres in every viewer.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --show-collision-spheres

    # Use a full overhead camera for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --top-view

    # Run headlessly and record both camera views for every mode.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --record-video

    # Compare the objectives without the soft collision penalty.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --disable-collision-penalty

    # Override the shared optimization gain and joint-speed limit.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_pick_place_comparison \
        --optimization-gain 3000 --max-joint-speed 3

Options can be combined. Run with ``--help`` for the complete option list.
"""

import argparse
from datetime import datetime
from pathlib import Path

from MUJOCO.scripts import dual_franka_eq8_optimized_pick_place as experiment
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
    parser.add_argument(
        "--record-data",
        action="store_true",
        help="record every mode to its own timestamped CSV",
    )
    parser.add_argument(
        "--show-collision-spheres",
        action="store_true",
        help="draw fitted collision spheres in every viewer",
    )
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

    video_run_dir = None
    if arguments.record_video:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        video_run_dir = (
            arguments.video_output_dir.expanduser().resolve()
            / f"pick_place_comparison_{timestamp}"
        )
        video_run_dir.mkdir(parents=True, exist_ok=False)
        print(f"Headless dual-view video output: {video_run_dir}")

    print("Equation (8) pick-and-place four-mode comparison")
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
            top_view=arguments.top_view,
            video_output_dir=(
                video_run_dir / name if video_run_dir is not None else None
            ),
            video_width=arguments.video_width,
            video_height=arguments.video_height,
            video_fps=arguments.video_fps,
        )

    print("\nCompleted all four pick-and-place comparison runs.")


if __name__ == "__main__":
    main()
