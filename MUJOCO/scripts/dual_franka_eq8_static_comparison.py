"""Run baseline and all three static Equation (8) optimization modes.

Four independent MuJoCo scenes are run in this order:

1. baseline (no null-space optimization),
2. velocity manipulability,
3. force manipulability, and
4. directional-force manipulability.

Each viewer remains visible until the null-space motion has converged, then
closes before the next mode starts. A run is converged when its maximum
null-space joint speed stays below the selected threshold for a sustained
window. Supplying ``--duration`` instead runs every mode for the same fixed
number of seconds and disables convergence-based stopping.

Record both camera views for all four modes without opening a viewer::

    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_static_comparison \
        --record-video
"""

import argparse
from datetime import datetime
from pathlib import Path

from MUJOCO.scripts.dual_franka_eq8_static_optimization import (
    main as run_static_experiment,
)
from MUJOCO.utils.redundancy_optimization import ManipulabilityObjective


DEFAULT_CONVERGENCE_SPEED = 0.005
DEFAULT_CONVERGENCE_HOLD = 0.5
DEFAULT_MINIMUM_RUN_TIME = 1.0
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
        help="record each mode to its own timestamped CSV",
    )
    parser.add_argument(
        "--show-collision-spheres",
        action="store_true",
        help="draw the fitted collision spheres in every viewer",
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
        help="disable the soft collision term for all optimized modes",
    )
    return parser.parse_args()


def main():
    arguments = parse_arguments()
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

    video_run_dir = None
    if arguments.record_video:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        video_run_dir = (
            arguments.video_output_dir.expanduser().resolve()
            / f"static_comparison_{timestamp}"
        )
        video_run_dir.mkdir(parents=True, exist_ok=False)
        print(f"Headless dual-view video output: {video_run_dir}")

    print("Static Equation (8) four-mode comparison")
    if arguments.record_video:
        print("Running headlessly; recording perspective and top views.")
    else:
        print("Close a viewer early to advance immediately to the next mode.")
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
    for index, (name, objective, enabled) in enumerate(EXPERIMENTS, start=1):
        print("\n" + "=" * 72)
        print(f"Run {index}/{len(EXPERIMENTS)}: {name}")
        print("=" * 72)
        run_static_experiment(
            objective=objective,
            enable_redundancy_optimization=enabled,
            duration=arguments.duration,
            convergence_speed_threshold=(
                arguments.convergence_speed
                if arguments.duration is None
                else None
            ),
            convergence_hold_duration=arguments.convergence_hold,
            minimum_convergence_time=arguments.minimum_run_time,
            record_data=arguments.record_data,
            show_collision_spheres=arguments.show_collision_spheres,
            enable_collision_penalty=not arguments.disable_collision_penalty,
            top_view=arguments.top_view,
            video_output_dir=(
                video_run_dir / name if video_run_dir is not None else None
            ),
            video_width=arguments.video_width,
            video_height=arguments.video_height,
            video_fps=arguments.video_fps,
        )

    print("\nCompleted all four static comparison runs.")


if __name__ == "__main__":
    main()
