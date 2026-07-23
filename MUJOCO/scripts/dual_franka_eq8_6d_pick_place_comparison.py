"""Run spatial Equation (8) comparisons with per-pickup 6D trajectories.

Only complete entries in ``SIX_D_TRAJECTORY_CASES`` are swept. Every entry
stores position and extrinsic XYZ Euler orientation for its pickup,
intermediate, and final pose. CLI overrides can temporarily complete blank
entries without editing the table.

Run from the repository root with the project virtual environment::

    # Option 1: run all four modes at each configured 6D trajectory.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --sweep-option 1

    # Option 2: run each mode across all configured 6D trajectories.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --sweep-option 2

    # Run pose 1 (its complete position/orientation path) in all four modes.
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --pose 1

    # Override pose 2 from the CLI if needed (orientations are radians).
    .venv/bin/python -m MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --pose 2 \\
        --intermediate-position 0.40 0.00 0.52 \\
        --intermediate-euler-xyz 0 0 1.9708 \\
        --goal-position 0.20 0.45 0.269 \\
        --goal-euler-xyz 0 0 2.3708

    # Position map [x, y, z] metres (pickup/start position only):
    # 1=(0.30, 0.18, 0.28), 2=(0.60, 0.18, 0.28)
    # 3=(0.30, 0.00, 0.28), 4=(0.60, 0.00, 0.28)
    # 5=(0.30,-0.18, 0.28), 6=(0.60,-0.18, 0.28)

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

    # Record one default perspective video: pose 1, velocity mode only.
    .venv/bin/python -m \\
        MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --pose 1 --mode velocity --record-video \\
        --video-view perspective

    # Record perspective, top, and front videos for one selected run.
    .venv/bin/python -m \\
        MUJOCO.scripts.dual_franka_eq8_6d_pick_place_comparison \\
        --pose 1 --mode velocity --record-video --video-view all

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
    SIX_D_TRAJECTORY_CASES,
    SWEEP_OPTIONS,
    ordered_comparison_cases,
    six_d_trajectory_case_for_number,
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
            "run order: 1 = four modes at each 6D pose path; "
            "2 = all 6D pose paths for each mode (default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--hold-duration",
        type=float,
        default=2.0,
        help="final hold time for each run in seconds (default: %(default)s)",
    )
    parser.add_argument(
        "--mode",
        choices=EXPERIMENT_MODES,
        default="all",
        help="run all optimization modes or one selected mode",
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
        help="run headlessly and record the selected video view(s)",
    )
    parser.add_argument(
        "--video-view",
        choices=VIDEO_VIEW_CHOICES,
        default="both",
        help=(
            "record all, the legacy perspective/top pair, or one camera "
            "when using --record-video "
            "(default: %(default)s)"
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
        "--pose",
        "--position",
        dest="pose",
        type=int,
        choices=range(1, len(SIX_D_TRAJECTORY_CASES) + 1),
        help=(
            "run only predefined complete 6D pose path 1 through 6; "
            "--position is retained as a compatibility alias"
        ),
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
        default=None,
        metavar=("RX", "RY", "RZ"),
    )
    parser.add_argument(
        "--intermediate-position",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help=(
            "override the configured intermediate position for every selected "
            "pose case"
        ),
    )
    parser.add_argument(
        "--intermediate-euler-xyz",
        type=float,
        nargs=3,
        default=None,
        metavar=("RX", "RY", "RZ"),
    )
    parser.add_argument(
        "--goal-position",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help=(
            "override the configured final position for every selected "
            "pose case"
        ),
    )
    parser.add_argument(
        "--goal-euler-xyz",
        type=float,
        nargs=3,
        default=None,
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


def resolve_trajectory_cases(
    pose_cases,
    start_position_override=None,
    start_euler_xyz_override=None,
    intermediate_position_override=None,
    intermediate_euler_xyz_override=None,
    goal_position_override=None,
    goal_euler_xyz_override=None,
):
    """Apply CLI overrides and return complete position/orientation paths."""
    overrides = (
        start_position_override,
        start_euler_xyz_override,
        intermediate_position_override,
        intermediate_euler_xyz_override,
        goal_position_override,
        goal_euler_xyz_override,
    )
    resolved = []
    incomplete = []
    for configured in pose_cases:
        pose_name = configured[0]
        values = tuple(
            override if override is not None else configured_value
            for override, configured_value in zip(overrides, configured[1:])
        )
        if any(value is None for value in values):
            incomplete.append(pose_name)
            continue
        resolved.append((pose_name, values))
    return tuple(resolved), tuple(incomplete)


def main():
    arguments = parse_arguments()
    validate_arguments(arguments)
    selected_experiments = experiments_for_mode(arguments.mode)
    selected_video_views = video_views_for_choice(arguments.video_view)

    if arguments.pose is not None:
        pose_cases = (six_d_trajectory_case_for_number(arguments.pose),)
    elif arguments.start_position is not None:
        pose_cases = (
            (
                "custom_pose",
                experiment.TABLE_START_POSITION,
                experiment.TABLE_START_EULER_XYZ,
                experiment.TABLE_INTERMEDIATE_POSITION,
                experiment.TABLE_INTERMEDIATE_EULER_XYZ,
                experiment.TABLE_GOAL_POSITION,
                experiment.TABLE_GOAL_EULER_XYZ,
            ),
        )
    else:
        pose_cases = SIX_D_TRAJECTORY_CASES

    trajectory_cases, incomplete_cases = resolve_trajectory_cases(
        pose_cases,
        start_position_override=arguments.start_position,
        start_euler_xyz_override=arguments.start_euler_xyz,
        intermediate_position_override=arguments.intermediate_position,
        intermediate_euler_xyz_override=arguments.intermediate_euler_xyz,
        goal_position_override=arguments.goal_position,
        goal_euler_xyz_override=arguments.goal_euler_xyz,
    )
    if incomplete_cases and arguments.pose is not None:
        raise ValueError(
            f"{incomplete_cases[0]} has blank intermediate/final pose data. "
            "Fill SIX_D_TRAJECTORY_CASES or pass each missing position and "
            "orientation option."
        )
    if incomplete_cases:
        print(
            "Skipping 6D cases with blank intermediate/final pose data: "
            + ", ".join(incomplete_cases)
        )
    if not trajectory_cases:
        raise ValueError("No complete 6D trajectory cases were selected")

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    video_run_dir = None
    if arguments.record_video:
        video_run_dir = (
            arguments.video_output_dir.expanduser().resolve()
            / f"6d_pick_place_comparison_{timestamp}"
        )
        video_run_dir.mkdir(parents=True, exist_ok=False)
        print(f"Headless video output: {video_run_dir}")

    data_run_dir = None
    if arguments.record_data:
        data_run_dir = (
            arguments.data_output_dir.expanduser().resolve()
            / f"6d_pick_place_comparison_{timestamp}"
        )
        data_run_dir.mkdir(parents=True, exist_ok=False)
        print(f"Comparison CSV output: {data_run_dir}")

    total_runs = len(trajectory_cases) * len(selected_experiments)
    print(
        "Equation (8) 6D pick-and-place comparison: "
        f"{len(trajectory_cases)} configured trajectories x "
        f"{len(selected_experiments)} mode(s) "
        f"= {total_runs} runs"
    )
    print(
        f"Sweep option {arguments.sweep_option}: "
        + (
            "pose-first "
            f"({len(selected_experiments)} mode(s) at each 6D pose path)."
            if arguments.sweep_option == 1
            else "optimization-first (all 6D pose paths for each mode)."
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
        trajectory_cases,
        selected_experiments,
        arguments.sweep_option,
    )
    failed_runs = []
    for run_index, case in enumerate(run_cases, start=1):
        pose_name, poses, name, objective, enabled = case
        (
            start_position,
            start_euler_xyz,
            intermediate_position,
            intermediate_euler_xyz,
            goal_position,
            goal_euler_xyz,
        ) = poses
        print("\n" + "=" * 72)
        print(
            f"Run {run_index}/{total_runs}: {pose_name} / {name}; "
            f"pickup site_top_middle = {start_position} m"
        )
        print("=" * 72)
        run_label = f"{pose_name} / {name}"
        try:
            experiment.main(
                objective=objective,
                characteristic_length=arguments.characteristic_length,
                enable_redundancy_optimization=enabled,
                hold_duration=arguments.hold_duration,
                record_data=arguments.record_data,
                output_csv=(
                    data_run_dir / pose_name / f"{name}.csv"
                    if data_run_dir is not None
                    else None
                ),
                show_collision_spheres=arguments.show_collision_spheres,
                enable_collision_penalty=(
                    not arguments.disable_collision_penalty
                ),
                optimization_gain=arguments.optimization_gain,
                maximum_joint_speed=arguments.max_joint_speed,
                start_position=start_position,
                start_euler_xyz=start_euler_xyz,
                intermediate_position=intermediate_position,
                intermediate_euler_xyz=intermediate_euler_xyz,
                goal_position=goal_position,
                goal_euler_xyz=goal_euler_xyz,
                start_to_intermediate_duration=(
                    arguments.start_to_intermediate_duration
                ),
                intermediate_to_goal_duration=(
                    arguments.intermediate_to_goal_duration
                ),
                top_view=arguments.top_view,
                front_view=arguments.front_view,
                video_output_dir=(
                    video_run_dir / pose_name / name
                    if video_run_dir is not None
                    else None
                ),
                video_width=arguments.video_width,
                video_height=arguments.video_height,
                video_fps=arguments.video_fps,
                video_views=selected_video_views,
                video_encoder=arguments.video_encoder,
                video_nvenc_view_limit=arguments.nvenc_max_views,
                # The reoriented pose 4 uses the nominal contact-site frame.
                use_alternate_grasp_orientation=False,
            )
        except Exception as error:
            failed_runs.append(run_label)
            print_run_failure(run_label, error)

    print_sweep_summary(total_runs, failed_runs)


if __name__ == "__main__":
    run_cli(main)
