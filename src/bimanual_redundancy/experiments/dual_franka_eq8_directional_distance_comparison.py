"""Compare four directional-distance permutations at six table positions.

With sweep option 1 (the default), each table position runs these four
independent MuJoCo scenes:

1. ``(A A.T)^dagger`` + minimize distance (force alignment),
2. ``(A A.T)^dagger`` + maximize distance (force negative control),
3. ``A A.T`` + minimize distance (velocity alignment), and
4. ``A A.T`` + maximize distance (velocity avoidance).

Sweep option 2 runs each permutation at all six positions before advancing to
the next permutation. Both options produce 24 runs::

    # Option 1: run all four permutations at each position.
    .venv/bin/python -m \
        bimanual_redundancy.experiments.dual_franka_eq8_directional_distance_comparison \
        --sweep-option 1

    # Option 2: run each permutation across all six positions.
    .venv/bin/python -m \
        bimanual_redundancy.experiments.dual_franka_eq8_directional_distance_comparison \
        --sweep-option 2

    # Select a front or overhead interactive camera.
    .venv/bin/python -m \
        bimanual_redundancy.experiments.dual_franka_eq8_directional_distance_comparison \
        --front-view
    .venv/bin/python -m \
        bimanual_redundancy.experiments.dual_franka_eq8_directional_distance_comparison \
        --top-view

Convergence-based stopping is the default. Supplying ``--duration`` runs each
case for the same fixed interval instead. Each completed static case ends with
the grippers closed and does not disengage; close any viewer to advance early.
Press Ctrl+C to close the active viewer and stop the entire sweep cleanly.
"""

import argparse

import numpy as np
from loop_rate_limiters import RateLimiter

from bimanual_redundancy.experiments import dual_franka_eq8_static_optimization as static_setup
from bimanual_redundancy.experiments.table_spawn_comparison_positions import (
    SWEEP_OPTIONS,
    TABLE_SPAWN_CASES,
    ordered_comparison_cases,
)
from bimanual_redundancy.simulation.cli import add_camera_view_arguments, run_cli
from bimanual_redundancy.simulation.recording import Equation8CSVRecorder
from bimanual_redundancy.core import (
    DirectionalDistanceCase,
    DirectionalDistancePermutationOptimizer,
    Equation8Controller,
)
from bimanual_redundancy.simulation import DualFrankaMuJoCoScene


DEFAULT_CONVERGENCE_SPEED = 0.005
DEFAULT_CONVERGENCE_HOLD = 0.5
DEFAULT_MINIMUM_RUN_TIME = 1.0

CASES = (
    DirectionalDistanceCase.FORCE_MINIMIZE,
    DirectionalDistanceCase.FORCE_MAXIMIZE,
    DirectionalDistanceCase.VELOCITY_MINIMIZE,
    DirectionalDistanceCase.VELOCITY_MAXIMIZE,
)
EXPERIMENTS = tuple((case.value, case, True) for case in CASES)

CASE_DESCRIPTIONS = {
    DirectionalDistanceCase.FORCE_MINIMIZE: (
        "C=(A A.T)^dagger; minimize D; force-capability alignment"
    ),
    DirectionalDistanceCase.FORCE_MAXIMIZE: (
        "C=(A A.T)^dagger; maximize D; force negative control"
    ),
    DirectionalDistanceCase.VELOCITY_MINIMIZE: (
        "C=A A.T; minimize D; velocity-capability alignment"
    ),
    DirectionalDistanceCase.VELOCITY_MAXIMIZE: (
        "C=A A.T; maximize D; velocity-direction avoidance"
    ),
}


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
            "run order: 1 = four permutations at each position; "
            "2 = all six positions for each permutation (default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--duration",
        type=float,
        help=(
            "fixed duration of each case in seconds; overrides convergence "
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
        help="time below threshold required for convergence (default: %(default)s)",
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
        help="record every position/permutation run to a separately named CSV",
    )
    parser.add_argument(
        "--show-collision-spheres",
        action="store_true",
        help="draw fitted collision spheres in every viewer",
    )
    add_camera_view_arguments(parser, scope="comparison runs")
    parser.add_argument(
        "--disable-collision-penalty",
        action="store_true",
        help="disable the shared soft collision term in all comparison runs",
    )
    parser.add_argument(
        "--optimization-gain",
        type=float,
        default=static_setup.OPTIMIZATION_GAIN,
        help="signed-score gradient gain (default: %(default)s)",
    )
    parser.add_argument(
        "--max-joint-speed",
        type=float,
        default=static_setup.MAXIMUM_OPTIMIZATION_JOINT_SPEED,
        help="symmetric optimization speed bound in rad/s (default: %(default)s)",
    )
    parser.add_argument(
        "--characteristic-length",
        type=float,
        default=None,
        help=(
            "manual spatial characteristic length in metres; by default "
            "compute it once from the rigid object contact sites"
        ),
    )
    return parser.parse_args()


def validate_arguments(arguments):
    if arguments.duration is not None and arguments.duration <= 0.0:
        raise ValueError("--duration must be greater than zero")
    if arguments.convergence_speed < 0.0:
        raise ValueError("--convergence-speed cannot be negative")
    if arguments.convergence_hold <= 0.0:
        raise ValueError("--convergence-hold must be greater than zero")
    if arguments.minimum_run_time < 0.0:
        raise ValueError("--minimum-run-time cannot be negative")
    if arguments.optimization_gain <= 0.0:
        raise ValueError("--optimization-gain must be greater than zero")
    if arguments.max_joint_speed <= 0.0:
        raise ValueError("--max-joint-speed must be greater than zero")


def build_experiment(case, arguments, table_position):
    """Build one independent scene, controller, and permutation optimizer."""
    scene = DualFrankaMuJoCoScene(
        control_hz=static_setup.CONTROL_HZ,
        left_arm_base_position=static_setup.LEFT_ARM_SPAWN_POSITION,
        right_arm_base_position=static_setup.RIGHT_ARM_SPAWN_POSITION,
        left_arm_base_euler_xyz_degrees=(
            static_setup.LEFT_ARM_SPAWN_EULER_XYZ_DEGREES
        ),
        right_arm_base_euler_xyz_degrees=(
            static_setup.RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES
        ),
        show_mocap_targets=static_setup.SHOW_MOCAP_TARGETS,
        enable_bias_compensation=static_setup.ENABLE_ARM_BIAS_COMPENSATION,
    )
    scene.set_table_reference_pose(table_position)
    kinematics = scene.make_kinematics()
    # Resolve once from the initialized rigid grasp; keep it fixed for the run.
    (
        selected_characteristic_length,
        computed_characteristic_length,
        contact_midpoint,
        midpoint_reference_distance,
    ) = kinematics.resolve_characteristic_length(
        scene.data,
        arguments.characteristic_length,
    )
    print(
        "Computed grasp characteristic length: "
        f"{computed_characteristic_length:.9f} m"
    )
    print(f"Contact midpoint: {contact_midpoint} m")
    print(
        "Contact midpoint to object reference distance: "
        f"{midpoint_reference_distance:.9g} m"
    )
    print(
        "Characteristic length used: "
        f"{selected_characteristic_length:.9f} m "
        "(automatic)"
        if arguments.characteristic_length is None
        else (
            "Characteristic length used: "
            f"{selected_characteristic_length:.9f} m (manual override)"
        )
    )

    joint_position_lower = scene.joint_position_limits[:, 0]
    joint_position_upper = scene.joint_position_limits[:, 1]
    equation_8 = Equation8Controller(
        kinematics,
        control_dt=scene.control_dt,
        feedback_gain=static_setup.K_P,
        grasp_feedback_gain=static_setup.GRASP_K_P,
        joint_position_lower=joint_position_lower,
        joint_position_upper=joint_position_upper,
        joint_velocity_lower=-arguments.max_joint_speed,
        joint_velocity_upper=arguments.max_joint_speed,
        joint_limit_margin=static_setup.JOINT_LIMIT_MARGIN,
        joint_limit_stop_distance=static_setup.JOINT_LIMIT_STOP_DISTANCE,
        joint_limit_slow_distance=static_setup.JOINT_LIMIT_SLOW_DISTANCE,
    )
    optimizer = DirectionalDistancePermutationOptimizer(
        kinematics,
        scene.arm_qpos,
        case=case,
        gain=arguments.optimization_gain,
        finite_difference_step=static_setup.FINITE_DIFFERENCE_STEP,
        maximum_joint_speed=arguments.max_joint_speed,
        desired_wrench_direction=static_setup.DESIRED_WRENCH_DIRECTION,
        characteristic_length=selected_characteristic_length,
        enable_collision_penalty=not arguments.disable_collision_penalty,
        collision_weight=static_setup.COLLISION_WEIGHT,
        collision_safety_margin=static_setup.COLLISION_SAFETY_MARGIN,
        collision_proximity_scale=static_setup.COLLISION_PROXIMITY_SCALE,
        collision_version="version2",
        collision_sphere_model_path=(
            static_setup.COLLISION_SPHERE_MODEL_PATH
        ),
    )
    return scene, kinematics, equation_8, optimizer


def run_case(case, arguments, position_name, table_position):
    scene, kinematics, equation_8, optimizer = build_experiment(
        case,
        arguments,
        table_position,
    )
    print(f"Table position: {position_name} = {table_position} m")
    print(f"Case: {case.value}")
    print(f"Definition: {CASE_DESCRIPTIONS[case]}")
    print(
        f"Collision penalty: {optimizer.enable_collision_penalty}, "
        f"weight={optimizer.collision_weight:g}, "
        f"margin={optimizer.collision_safety_margin:.3f} m"
    )

    rate = RateLimiter(frequency=static_setup.CONTROL_HZ, warn=False)
    recorder = None
    if arguments.record_data:
        recorder = Equation8CSVRecorder(
            scene,
            kinematics,
            equation_8,
            optimizer,
            experiment_name=(
                "dual_franka_eq8_directional_distance_"
                f"{position_name}_{case.value}_fitted_spheres"
            ),
            optimization_mode=case.value,
        )
        print(f"Recording data to: {recorder.output_path}")

    try:
        with scene.launch_viewer() as viewer:
            scene.configure_viewer_camera(
                viewer,
                top_view=arguments.top_view,
                front_view=arguments.front_view,
            )
            scene.settle(viewer, rate)
            scene.run_grasp_approach(viewer, rate)
            print("Closing both grippers...")
            scene.close_grippers(viewer, rate)
            equation_8.capture_grasp_reference(scene.data)
            static_setup.run_static_optimization(
                scene,
                kinematics,
                equation_8,
                optimizer,
                viewer,
                rate,
                duration=arguments.duration,
                recorder=recorder,
                show_collision_spheres=arguments.show_collision_spheres,
                enable_redundancy_optimization=True,
                convergence_speed_threshold=(
                    arguments.convergence_speed
                    if arguments.duration is None
                    else None
                ),
                convergence_hold_duration=arguments.convergence_hold,
                minimum_convergence_time=arguments.minimum_run_time,
            )
    finally:
        if recorder is not None:
            recorder.close()
            print(
                f"Saved {recorder.rows_written} rows to: "
                f"{recorder.output_path}"
            )


def main():
    arguments = parse_arguments()
    validate_arguments(arguments)
    print("Directional-distance 2x2 permutation comparison")
    total_runs = len(TABLE_SPAWN_CASES) * len(CASES)
    print(
        f"{len(TABLE_SPAWN_CASES)} positions x {len(CASES)} permutations "
        f"= {total_runs} runs"
    )
    print(
        f"Sweep option {arguments.sweep_option}: "
        + (
            "position-first (four permutations at each position)."
            if arguments.sweep_option == 1
            else "optimization-first (all positions for each permutation)."
        )
    )
    print("Close a viewer early to advance to the next case.")
    if arguments.duration is None:
        print(
            "Stopping rule: null-space speed <= "
            f"{arguments.convergence_speed:g} rad/s for "
            f"{arguments.convergence_hold:g} s."
        )
    else:
        print(f"Stopping rule: fixed {arguments.duration:g} s per case.")

    run_cases = ordered_comparison_cases(
        TABLE_SPAWN_CASES,
        EXPERIMENTS,
        arguments.sweep_option,
    )
    for index, values in enumerate(run_cases, start=1):
        position_name, table_position, _, case, _ = values
        print("\n" + "=" * 76)
        print(f"Run {index}/{total_runs}: {position_name} / {case.value}")
        print("=" * 76)
        run_case(case, arguments, position_name, table_position)
    print(f"\nCompleted all {total_runs} directional-distance runs.")


if __name__ == "__main__":
    run_cli(main)
