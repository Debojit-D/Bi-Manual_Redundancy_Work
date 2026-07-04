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
"""

import argparse

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


def parse_arguments():
    parser = argparse.ArgumentParser(description=__doc__)
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

    print("Static Equation (8) four-mode comparison")
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
        )

    print("\nCompleted all four static comparison runs.")


if __name__ == "__main__":
    main()
