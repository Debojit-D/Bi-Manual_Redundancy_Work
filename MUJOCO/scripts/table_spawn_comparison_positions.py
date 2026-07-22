"""Shared pickup positions and complete 6D poses for Equation (8) sweeps.

``TABLE_SPAWN_CASES`` remains the common pickup-only list used by every
experiment. ``SIX_D_TRAJECTORY_CASES`` stores position and extrinsic XYZ Euler
orientation for the pickup, intermediate, and final pose. Leave any
intermediate/final component as ``None`` until that 6D path has been designed;
incomplete entries are not swept.
"""

from math import pi


TABLE_SPAWN_CASES = (
    ("position_1", (0.30, 0.18, 0.28)),
    ("position_2", (0.60, 0.18, 0.28)),
    ("position_3", (0.30, 0.00, 0.28)),
    ("position_4", (0.60, 0.00, 0.28)),
    ("position_5", (0.30, -0.18, 0.28)),
    ("position_6", (0.60, -0.18, 0.28)),
)

# Each entry is:
# (name,
#  pickup_position, pickup_euler_xyz,
#  intermediate_position, intermediate_euler_xyz,
#  final_position, final_euler_xyz)
#
# Poses 1, 2, 5, and 6 are complete. Poses 3 and 4 remain placeholders.
SIX_D_TRAJECTORY_CASES = (
    (
        "pose_1",
        (0.30, 0.18, 0.28), (0.0, 0.0, pi / 2.0),
        (0.40, 0.00, 0.52), (0.0, 0.0, pi / 2.0 - 0.40),
        (0.25, -0.45, 0.269), (0.0, 0.0, pi / 2.0 - 0.80),
    ),
    (
        "pose_2", (0.60, 0.18, 0.28), (0.0, 0.0, pi / 2.0),
        (0.40, 0.00, 0.52), (0.0, 0.0, pi / 2.0 - 0.40),
        (0.25, -0.45, 0.269), (0.0, 0.0, pi / 2.0 - 0.80),
    ),
    (
        "pose_3", (0.30, 0.00, 0.28), (-pi / 2.0, 0.0, pi / 2.0),
        (0.30, 0.00, 0.52), (0.0, 0.0, pi / 2.0),
        (0.30, 0.00, 0.28), (0.0, 0.0, pi / 2.0),
    ),
    (
        "pose_4", (0.60, 0.00, 0.28), (pi / 2.0, 0.0, pi / 2.0),
        (0.40, 0.00, 0.52), (0.0, 0.0, pi / 2.0 - 0.40),
        (0.25, -0.45, 0.269), (0.0, 0.0, pi / 2.0 - 0.80),
    ),
    (
        "pose_5", (0.30, -0.18, 0.28), (0.0, 0.0, pi / 2.0),
        (0.40, 0.00, 0.52),(0.0, 0.0, pi / 2.0 + 0.40),
        (0.25, 0.45, 0.269),(0.0, 0.0, pi / 2.0 + 0.80),
    ),
    (
        "pose_6", (0.60, -0.18, 0.28), (0.0, 0.0, pi / 2.0),
        (0.40, 0.00, 0.52), (0.0, 0.0, pi / 2.0 + 0.40),
        (0.25, 0.45, 0.269), (0.0, 0.0, pi / 2.0 + 0.80),
    ),
)

SWEEP_OPTIONS = (1, 2)


def table_spawn_position_for_number(position_number):
    """Return xyz coordinates for a one-based comparison position number."""
    if not 1 <= position_number <= len(TABLE_SPAWN_CASES):
        raise ValueError(
            f"position_number must be between 1 and {len(TABLE_SPAWN_CASES)}"
        )
    return TABLE_SPAWN_CASES[position_number - 1][1]


def six_d_trajectory_case_for_number(pose_number):
    """Return the configured pickup/intermediate/final 6D pose tuple."""
    if not 1 <= pose_number <= len(SIX_D_TRAJECTORY_CASES):
        raise ValueError(
            "pose_number must be between 1 and "
            f"{len(SIX_D_TRAJECTORY_CASES)}"
        )
    return SIX_D_TRAJECTORY_CASES[pose_number - 1]


def ordered_comparison_cases(position_cases, experiments, sweep_option=1):
    """Return comparison cases in position-first or optimization-first order.

    Option 1 preserves the original ordering: all optimization modes are run
    at one position before advancing to the next position. Option 2 runs one
    optimization mode at every position before advancing to the next mode.
    """
    if sweep_option not in SWEEP_OPTIONS:
        raise ValueError(f"sweep_option must be one of {SWEEP_OPTIONS}")

    if sweep_option == 1:
        return tuple(
            (position_name, position, name, objective, enabled)
            for position_name, position in position_cases
            for name, objective, enabled in experiments
        )
    return tuple(
        (position_name, position, name, objective, enabled)
        for name, objective, enabled in experiments
        for position_name, position in position_cases
    )
