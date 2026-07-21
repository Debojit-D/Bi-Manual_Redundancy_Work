"""Shared pickup/start positions for Equation (8) comparison sweeps.

Only pickup positions belong here. Intermediate and final placement poses are
experiment-specific and remain configurable through their CLI arguments.
"""


TABLE_SPAWN_CASES = (
    ("position_1", (0.30, 0.20, 0.28)),
    ("position_2", (0.60, 0.20, 0.28)),
    ("position_3", (0.30, 0.00, 0.28)),
    ("position_4", (0.60, 0.00, 0.28)),
    ("position_5", (0.30, -0.20, 0.28)),
    ("position_6", (0.60, -0.20, 0.28)),
)

SWEEP_OPTIONS = (1, 2)


def table_spawn_position_for_number(position_number):
    """Return xyz coordinates for a one-based comparison position number."""
    if not 1 <= position_number <= len(TABLE_SPAWN_CASES):
        raise ValueError(
            f"position_number must be between 1 and {len(TABLE_SPAWN_CASES)}"
        )
    return TABLE_SPAWN_CASES[position_number - 1][1]


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
