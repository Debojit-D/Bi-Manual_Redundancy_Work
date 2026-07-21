"""Shared table-reference positions for Equation (8) comparison sweeps."""


TABLE_SPAWN_CASES = (
    ("position_1", (0.30, 0.15, 0.28)),
    ("position_2", (0.60, 0.15, 0.28)),
    ("position_3", (0.30, 0.00, 0.28)),
    ("position_4", (0.60, 0.00, 0.28)),
    ("position_5", (0.30, -0.15, 0.28)),
    ("position_6", (0.60, -0.15, 0.28)),
)


def table_spawn_position_for_number(position_number):
    """Return xyz coordinates for a one-based comparison position number."""
    if not 1 <= position_number <= len(TABLE_SPAWN_CASES):
        raise ValueError(
            f"position_number must be between 1 and {len(TABLE_SPAWN_CASES)}"
        )
    return TABLE_SPAWN_CASES[position_number - 1][1]
