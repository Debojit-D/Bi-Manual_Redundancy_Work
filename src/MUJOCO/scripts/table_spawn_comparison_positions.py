"""Deprecated shim: import from ``bimanual_redundancy.experiments.table_spawn_comparison_positions`` instead."""

import warnings

warnings.warn(
    "MUJOCO.scripts.table_spawn_comparison_positions is deprecated; import from bimanual_redundancy.experiments.table_spawn_comparison_positions instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.experiments.table_spawn_comparison_positions import *  # noqa: F401,F403
