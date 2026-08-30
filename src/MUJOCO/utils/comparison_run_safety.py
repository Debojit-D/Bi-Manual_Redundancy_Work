"""Deprecated shim: import from ``bimanual_redundancy.simulation.comparison_run_safety`` instead."""

import warnings

warnings.warn(
    "MUJOCO.utils.comparison_run_safety is deprecated; import from bimanual_redundancy.simulation.comparison_run_safety instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.simulation.comparison_run_safety import *  # noqa: F401,F403
