"""Deprecated shim: import from ``bimanual_redundancy.simulation.grasp_safety`` instead."""

import warnings

warnings.warn(
    "MUJOCO.utils.grasp_safety is deprecated; import from bimanual_redundancy.simulation.grasp_safety instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.simulation.grasp_safety import *  # noqa: F401,F403
