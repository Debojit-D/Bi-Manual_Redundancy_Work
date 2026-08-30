"""Deprecated shim: import from ``bimanual_redundancy.simulation.control_timing`` instead."""

import warnings

warnings.warn(
    "MUJOCO.utils.control_timing is deprecated; import from bimanual_redundancy.simulation.control_timing instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.simulation.control_timing import *  # noqa: F401,F403
