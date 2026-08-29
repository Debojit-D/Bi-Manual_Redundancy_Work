"""Deprecated shim — import from ``bimanual_redundancy.simulation.cli`` instead."""

import warnings

warnings.warn(
    "MUJOCO.utils.cli is deprecated; import from bimanual_redundancy.simulation.cli instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.simulation.cli import *  # noqa: F401,F403
