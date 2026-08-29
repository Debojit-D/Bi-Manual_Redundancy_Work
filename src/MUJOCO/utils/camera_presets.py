"""Deprecated shim — import from ``bimanual_redundancy.simulation.cameras`` instead."""

import warnings

warnings.warn(
    "MUJOCO.utils.camera_presets is deprecated; import from bimanual_redundancy.simulation.cameras instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.simulation.cameras import *  # noqa: F401,F403
