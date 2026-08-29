"""Deprecated shim — import from ``bimanual_redundancy.plotting.publication_style`` instead."""

import warnings

warnings.warn(
    "MUJOCO.plotting_scripts.publication_style is deprecated; import from bimanual_redundancy.plotting.publication_style instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.plotting.publication_style import *  # noqa: F401,F403
