"""Deprecated shim — import from ``bimanual_redundancy.plotting.equation8_plot_style`` instead."""

import warnings

warnings.warn(
    "MUJOCO.plotting_scripts.equation8_plot_style is deprecated; import from bimanual_redundancy.plotting.equation8_plot_style instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.plotting.equation8_plot_style import *  # noqa: F401,F403
