"""Deprecated shim — import from ``bimanual_redundancy.plotting.plot_directional_force_cross_evaluation`` instead."""

import warnings

warnings.warn(
    "MUJOCO.scripts.plot_directional_force_cross_evaluation is deprecated; import from bimanual_redundancy.plotting.plot_directional_force_cross_evaluation instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.plotting.plot_directional_force_cross_evaluation import *  # noqa: F401,F403

from bimanual_redundancy.plotting.plot_directional_force_cross_evaluation import main  # noqa: F401

if __name__ == "__main__":
    main()
