"""Deprecated shim — import from ``bimanual_redundancy.experiments.load_dual_franka_robots_only`` instead."""

import warnings

warnings.warn(
    "MUJOCO.scripts.load_dual_franka_robots_only is deprecated; import from bimanual_redundancy.experiments.load_dual_franka_robots_only instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.experiments.load_dual_franka_robots_only import *  # noqa: F401,F403

from bimanual_redundancy.experiments.load_dual_franka_robots_only import main  # noqa: F401

if __name__ == "__main__":
    main()
