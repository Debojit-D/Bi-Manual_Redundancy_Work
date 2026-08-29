"""Deprecated shim — import from ``bimanual_redundancy.experiments.dual_franka_eq8_optimized_pick_place`` instead."""

import warnings

warnings.warn(
    "MUJOCO.scripts.dual_franka_eq8_optimized_pick_place is deprecated; import from bimanual_redundancy.experiments.dual_franka_eq8_optimized_pick_place instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.experiments.dual_franka_eq8_optimized_pick_place import *  # noqa: F401,F403

from bimanual_redundancy.experiments.dual_franka_eq8_optimized_pick_place import main  # noqa: F401

if __name__ == "__main__":
    main()
