"""Deprecated shim: import from ``bimanual_redundancy.experiments.dual_franka_eq8_static_comparison`` instead."""

import warnings

warnings.warn(
    "MUJOCO.scripts.dual_franka_eq8_static_comparison is deprecated; import from bimanual_redundancy.experiments.dual_franka_eq8_static_comparison instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.experiments.dual_franka_eq8_static_comparison import *  # noqa: F401,F403

from bimanual_redundancy.experiments.dual_franka_eq8_static_comparison import main  # noqa: F401

if __name__ == "__main__":
    main()
