"""Deprecated shim: import from ``bimanual_redundancy.experiments.add_directional_force_indirect_to_batch`` instead."""

import warnings

warnings.warn(
    "MUJOCO.scripts.add_directional_force_indirect_to_batch is deprecated; import from bimanual_redundancy.experiments.add_directional_force_indirect_to_batch instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.experiments.add_directional_force_indirect_to_batch import *  # noqa: F401,F403

from bimanual_redundancy.experiments.add_directional_force_indirect_to_batch import main  # noqa: F401

if __name__ == "__main__":
    main()
