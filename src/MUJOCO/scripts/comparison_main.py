"""Deprecated shim: import from ``bimanual_redundancy.experiments.comparison_main`` instead."""

import warnings

warnings.warn(
    "MUJOCO.scripts.comparison_main is deprecated; import from bimanual_redundancy.experiments.comparison_main instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.experiments.comparison_main import *  # noqa: F401,F403

from bimanual_redundancy.experiments.comparison_main import main  # noqa: F401

if __name__ == "__main__":
    main()
