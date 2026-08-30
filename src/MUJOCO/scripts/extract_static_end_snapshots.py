"""Deprecated shim: import from ``bimanual_redundancy.experiments.extract_static_end_snapshots`` instead."""

import warnings

warnings.warn(
    "MUJOCO.scripts.extract_static_end_snapshots is deprecated; import from bimanual_redundancy.experiments.extract_static_end_snapshots instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.experiments.extract_static_end_snapshots import *  # noqa: F401,F403

from bimanual_redundancy.experiments.extract_static_end_snapshots import main  # noqa: F401

if __name__ == "__main__":
    main()
