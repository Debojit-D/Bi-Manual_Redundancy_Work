"""Deprecated shim: import from ``bimanual_redundancy.plotting.plot_directional_joint_angle_metrics`` instead."""

import warnings

warnings.warn(
    "MUJOCO.scripts.plot_directional_joint_angle_metrics is deprecated; import from bimanual_redundancy.plotting.plot_directional_joint_angle_metrics instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.plotting.plot_directional_joint_angle_metrics import *  # noqa: F401,F403

from bimanual_redundancy.plotting.plot_directional_joint_angle_metrics import main  # noqa: F401

if __name__ == "__main__":
    main()
