"""Deprecated shim: import from ``bimanual_redundancy.core`` instead."""

import warnings

warnings.warn(
    "MUJOCO.utils.grasping_kinematics is deprecated; import from bimanual_redundancy.core instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.core import (
    CooperativeManipulationKinematics,

)

__all__ = [
    "CooperativeManipulationKinematics",

]
