"""Deprecated shim: import from ``bimanual_redundancy.simulation`` instead."""

import warnings

warnings.warn(
    "MUJOCO.utils.scene_builder is deprecated; import from bimanual_redundancy.simulation instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.simulation import (
    DualFrankaMuJoCoScene,

)

__all__ = [
    "DualFrankaMuJoCoScene",

]
