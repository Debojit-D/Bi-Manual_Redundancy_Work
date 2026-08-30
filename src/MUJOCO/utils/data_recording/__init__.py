"""Deprecated shim: import from ``bimanual_redundancy.simulation.recording`` instead."""

import warnings

warnings.warn(
    "MUJOCO.utils.data_recording is deprecated; import from bimanual_redundancy.simulation.recording instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.simulation.recording import (
    CSV_COLUMNS,
    Equation8CSVRecorder,

)

__all__ = [
    "CSV_COLUMNS",
    "Equation8CSVRecorder",

]
