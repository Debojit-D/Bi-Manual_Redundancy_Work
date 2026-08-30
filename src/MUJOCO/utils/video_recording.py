"""Deprecated shim: import from ``bimanual_redundancy.simulation.recording.video`` instead."""

import warnings

warnings.warn(
    "MUJOCO.utils.video_recording is deprecated; import from bimanual_redundancy.simulation.recording.video instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.simulation.recording.video import *  # noqa: F401,F403
