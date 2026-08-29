"""Built-in cooperative robot embodiments and their registry."""

from .dual_franka_panda import DUAL_FRANKA_PANDA
from .spec import ArmSpec, CollisionSpec, CooperativeSystemSpec, ObjectSpec
from .validation import SpecValidationError, validate_cooperative_system_spec


SYSTEM_SPECS = {DUAL_FRANKA_PANDA.identifier: DUAL_FRANKA_PANDA}


def get_cooperative_system_spec(identifier: str) -> CooperativeSystemSpec:
    try:
        return SYSTEM_SPECS[identifier]
    except KeyError as error:
        raise KeyError(
            f"unknown robot {identifier!r}; available robots: {tuple(SYSTEM_SPECS)}"
        ) from error


__all__ = [
    "ArmSpec",
    "CollisionSpec",
    "CooperativeSystemSpec",
    "ObjectSpec",
    "DUAL_FRANKA_PANDA",
    "SYSTEM_SPECS",
    "get_cooperative_system_spec",
    "SpecValidationError",
    "validate_cooperative_system_spec",
]
