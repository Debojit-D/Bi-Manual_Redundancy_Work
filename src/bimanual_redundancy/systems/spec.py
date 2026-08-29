"""Small embodiment specification for a cooperative two-arm system."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


Limit = tuple[float, float]


@dataclass(frozen=True)
class ArmSpec:
    """Backend names and physical limits for one controlled arm."""

    joint_names: tuple[str, ...]
    actuator_names: tuple[str, ...]
    base_body: str
    hand_site: str
    target_body: str
    gripper_actuator: str
    joint_position_limits: tuple[Limit, ...]
    joint_velocity_limits: tuple[Limit, ...]


@dataclass(frozen=True)
class ObjectSpec:
    """Object frame, free joint, and rigid contact frames."""

    body: str
    joint: str
    reference_site: str
    contact_sites: tuple[str, str]


@dataclass(frozen=True)
class CollisionSpec:
    """Optional MuJoCo collision resources and body group declarations."""

    sphere_resource: Path | None
    sphere_geom_prefix: str
    left_root_body: str
    right_root_body: str
    excluded_bodies: tuple[str, ...]
    left_inter_arm_bodies: tuple[str, ...]
    right_inter_arm_bodies: tuple[str, ...]
    left_table_bodies: tuple[str, ...]
    right_table_bodies: tuple[str, ...]
    left_terminal_bodies: tuple[str, ...] = ()
    right_terminal_bodies: tuple[str, ...] = ()
    left_gripper_bodies: tuple[str, ...] = ()
    right_gripper_bodies: tuple[str, ...] = ()
    table_root_body: str | None = None
    table_geom_names: tuple[str, ...] = ()
    table_top_half_size: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class CooperativeSystemSpec:
    """Everything robot-specific needed by the current dual-arm backend."""

    identifier: str
    backend: str
    model_path: Path
    left_arm: ArmSpec
    right_arm: ArmSpec
    object: ObjectSpec
    home_keyframe: str
    collision: CollisionSpec | None = None
    mounting_platform_geom: str | None = None

    @property
    def controlled_joint_names(self) -> tuple[str, ...]:
        return self.left_arm.joint_names + self.right_arm.joint_names

    @property
    def controlled_actuator_names(self) -> tuple[str, ...]:
        return self.left_arm.actuator_names + self.right_arm.actuator_names

    @property
    def joint_position_limits(self) -> tuple[Limit, ...]:
        return self.left_arm.joint_position_limits + self.right_arm.joint_position_limits

    @property
    def joint_velocity_limits(self) -> tuple[Limit, ...]:
        return self.left_arm.joint_velocity_limits + self.right_arm.joint_velocity_limits
