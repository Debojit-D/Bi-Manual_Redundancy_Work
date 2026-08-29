"""MuJoCo-backed validation for cooperative-system specifications."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np

from .spec import CooperativeSystemSpec


class SpecValidationError(ValueError):
    """A specification does not resolve against its declared backend model."""


def _resolve(model, kind, name, label, errors):
    identifier = mujoco.mj_name2id(model, kind, name)
    if identifier < 0:
        errors.append(f"{label} {name!r} does not resolve")
    return identifier


def validate_cooperative_system_spec(spec: CooperativeSystemSpec) -> dict:
    """Validate names, mappings, limits, object frames, and collision assets."""
    errors: list[str] = []
    if spec.backend != "mujoco":
        raise SpecValidationError(
            f"{spec.identifier}: unsupported backend {spec.backend!r}; expected 'mujoco'"
        )
    model_path = Path(spec.model_path)
    if not model_path.is_file():
        raise SpecValidationError(f"model does not exist: {model_path}")
    try:
        model = mujoco.MjModel.from_xml_path(str(model_path))
    except Exception as error:
        raise SpecValidationError(f"cannot load model {model_path}: {error}") from error

    joint_names = spec.controlled_joint_names
    if len(set(joint_names)) != len(joint_names):
        errors.append("controlled joint names must be unique")
    qpos_indices = []
    dof_indices = []
    joint_ids = []
    for name in joint_names:
        joint_id = _resolve(model, mujoco.mjtObj.mjOBJ_JOINT, name, "joint", errors)
        if joint_id >= 0:
            joint_ids.append(joint_id)
            qpos_indices.append(int(model.jnt_qposadr[joint_id]))
            dof_indices.append(int(model.jnt_dofadr[joint_id]))
    if len(set(qpos_indices)) != len(qpos_indices):
        errors.append("controlled joint qpos mappings must be unique")
    if len(set(dof_indices)) != len(dof_indices):
        errors.append("controlled joint DoF mappings must be unique")

    controlled_actuator_ids = []
    for name in spec.controlled_actuator_names:
        controlled_actuator_ids.append(
            _resolve(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name, "actuator", errors)
        )
    for name in (spec.left_arm.gripper_actuator, spec.right_arm.gripper_actuator):
        _resolve(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name, "gripper actuator", errors)
    if len(joint_ids) == len(controlled_actuator_ids):
        for joint_name, joint_id, actuator_id in zip(
            joint_names, joint_ids, controlled_actuator_ids
        ):
            if actuator_id >= 0 and int(model.actuator_trnid[actuator_id, 0]) != joint_id:
                errors.append(
                    f"actuator for joint {joint_name!r} does not map to that joint"
                )
    for name in (
        spec.left_arm.base_body,
        spec.right_arm.base_body,
        spec.left_arm.target_body,
        spec.right_arm.target_body,
        spec.object.body,
    ):
        _resolve(model, mujoco.mjtObj.mjOBJ_BODY, name, "body", errors)
    for name in (spec.left_arm.target_body, spec.right_arm.target_body):
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id >= 0 and model.body_mocapid[body_id] < 0:
            errors.append(f"target body {name!r} is not a mocap body")
    for name in (
        spec.left_arm.hand_site,
        spec.right_arm.hand_site,
        spec.object.reference_site,
        *spec.object.contact_sites,
    ):
        _resolve(model, mujoco.mjtObj.mjOBJ_SITE, name, "site", errors)
    _resolve(model, mujoco.mjtObj.mjOBJ_JOINT, spec.object.joint, "object joint", errors)
    _resolve(model, mujoco.mjtObj.mjOBJ_KEY, spec.home_keyframe, "keyframe", errors)
    if spec.mounting_platform_geom:
        _resolve(
            model,
            mujoco.mjtObj.mjOBJ_GEOM,
            spec.mounting_platform_geom,
            "mounting platform geom",
            errors,
        )
    if len(set(spec.object.contact_sites)) != 2:
        errors.append("object contact sites must be distinct")
    if spec.object.reference_site in spec.object.contact_sites:
        errors.append("object reference site must differ from contact sites")

    position_limits = spec.joint_position_limits
    velocity_limits = spec.joint_velocity_limits
    if len(position_limits) != len(joint_names):
        errors.append("one position limit pair is required per controlled joint")
    if len(velocity_limits) != len(joint_names):
        errors.append("one velocity limit pair is required per controlled joint")
    for label, limits in (("position", position_limits), ("velocity", velocity_limits)):
        for index, pair in enumerate(limits):
            if len(pair) != 2 or not np.all(np.isfinite(pair)) or pair[0] >= pair[1]:
                errors.append(f"invalid {label} limits at controlled joint index {index}: {pair}")
    if len(position_limits) == len(qpos_indices):
        for name, joint_id, declared in zip(joint_names, joint_ids, position_limits):
            actual = tuple(float(value) for value in model.jnt_range[joint_id])
            if not np.allclose(declared, actual, rtol=0, atol=1e-12):
                errors.append(f"joint {name!r} position limits {declared} do not match model {actual}")

    collision = spec.collision
    if collision is not None:
        if collision.sphere_resource is not None:
            sphere_path = Path(collision.sphere_resource)
            if not sphere_path.is_file():
                errors.append(f"collision sphere resource does not exist: {sphere_path}")
            else:
                try:
                    sphere_model = mujoco.MjModel.from_xml_path(str(sphere_path))
                    for name in (collision.left_root_body, collision.right_root_body):
                        _resolve(
                            sphere_model,
                            mujoco.mjtObj.mjOBJ_BODY,
                            name,
                            "collision root body",
                            errors,
                        )
                    sphere_count = sum(
                        sphere_model.geom(index).name.startswith(
                            collision.sphere_geom_prefix
                        )
                        for index in range(sphere_model.ngeom)
                    )
                    if sphere_count == 0:
                        errors.append(
                            "collision sphere resource contains no geoms with "
                            f"prefix {collision.sphere_geom_prefix!r}"
                        )
                except Exception as error:
                    errors.append(f"cannot load collision sphere resource: {error}")
        body_groups = (
            collision.left_inter_arm_bodies,
            collision.right_inter_arm_bodies,
            collision.left_table_bodies,
            collision.right_table_bodies,
            collision.left_terminal_bodies,
            collision.right_terminal_bodies,
            collision.left_gripper_bodies,
            collision.right_gripper_bodies,
        )
        for name in dict.fromkeys(name for group in body_groups for name in group):
            _resolve(model, mujoco.mjtObj.mjOBJ_BODY, name, "collision body", errors)
        if collision.table_root_body:
            _resolve(model, mujoco.mjtObj.mjOBJ_BODY, collision.table_root_body, "collision table body", errors)

    if errors:
        raise SpecValidationError(
            f"{spec.identifier} specification is invalid:\n- " + "\n- ".join(errors)
        )
    return {
        "identifier": spec.identifier,
        "model_path": str(model_path),
        "controlled_joint_count": len(joint_names),
        "qpos_indices": tuple(qpos_indices),
        "dof_indices": tuple(dof_indices),
        "collision_resource": (
            str(collision.sphere_resource)
            if collision is not None and collision.sphere_resource is not None
            else None
        ),
    }
