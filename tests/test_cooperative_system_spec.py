from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

from bimanual_redundancy.cli import build_parser, main as cli_main
from bimanual_redundancy.core import ManipulabilityOptimizer
from bimanual_redundancy.simulation import CooperativeMuJoCoScene
from bimanual_redundancy.systems import (
    DUAL_FRANKA_PANDA,
    SpecValidationError,
    validate_cooperative_system_spec,
)


def test_reference_spec_resolves_all_mappings_and_limits():
    result = validate_cooperative_system_spec(DUAL_FRANKA_PANDA)
    assert result["controlled_joint_count"] == 14
    assert result["qpos_indices"] == (0, 1, 2, 3, 4, 5, 6, 9, 10, 11, 12, 13, 14, 15)
    assert result["dof_indices"] == result["qpos_indices"]
    assert Path(result["model_path"]).is_file()
    assert Path(result["collision_resource"]).is_file()


def test_invalid_joint_name_has_a_useful_error():
    bad_left = replace(
        DUAL_FRANKA_PANDA.left_arm,
        joint_names=("missing_joint",) + DUAL_FRANKA_PANDA.left_arm.joint_names[1:],
    )
    bad_spec = replace(DUAL_FRANKA_PANDA, identifier="bad_joint", left_arm=bad_left)
    with pytest.raises(SpecValidationError, match="joint 'missing_joint' does not resolve"):
        validate_cooperative_system_spec(bad_spec)


def test_invalid_contact_site_has_a_useful_error():
    bad_object = replace(
        DUAL_FRANKA_PANDA.object,
        contact_sites=("missing_contact", DUAL_FRANKA_PANDA.object.contact_sites[1]),
    )
    bad_spec = replace(DUAL_FRANKA_PANDA, identifier="bad_site", object=bad_object)
    with pytest.raises(SpecValidationError, match="site 'missing_contact' does not resolve"):
        validate_cooperative_system_spec(bad_spec)


def test_collision_configuration_is_optional():
    no_collision = replace(
        DUAL_FRANKA_PANDA,
        identifier="reference_without_collision",
        collision=None,
    )
    result = validate_cooperative_system_spec(no_collision)
    assert result["collision_resource"] is None


def test_reference_dual_arm_regression_dimensions_and_values():
    scene = CooperativeMuJoCoScene(system_spec=DUAL_FRANKA_PANDA)
    kinematics = scene.make_kinematics()
    assert kinematics.hand_jacobian(scene.data).shape == (12, 14)
    assert kinematics.grasp_matrix(scene.data).shape == (6, 12)
    velocity_map = kinematics.paper_object_velocity_map(scene.data)
    assert velocity_map.shape == (6, 14)
    np.testing.assert_allclose(np.sum(velocity_map), 2.7475825193317913, rtol=1e-12)
    optimizer = ManipulabilityOptimizer(
        kinematics,
        scene.arm_qpos,
        characteristic_length=kinematics.grasp_characteristic_length(scene.data),
    )
    np.testing.assert_allclose(
        optimizer.velocity_manipulability(scene.data),
        0.01053745589837254,
        rtol=1e-12,
    )


def test_generic_math_modules_contain_no_reference_embodiment_names():
    package = Path(__file__).parents[1] / "src" / "bimanual_redundancy" / "core"
    for filename in (
        "cooperative_kinematics.py",
        "controller.py",
        "objectives.py",
        "gradients.py",
    ):
        source = (package / filename).read_text(encoding="utf-8").lower()
        assert "franka" not in source
        assert "link7_l" not in source
        assert "attachment_site_left" not in source


def test_validate_robot_cli_and_argument_validation(capsys):
    parsed = build_parser().parse_args(
        ["validate-robot", "--robot", "dual_franka_panda"]
    )
    assert parsed.robot == "dual_franka_panda"
    assert cli_main(["validate-robot", "--robot", "dual_franka_panda"]) == 0
    assert "14 controlled joints" in capsys.readouterr().out
    with pytest.raises(SystemExit):
        build_parser().parse_args(["validate-robot", "--robot", "unknown"])
