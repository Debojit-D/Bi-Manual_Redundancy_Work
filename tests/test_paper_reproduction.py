import json
from pathlib import Path
import tomllib

import numpy as np
import pytest

from bimanual_redundancy.cli import build_parser
from bimanual_redundancy.core import (
    CooperativeManipulationKinematics,
    ManipulabilityOptimizer,
)
from bimanual_redundancy.paper_config import (
    ConfigError,
    EXPECTED_OBJECTIVES,
    PAPER_CONFIG_NAMES,
    load_named_paper_config,
    toml_dumps,
    validate_paper_config,
)
from bimanual_redundancy.paper_reproduction import (
    RunSelection,
    build_run_metadata,
    plan_runs,
    reproduce_paper,
)
from bimanual_redundancy.simulation import DualFrankaMuJoCoScene
from bimanual_redundancy.experiments import (
    dual_franka_eq8_optimized_6d_pick_place as six_d_runner,
    dual_franka_eq8_optimized_pick_place as translational_runner,
    dual_franka_eq8_static_optimization as static_runner,
)
from bimanual_redundancy.experiments.table_spawn_comparison_positions import (
    SIX_D_TRAJECTORY_CASES,
    TABLE_SPAWN_CASES,
)


def test_all_paper_toml_files_parse_and_round_trip():
    for name in PAPER_CONFIG_NAMES:
        config = load_named_paper_config(name)
        assert config.experiment == name
        assert tomllib.loads(toml_dumps(config.data)) == config.data


def test_all_six_pose_identifiers_are_present():
    config = load_named_paper_config("six_d")
    assert [case["id"] for case in config.data["initial_configurations"]] == [
        f"pose_{index}" for index in range(1, 7)
    ]


def test_config_values_match_authoritative_runner_constants():
    modules = {
        "static": static_runner,
        "translational": translational_runner,
        "six_d": six_d_runner,
        "directional_direct_vs_indirect": static_runner,
    }
    for name, module in modules.items():
        data = load_named_paper_config(name).data
        assert data["controller"]["frequency_hz"] == module.CONTROL_HZ
        np.testing.assert_array_equal(
            data["controller"]["feedback_gains"], np.diag(module.K_P)
        )
        np.testing.assert_array_equal(
            data["controller"]["grasp_feedback_gains"], np.diag(module.GRASP_K_P)
        )
        assert data["optimization"]["gain"] == module.OPTIMIZATION_GAIN
        assert data["optimization"]["finite_difference_step"] == module.FINITE_DIFFERENCE_STEP
        assert data["optimization"]["maximum_joint_velocity"] == module.MAXIMUM_OPTIMIZATION_JOINT_SPEED
        np.testing.assert_array_equal(
            data["optimization"]["desired_wrench_direction"],
            module.DESIRED_WRENCH_DIRECTION,
        )

    static_positions = tuple(
        (case["id"], tuple(case["position"]))
        for case in load_named_paper_config("static").data["initial_configurations"]
    )
    assert static_positions == TABLE_SPAWN_CASES
    configured_poses = load_named_paper_config("six_d").data["initial_configurations"]
    for configured, authoritative in zip(configured_poses, SIX_D_TRAJECTORY_CASES):
        assert configured["id"] == authoritative[0]
        flattened = (
            configured["position"], configured["euler_xyz"],
            configured["intermediate_position"], configured["intermediate_euler_xyz"],
            configured["goal_position"], configured["goal_euler_xyz"],
        )
        for actual, expected in zip(flattened, authoritative[1:]):
            np.testing.assert_allclose(actual, expected, rtol=0, atol=1e-15)
    translational = load_named_paper_config("translational").data["trajectory"]
    assert translational == {
        "lift_height": translational_runner.LIFT_HEIGHT,
        "lift_duration": translational_runner.LIFT_DURATION,
        "lower_duration": translational_runner.LOWER_DURATION,
        "final_hold_duration": translational_runner.FINAL_HOLD_DURATION,
    }
    six_d = load_named_paper_config("six_d").data["trajectory"]
    assert six_d == {
        "start_to_intermediate_duration": six_d_runner.START_TO_INTERMEDIATE_DURATION,
        "intermediate_to_goal_duration": six_d_runner.INTERMEDIATE_TO_GOAL_DURATION,
        "final_hold_duration": six_d_runner.FINAL_HOLD_DURATION,
    }


@pytest.mark.parametrize("name", PAPER_CONFIG_NAMES)
def test_expected_objectives_per_experiment(name):
    config = load_named_paper_config(name)
    assert tuple(config.data["paper"]["objectives"]) == EXPECTED_OBJECTIVES[name]
    assert len(plan_runs(config)) == 6 * len(EXPECTED_OBJECTIVES[name])


def test_resolved_config_validation_rejects_campaign_drift():
    config = load_named_paper_config("translational")
    changed = config.data | {"initial_configurations": config.data["initial_configurations"][:5]}
    with pytest.raises(ConfigError, match="exactly six"):
        validate_paper_config(changed)

    changed = dict(config.data)
    changed["paper"] = dict(config.data["paper"], objectives=["baseline", "force"])
    with pytest.raises(ConfigError, match="objectives must be"):
        validate_paper_config(changed)


def test_metadata_generation_records_required_provenance():
    config = load_named_paper_config("static")
    selection = RunSelection("static", "force", config.data["initial_configurations"][0])
    metadata = build_run_metadata(config, selection, timestamp="2026-01-02T03:04:05+00:00")
    required = {
        "git_commit_sha",
        "git_working_tree_dirty",
        "paper_experiment",
        "objective",
        "initial_configuration_identifier",
        "python_version",
        "mujoco_version",
        "mink_version",
        "numpy_version",
        "scipy_version",
        "platform",
        "operating_system",
        "resolved_config",
        "timestamp",
    }
    assert required <= metadata.keys()
    assert metadata["objective"] == "force"
    assert metadata["initial_configuration_identifier"] == "position_1"


def test_smoke_orchestration_uses_runner_path_and_writes_audits(tmp_path):
    calls = []

    def fake_executor(selection, run_dir, config):
        calls.append((selection, run_dir, config))
        (run_dir / "data.csv").write_text("time\n0.0\n", encoding="utf-8")

    batch = reproduce_paper(output_root=tmp_path, smoke=True, executor=fake_executor)
    assert len(calls) == 1
    selection, run_dir, config = calls[0]
    assert selection.experiment == "static"
    assert selection.objective == "baseline"
    assert selection.identifier == "position_1"
    assert config.data["smoke"] is True
    assert config.data["stopping"]["fixed_duration_seconds"] == 0.02
    assert (run_dir / "run_metadata.json").is_file()
    assert (run_dir / "resolved_config.toml").is_file()
    metadata = json.loads((run_dir / "run_metadata.json").read_text())
    assert metadata["status"] == "completed"
    assert tomllib.loads((run_dir / "resolved_config.toml").read_text())["smoke"] is True
    assert run_dir == batch / "static" / "position_1" / "baseline"


def test_cli_argument_validation():
    parser = build_parser()
    with pytest.raises(SystemExit):
        parser.parse_args([])
    with pytest.raises(SystemExit):
        parser.parse_args(["run"])
    parsed = parser.parse_args(["reproduce-paper", "--smoke"])
    assert parsed.command == "reproduce-paper"
    assert parsed.smoke is True


def test_reference_initial_objective_values_are_unchanged():
    """Regression captured before configuration injection was introduced."""
    config = load_named_paper_config("static").data
    scene = DualFrankaMuJoCoScene(
        control_hz=config["controller"]["frequency_hz"],
        left_arm_base_position=config["robot_bases"]["left_position"],
        right_arm_base_position=config["robot_bases"]["right_position"],
        left_arm_base_euler_xyz_degrees=config["robot_bases"]["left_euler_xyz_degrees"],
        right_arm_base_euler_xyz_degrees=config["robot_bases"]["right_euler_xyz_degrees"],
        enable_bias_compensation=config["controller"]["arm_bias_compensation"],
    )
    scene.set_table_reference_pose(config["initial_configurations"][0]["position"])
    kinematics = scene.make_kinematics()
    characteristic_length = kinematics.grasp_characteristic_length(scene.data)
    optimizer = ManipulabilityOptimizer(
        kinematics,
        scene.arm_qpos,
        desired_wrench_direction=config["optimization"]["desired_wrench_direction"],
        characteristic_length=characteristic_length,
    )
    np.testing.assert_allclose(characteristic_length, 0.20000000000000007, rtol=0, atol=1e-14)
    np.testing.assert_allclose(optimizer.velocity_manipulability(scene.data), 0.01053745589837254, rtol=1e-12)
    np.testing.assert_allclose(optimizer.force_manipulability(scene.data), 94.89956680667525, rtol=1e-12)
    np.testing.assert_allclose(optimizer.directional_force_direct_cost(scene.data), 0.9723279309616865, rtol=1e-12)
    np.testing.assert_allclose(optimizer.directional_force_indirect_cost(scene.data), 1.107868754232261, rtol=1e-12)
