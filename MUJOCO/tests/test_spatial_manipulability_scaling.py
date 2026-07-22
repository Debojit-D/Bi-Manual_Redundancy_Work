"""Focused tests for spatial scaling and raw/scaled CSV diagnostics."""

import csv
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

from MUJOCO.utils.data_recording import CSV_COLUMNS, Equation8CSVRecorder
from MUJOCO.utils.redundancy_optimization import (
    ManipulabilityObjective,
    ManipulabilityOptimizer,
)


LEGACY_CSV_COLUMNS = {
    "time",
    "optimization_mode",
    "objective",
    "collision_version",
    "objective_value",
    "velocity_manipulability",
    "force_manipulability",
    "directional_force_cost",
    "object_x",
    "object_y",
    "object_z",
    "object_qw",
    "object_qx",
    "object_qy",
    "object_qz",
    "position_error_x",
    "position_error_y",
    "position_error_z",
    "position_error_norm",
    "orientation_error_x",
    "orientation_error_y",
    "orientation_error_z",
    "orientation_error_norm",
    "grasp_error_norm",
    "primary_speed_max",
    "null_speed_max",
    "null_space_scale",
    "min_joint_limit_distance",
    "unscaled_null_space_leakage",
    "scaled_null_space_leakage",
    "command_speed_max",
    "phi_dot_opt_max",
    "gradient_norm",
    "min_inter_arm_clearance",
    "collision_cost",
    "min_arm_table_clearance",
    "arm_table_collision_cost",
    "min_self_collision_clearance",
    "self_collision_cost",
    "total_weighted_collision_cost",
    "tau_actuator_norm",
    "tau_bias_norm",
    "tau_constraint_norm",
    "tau_applied_norm",
    "tau_total_est_norm",
} | {
    f"{prefix}{index}"
    for prefix in ("q_l", "q_r", "tau_act_l", "tau_act_r")
    for index in range(1, 8)
}


class _FakeNamedObject:
    id = 0


class _FakeModel:
    def body(self, _name):
        return _FakeNamedObject()


class _FakeKinematics:
    def __init__(self, velocity_map):
        self.model = _FakeModel()
        self.arm_dofs = np.arange(14)
        self.pinv_rcond = 1e-9
        self.velocity_map = np.asarray(velocity_map, dtype=float)

    def paper_object_velocity_map(self, _data):
        return self.velocity_map.copy()

    def object_pose(self, _data):
        return np.array([0.1, 0.2, 0.3]), np.eye(3)


class _FakeScene:
    def __init__(self):
        zeros = np.zeros(14)
        self.data = SimpleNamespace(
            time=0.02,
            qfrc_actuator=zeros.copy(),
            qfrc_bias=zeros.copy(),
            qfrc_constraint=zeros.copy(),
            qfrc_applied=zeros.copy(),
        )
        self.arm_dofs = np.arange(14)

    @staticmethod
    def arm_configuration():
        return np.linspace(-0.5, 0.5, 14)


class _FakeEquation8:
    @staticmethod
    def pose_error(*_args):
        return np.zeros(6)


def _full_rank_velocity_map():
    matrix = np.zeros((6, 14))
    matrix[:, :6] = np.diag([1.0, 1.2, 1.4, 0.8, 1.1, 1.3])
    matrix[:, 6:] = np.arange(48, dtype=float).reshape(6, 8) / 100.0
    return matrix


def _optimizer(*, characteristic_length=0.4, objective="force", direction=None):
    if direction is None:
        direction = (1.0, 2.0, 3.0, 0.5, 0.75, 1.25)
    return ManipulabilityOptimizer(
        _FakeKinematics(_full_rank_velocity_map()),
        np.arange(14),
        objective=objective,
        characteristic_length=characteristic_length,
        desired_wrench_direction=direction,
        maximum_joint_speed=100.0,
    )


class SpatialManipulabilityScalingTests(unittest.TestCase):
    def test_scaling_matrix_and_velocity_map(self):
        optimizer = _optimizer(characteristic_length=0.4)
        expected_scaling = np.diag([1.0, 1.0, 1.0, 0.4, 0.4, 0.4])
        raw_map, scaled_map = optimizer.object_velocity_maps(None)

        np.testing.assert_allclose(
            optimizer.spatial_scaling_matrix(),
            expected_scaling,
        )
        np.testing.assert_allclose(raw_map, _full_rank_velocity_map())
        np.testing.assert_allclose(scaled_map, expected_scaling @ raw_map)

    def test_velocity_and_force_capability_matrices(self):
        optimizer = _optimizer(characteristic_length=0.4)
        (
            raw_map,
            scaled_map,
            velocity_raw,
            velocity_scaled,
            force_raw,
            force_scaled,
        ) = optimizer.spatial_capability_matrices(None)

        np.testing.assert_allclose(velocity_raw, raw_map @ raw_map.T)
        np.testing.assert_allclose(velocity_scaled, scaled_map @ scaled_map.T)
        np.testing.assert_allclose(
            force_raw,
            np.linalg.pinv(
                velocity_raw,
                rcond=optimizer.kinematics.pinv_rcond,
            ),
        )
        np.testing.assert_allclose(
            force_scaled,
            np.linalg.pinv(
                velocity_scaled,
                rcond=optimizer.kinematics.pinv_rcond,
            ),
        )

    def test_characteristic_length_one_makes_raw_and_scaled_equal(self):
        optimizer = _optimizer(characteristic_length=1.0)
        raw_map, scaled_map = optimizer.object_velocity_maps(None)
        np.testing.assert_allclose(raw_map, scaled_map)
        np.testing.assert_allclose(
            optimizer.desired_force_matrix_raw,
            optimizer.desired_force_matrix_scaled,
        )
        for raw_method, scaled_method in (
            (
                optimizer.velocity_manipulability_raw,
                optimizer.velocity_manipulability_scaled,
            ),
            (
                optimizer.force_manipulability_raw,
                optimizer.force_manipulability_scaled,
            ),
            (
                optimizer.directional_force_cost_raw,
                optimizer.directional_force_cost_scaled,
            ),
        ):
            self.assertAlmostEqual(raw_method(None), scaled_method(None))

    def test_pure_moment_wrench_is_dual_scaled_once(self):
        optimizer = _optimizer(
            characteristic_length=0.4,
            direction=(0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
        )
        self.assertEqual(optimizer.desired_force_matrix_raw[3, 3], 1.0)
        self.assertEqual(optimizer.desired_force_matrix_scaled[3, 3], 2.5)
        self.assertIs(
            optimizer.desired_force_matrix,
            optimizer.desired_force_matrix_scaled,
        )

    def test_unsuffixed_public_methods_return_scaled_metrics(self):
        optimizer = _optimizer(characteristic_length=0.4)
        self.assertEqual(
            optimizer.velocity_manipulability(None),
            optimizer.velocity_manipulability_scaled(None),
        )
        self.assertEqual(
            optimizer.force_manipulability(None),
            optimizer.force_manipulability_scaled(None),
        )
        self.assertEqual(
            optimizer.directional_force_cost(None),
            optimizer.directional_force_cost_scaled(None),
        )

    def test_optimization_signs_are_unchanged(self):
        for objective, expected_sign in (
            (ManipulabilityObjective.VELOCITY, 1.0),
            (ManipulabilityObjective.FORCE, 1.0),
            (ManipulabilityObjective.DIRECTIONAL_FORCE, -1.0),
        ):
            optimizer = _optimizer(objective=objective)
            optimizer.value = lambda _data: 7.0
            optimizer.gradient = lambda _data: np.ones(14)
            result = optimizer.optimization_velocity(None)
            np.testing.assert_allclose(result.phi_dot_opt, expected_sign)

    def test_value_and_gradient_use_scaled_metrics(self):
        optimizer = _optimizer(
            characteristic_length=0.4,
            objective=ManipulabilityObjective.VELOCITY,
        )
        data = SimpleNamespace(qpos=np.ones(14))

        def velocity_map(current_data):
            matrix = np.zeros((6, 14))
            diagonal = np.ones(6)
            diagonal[0] = current_data.qpos[0]
            matrix[:, :6] = np.diag(diagonal)
            return matrix

        optimizer.kinematics.paper_object_velocity_map = velocity_map
        self.assertEqual(
            optimizer.value(data),
            optimizer.velocity_manipulability_scaled(data),
        )
        with patch(
            "MUJOCO.utils.redundancy_optimization."
            "manipulability_optimization.mujoco.mj_forward"
        ):
            gradient = optimizer.gradient(data)
        # sqrt(det(A_scaled A_scaled.T))=l^3*abs(det(A_raw)).
        self.assertAlmostEqual(gradient[0], 0.4**3, places=9)
        self.assertNotAlmostEqual(gradient[0], 1.0)

        collision_cost = 3.0
        optimizer.total_collision_cost = lambda _data: collision_cost
        for objective, expected in (
            (
                ManipulabilityObjective.VELOCITY,
                optimizer.velocity_manipulability_scaled(data)
                - collision_cost,
            ),
            (
                ManipulabilityObjective.FORCE,
                optimizer.force_manipulability_scaled(data)
                - collision_cost,
            ),
            (
                ManipulabilityObjective.DIRECTIONAL_FORCE,
                optimizer.directional_force_cost_scaled(data)
                + collision_cost,
            ),
        ):
            self.assertAlmostEqual(optimizer.value(data, objective), expected)

    def test_rank_deficient_diagnostics_and_pseudoinverse_are_safe(self):
        velocity_map = np.zeros((6, 14))
        velocity_map[:2, :2] = np.diag([2.0, 1.0])
        optimizer = _optimizer()
        optimizer.kinematics.velocity_map = velocity_map

        force_raw, force_scaled = optimizer.force_capability_matrices(None)
        self.assertTrue(np.all(np.isfinite(force_raw)))
        self.assertTrue(np.all(np.isfinite(force_scaled)))
        for matrix in optimizer.object_velocity_maps(None):
            rank, sigma_min, sigma_max, condition = (
                optimizer.velocity_map_diagnostics(matrix)
            )
            self.assertEqual(rank, 2)
            self.assertEqual(sigma_min, 0.0)
            self.assertGreater(sigma_max, 0.0)
            self.assertTrue(np.isinf(condition))

    def test_csv_schema_and_scaled_aliases(self):
        optimizer = _optimizer(
            objective=ManipulabilityObjective.FORCE,
            characteristic_length=0.4,
        )
        optimizer.minimum_inter_arm_clearance = lambda _data: 1.0
        optimizer.inter_arm_collision_cost = lambda _data: 0.0
        optimizer.minimum_arm_table_clearance = lambda _data: 1.0
        optimizer.arm_table_collision_cost = lambda _data: 0.0
        optimizer.minimum_self_collision_clearance = lambda _data: 1.0
        optimizer.self_collision_cost = lambda _data: 0.0
        optimizer.total_collision_cost = lambda _data: 0.0
        diagnostics = SimpleNamespace(
            grasp_pose_error=np.zeros(12),
            primary_joint_velocity=np.zeros(14),
            null_space_joint_velocity=np.zeros(14),
            null_space_scale=1.0,
            minimum_joint_limit_distance=0.5,
            unscaled_null_space_leakage=0.0,
            scaled_null_space_leakage=0.0,
            commanded_joint_velocity=np.zeros(14),
        )
        optimization = SimpleNamespace(
            phi_dot_opt=np.zeros(14),
            gradient=np.zeros(14),
        )

        with tempfile.TemporaryDirectory() as directory:
            output_path = Path(directory) / "scaling.csv"
            with Equation8CSVRecorder(
                _FakeScene(),
                optimizer.kinematics,
                _FakeEquation8(),
                optimizer,
                experiment_name="scaling_test",
                output_path=output_path,
                optimization_mode="baseline",
            ) as recorder:
                recorder.record(
                    np.zeros(3),
                    np.eye(3),
                    optimization,
                    diagnostics,
                )
            with output_path.open(newline="", encoding="utf-8") as stream:
                row = next(csv.DictReader(stream))

        required_new_columns = {
            "characteristic_length_m",
            "paper_objective_raw",
            "paper_objective_scaled",
            "velocity_manipulability_raw",
            "velocity_manipulability_scaled",
            "force_manipulability_raw",
            "force_manipulability_scaled",
            "directional_force_cost_raw",
            "directional_force_cost_scaled",
            "velocity_capability_trace_raw",
            "velocity_capability_trace_scaled",
            "force_capability_trace_raw",
            "force_capability_trace_scaled",
            "velocity_map_rank_raw",
            "velocity_map_rank_scaled",
            "velocity_map_sigma_min_raw",
            "velocity_map_sigma_max_raw",
            "velocity_map_condition_raw",
            "velocity_map_sigma_min_scaled",
            "velocity_map_sigma_max_scaled",
            "velocity_map_condition_scaled",
        }
        self.assertTrue(required_new_columns.issubset(CSV_COLUMNS))
        self.assertTrue(LEGACY_CSV_COLUMNS.issubset(CSV_COLUMNS))
        self.assertTrue(LEGACY_CSV_COLUMNS.issubset(row))
        for new_column in required_new_columns:
            self.assertNotEqual(row[new_column], "")
        self.assertEqual(
            row["velocity_manipulability"],
            row["velocity_manipulability_scaled"],
        )
        self.assertEqual(
            row["force_manipulability"],
            row["force_manipulability_scaled"],
        )
        self.assertEqual(
            row["directional_force_cost"],
            row["directional_force_cost_scaled"],
        )
        self.assertEqual(row["optimization_mode"], "baseline")
        self.assertEqual(
            row["paper_objective_scaled"],
            row["force_manipulability_scaled"],
        )


if __name__ == "__main__":
    unittest.main()
