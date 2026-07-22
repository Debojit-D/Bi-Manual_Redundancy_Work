"""Step-wise, interruption-safe CSV recording for Equation (8) runs."""

import csv
from datetime import datetime
from pathlib import Path

import mujoco
import numpy as np


CSV_COLUMNS = [
    "time",
    "optimization_mode",
    "objective",
    "collision_version",
    "objective_value",
    "characteristic_length_m",
    "paper_objective_raw",
    "paper_objective_scaled",
    "velocity_manipulability",
    "velocity_manipulability_raw",
    "velocity_manipulability_scaled",
    "force_manipulability",
    "force_manipulability_raw",
    "force_manipulability_scaled",
    "directional_force_cost",
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
    "q_l1",
    "q_l2",
    "q_l3",
    "q_l4",
    "q_l5",
    "q_l6",
    "q_l7",
    "q_r1",
    "q_r2",
    "q_r3",
    "q_r4",
    "q_r5",
    "q_r6",
    "q_r7",
    "tau_actuator_norm",
    "tau_bias_norm",
    "tau_constraint_norm",
    "tau_applied_norm",
    "tau_total_est_norm",
    "tau_act_l1",
    "tau_act_l2",
    "tau_act_l3",
    "tau_act_l4",
    "tau_act_l5",
    "tau_act_l6",
    "tau_act_l7",
    "tau_act_r1",
    "tau_act_r2",
    "tau_act_r3",
    "tau_act_r4",
    "tau_act_r5",
    "tau_act_r6",
    "tau_act_r7",
]


class Equation8CSVRecorder:
    """Write one fully flushed CSV row after every controlled MuJoCo step."""

    def __init__(
        self,
        scene,
        kinematics,
        equation_8,
        optimizer,
        *,
        experiment_name,
        output_path=None,
        optimization_mode=None,
    ):
        self.scene = scene
        self.kinematics = kinematics
        self.equation_8 = equation_8
        self.optimizer = optimizer
        self.optimization_mode = (
            optimization_mode
            if optimization_mode is not None
            else optimizer.objective.value
        )
        self.output_path = self._make_output_path(
            experiment_name,
            output_path,
        )
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self._file = self.output_path.open(
            "x",
            newline="",
            encoding="utf-8",
        )
        self._writer = csv.DictWriter(self._file, fieldnames=CSV_COLUMNS)
        self._writer.writeheader()
        self._file.flush()
        self._start_simulation_time = None
        self.rows_written = 0

    @staticmethod
    def _make_output_path(experiment_name, output_path):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        if output_path is None:
            repository_root = Path(__file__).resolve().parents[3]
            path = (
                repository_root
                / "outputs"
                / "mujoco_data"
                / f"{experiment_name}_{timestamp}.csv"
            )
        else:
            path = Path(output_path).expanduser()
            if path.suffix.lower() != ".csv":
                path = path.with_suffix(".csv")
            if not path.is_absolute():
                path = Path.cwd() / path

        candidate = path
        counter = 1
        while candidate.exists():
            candidate = path.with_name(f"{path.stem}_{counter:03d}.csv")
            counter += 1
        return candidate

    def record(
        self,
        desired_position,
        desired_rotation,
        optimization,
        diagnostics,
    ):
        """Record the current post-step state and the latest control terms."""
        data = self.scene.data
        if self._start_simulation_time is None:
            self._start_simulation_time = float(data.time)
        elapsed_time = float(data.time - self._start_simulation_time)

        object_position, object_rotation = self.kinematics.object_pose(data)
        pose_error = self.equation_8.pose_error(
            desired_position,
            desired_rotation,
            object_position,
            object_rotation,
        )
        object_quaternion = np.empty(4)
        mujoco.mju_mat2Quat(object_quaternion, object_rotation.ravel())

        arm_positions = self.scene.arm_configuration()
        arm_dofs = self.scene.arm_dofs
        actuator_torque = data.qfrc_actuator[arm_dofs].copy()
        bias_torque = data.qfrc_bias[arm_dofs].copy()
        constraint_torque = data.qfrc_constraint[arm_dofs].copy()
        applied_torque = data.qfrc_applied[arm_dofs].copy()
        total_estimated_torque = (
            actuator_torque
            + applied_torque
            + constraint_torque
            - bias_torque
        )

        (
            velocity_map_raw,
            velocity_map_scaled,
            velocity_capability_raw,
            velocity_capability_scaled,
            force_capability_raw,
            force_capability_scaled,
        ) = self.optimizer.spatial_capability_matrices(data)
        (
            velocity_map_rank_raw,
            velocity_map_sigma_min_raw,
            velocity_map_sigma_max_raw,
            velocity_map_condition_raw,
        ) = self.optimizer.velocity_map_diagnostics(velocity_map_raw)
        (
            velocity_map_rank_scaled,
            velocity_map_sigma_min_scaled,
            velocity_map_sigma_max_scaled,
            velocity_map_condition_scaled,
        ) = self.optimizer.velocity_map_diagnostics(velocity_map_scaled)
        velocity_manipulability_raw = (
            self.optimizer.velocity_manipulability_raw(data)
        )
        velocity_manipulability_scaled = (
            self.optimizer.velocity_manipulability_scaled(data)
        )
        force_manipulability_raw = (
            self.optimizer.force_manipulability_raw(data)
        )
        force_manipulability_scaled = (
            self.optimizer.force_manipulability_scaled(data)
        )
        directional_force_cost_raw = (
            self.optimizer.directional_force_cost_raw(data)
        )
        directional_force_cost_scaled = (
            self.optimizer.directional_force_cost_scaled(data)
        )
        paper_objective_raw, paper_objective_scaled = (
            self.optimizer.paper_objective_values(data)
        )

        row = {
            "time": elapsed_time,
            "optimization_mode": self.optimization_mode,
            "objective": self.optimizer.objective.value,
            "collision_version": self.optimizer.collision_version.value,
            "objective_value": self.optimizer.value(data),
            "characteristic_length_m": (
                self.optimizer.characteristic_length
            ),
            "paper_objective_raw": paper_objective_raw,
            "paper_objective_scaled": paper_objective_scaled,
            # Unsuffixed columns remain stable aliases of active scaled metrics.
            "velocity_manipulability": velocity_manipulability_scaled,
            "velocity_manipulability_raw": velocity_manipulability_raw,
            "velocity_manipulability_scaled": (
                velocity_manipulability_scaled
            ),
            "force_manipulability": force_manipulability_scaled,
            "force_manipulability_raw": force_manipulability_raw,
            "force_manipulability_scaled": force_manipulability_scaled,
            "directional_force_cost": directional_force_cost_scaled,
            "directional_force_cost_raw": directional_force_cost_raw,
            "directional_force_cost_scaled": directional_force_cost_scaled,
            "velocity_capability_trace_raw": np.trace(
                velocity_capability_raw
            ),
            "velocity_capability_trace_scaled": np.trace(
                velocity_capability_scaled
            ),
            "force_capability_trace_raw": np.trace(force_capability_raw),
            "force_capability_trace_scaled": np.trace(
                force_capability_scaled
            ),
            "velocity_map_rank_raw": velocity_map_rank_raw,
            "velocity_map_rank_scaled": velocity_map_rank_scaled,
            "velocity_map_sigma_min_raw": velocity_map_sigma_min_raw,
            "velocity_map_sigma_max_raw": velocity_map_sigma_max_raw,
            "velocity_map_condition_raw": velocity_map_condition_raw,
            "velocity_map_sigma_min_scaled": (
                velocity_map_sigma_min_scaled
            ),
            "velocity_map_sigma_max_scaled": (
                velocity_map_sigma_max_scaled
            ),
            "velocity_map_condition_scaled": velocity_map_condition_scaled,
            "object_x": object_position[0],
            "object_y": object_position[1],
            "object_z": object_position[2],
            "object_qw": object_quaternion[0],
            "object_qx": object_quaternion[1],
            "object_qy": object_quaternion[2],
            "object_qz": object_quaternion[3],
            "position_error_x": pose_error[0],
            "position_error_y": pose_error[1],
            "position_error_z": pose_error[2],
            "position_error_norm": np.linalg.norm(pose_error[:3]),
            "orientation_error_x": pose_error[3],
            "orientation_error_y": pose_error[4],
            "orientation_error_z": pose_error[5],
            "orientation_error_norm": np.linalg.norm(pose_error[3:]),
            "grasp_error_norm": np.linalg.norm(
                diagnostics.grasp_pose_error
            ),
            "primary_speed_max": np.max(
                np.abs(diagnostics.primary_joint_velocity)
            ),
            "null_speed_max": np.max(
                np.abs(diagnostics.null_space_joint_velocity)
            ),
            "null_space_scale": diagnostics.null_space_scale,
            "min_joint_limit_distance": (
                diagnostics.minimum_joint_limit_distance
            ),
            "unscaled_null_space_leakage": (
                diagnostics.unscaled_null_space_leakage
            ),
            "scaled_null_space_leakage": (
                diagnostics.scaled_null_space_leakage
            ),
            "command_speed_max": np.max(
                np.abs(diagnostics.commanded_joint_velocity)
            ),
            "phi_dot_opt_max": np.max(np.abs(optimization.phi_dot_opt)),
            "gradient_norm": np.linalg.norm(optimization.gradient),
            "min_inter_arm_clearance": (
                self.optimizer.minimum_inter_arm_clearance(data)
            ),
            "collision_cost": self.optimizer.inter_arm_collision_cost(data),
            "min_arm_table_clearance": (
                self.optimizer.minimum_arm_table_clearance(data)
            ),
            "arm_table_collision_cost": (
                self.optimizer.arm_table_collision_cost(data)
            ),
            "min_self_collision_clearance": (
                self.optimizer.minimum_self_collision_clearance(data)
            ),
            "self_collision_cost": self.optimizer.self_collision_cost(data),
            "total_weighted_collision_cost": (
                self.optimizer.total_collision_cost(data)
            ),
            "tau_actuator_norm": np.linalg.norm(actuator_torque),
            "tau_bias_norm": np.linalg.norm(bias_torque),
            "tau_constraint_norm": np.linalg.norm(constraint_torque),
            "tau_applied_norm": np.linalg.norm(applied_torque),
            "tau_total_est_norm": np.linalg.norm(total_estimated_torque),
        }
        row.update(
            {f"q_l{index + 1}": arm_positions[index] for index in range(7)}
        )
        row.update(
            {
                f"q_r{index + 1}": arm_positions[index + 7]
                for index in range(7)
            }
        )
        row.update(
            {
                f"tau_act_l{index + 1}": actuator_torque[index]
                for index in range(7)
            }
        )
        row.update(
            {
                f"tau_act_r{index + 1}": actuator_torque[index + 7]
                for index in range(7)
            }
        )

        self._writer.writerow(row)
        # Flushing every row makes Ctrl+C and early viewer closure durable.
        self._file.flush()
        self.rows_written += 1

    def close(self):
        """Flush and close the CSV; safe to call more than once."""
        if self._file is not None:
            self._file.flush()
            self._file.close()
            self._file = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()
