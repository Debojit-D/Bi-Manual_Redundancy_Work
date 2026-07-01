"""Closed-loop cooperative controller implementing Equation (8)."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Equation8Diagnostics:
    pose_error: np.ndarray
    rank_tracking_map: int
    condition_tracking_map: float
    primary_joint_velocity: np.ndarray
    null_space_joint_velocity: np.ndarray


class Equation8Controller:
    """Evaluate the primary and projected optimization terms of Equation (8)."""

    def __init__(
        self,
        kinematics,
        *,
        control_dt,
        feedback_gain,
        control_pinv_rcond=1e-5,
    ):
        self.kinematics = kinematics
        self.control_dt = float(control_dt)
        self.feedback_gain = np.asarray(feedback_gain, dtype=float)
        self.control_pinv_rcond = float(control_pinv_rcond)
        if self.feedback_gain.shape != (6, 6):
            raise ValueError("feedback_gain must be a 6x6 matrix")

    @staticmethod
    def pose_error(
        desired_position,
        desired_rotation,
        current_position,
        current_rotation,
    ):
        """Return world-frame position and rotation-vector tracking error."""
        position_error = desired_position - current_position
        orientation_error = Rotation.from_matrix(
            desired_rotation @ current_rotation.T
        ).as_rotvec()
        return np.concatenate((position_error, orientation_error))

    def update(
        self,
        data,
        phi,
        desired_position,
        desired_rotation,
        desired_twist,
        phi_dot_opt=None,
    ):
        """Apply one discrete Equation (8) update.

        phi_next = phi + [A^dagger(q_dot_d + K_p e)
                          + (I-J_H^dagger J_H)phi_dot_opt]dt

        ``phi_dot_opt=None`` disables null-space optimization.
        """
        phi = np.asarray(phi, dtype=float)
        desired_twist = np.asarray(desired_twist, dtype=float)
        number_of_joints = self.kinematics.arm_dofs.size
        if phi.shape != (number_of_joints,):
            raise ValueError(
                f"Expected phi shape {(number_of_joints,)}, got {phi.shape}"
            )

        current_position, current_rotation = self.kinematics.object_pose(data)
        error = self.pose_error(
            desired_position,
            desired_rotation,
            current_position,
            current_rotation,
        )
        closed_loop_twist = desired_twist + self.feedback_gain @ error

        tracking_map = self.kinematics.tracking_object_velocity_map(data)
        primary_joint_velocity = (
            np.linalg.pinv(
                tracking_map, rcond=self.control_pinv_rcond
            )
            @ closed_loop_twist
        )

        if phi_dot_opt is None:
            null_space_joint_velocity = np.zeros(number_of_joints)
        else:
            phi_dot_opt = np.asarray(phi_dot_opt, dtype=float)
            if phi_dot_opt.shape != (number_of_joints,):
                raise ValueError("phi_dot_opt must have the same shape as phi")
            null_space_joint_velocity = (
                self.kinematics.null_space_projector(data) @ phi_dot_opt
            )

        phi_next = phi + (
            primary_joint_velocity + null_space_joint_velocity
        ) * self.control_dt
        diagnostics = Equation8Diagnostics(
            pose_error=error,
            rank_tracking_map=int(np.linalg.matrix_rank(tracking_map)),
            condition_tracking_map=float(np.linalg.cond(tracking_map)),
            primary_joint_velocity=primary_joint_velocity,
            null_space_joint_velocity=null_space_joint_velocity,
        )
        return phi_next, diagnostics
