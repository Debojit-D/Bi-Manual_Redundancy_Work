"""Closed-loop cooperative controller implementing Equation (8)."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Equation8Diagnostics:
    pose_error: np.ndarray
    grasp_pose_error: np.ndarray
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
        grasp_feedback_gain=None,
    ):
        self.kinematics = kinematics
        self.control_dt = float(control_dt)
        self.feedback_gain = np.asarray(feedback_gain, dtype=float)
        self.control_pinv_rcond = float(control_pinv_rcond)
        self.grasp_feedback_gain = self._make_grasp_feedback_gain(
            grasp_feedback_gain
        )
        self._grasp_reference = None
        if self.feedback_gain.shape != (6, 6):
            raise ValueError("feedback_gain must be a 6x6 matrix")

    @staticmethod
    def _make_grasp_feedback_gain(gain):
        """Normalize an optional per-hand gain to a 12x12 matrix."""
        if gain is None:
            return None
        gain = np.asarray(gain, dtype=float)
        if gain.ndim == 0:
            return float(gain) * np.eye(12)
        if gain.shape == (6, 6):
            return np.block(
                [[gain, np.zeros((6, 6))], [np.zeros((6, 6)), gain]]
            )
        if gain.shape != (12, 12):
            raise ValueError(
                "grasp_feedback_gain must be scalar, 6x6, 12x12, or None"
            )
        return gain.copy()

    def capture_grasp_reference(self, data):
        """Store each hand pose relative to the currently grasped object."""
        object_position, object_rotation = self.kinematics.object_pose(data)
        references = []
        for site_name in self.kinematics.hand_sites:
            site_id = self.kinematics.model.site(site_name).id
            hand_position = data.site_xpos[site_id]
            hand_rotation = data.site_xmat[site_id].reshape(3, 3)
            references.append(
                (
                    object_rotation.T @ (hand_position - object_position),
                    object_rotation.T @ hand_rotation,
                )
            )
        self._grasp_reference = tuple(references)

    def _grasp_error(self, data, object_position, object_rotation):
        """Return world-frame hand errors relative to the live object pose."""
        if self.grasp_feedback_gain is None:
            return np.zeros(12)
        if self._grasp_reference is None:
            self.capture_grasp_reference(data)

        errors = []
        for site_name, (relative_position, relative_rotation) in zip(
            self.kinematics.hand_sites,
            self._grasp_reference,
        ):
            site_id = self.kinematics.model.site(site_name).id
            current_position = data.site_xpos[site_id]
            current_rotation = data.site_xmat[site_id].reshape(3, 3)
            expected_position = (
                object_position + object_rotation @ relative_position
            )
            expected_rotation = object_rotation @ relative_rotation
            errors.append(
                self.pose_error(
                    expected_position,
                    expected_rotation,
                    current_position,
                    current_rotation,
                )
            )
        return np.concatenate(errors)

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
                          + q_dot_grasp
                          + (I-J_H^dagger J_H)phi_dot_opt]dt

        ``q_dot_grasp`` is zero unless grasp feedback is enabled.
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

        # Equation (8) constrains instantaneous hand velocity, but numerical
        # integration, actuator lag, and contact compliance can still create a
        # position-level hand/object drift. Feed that accumulated relative-pose
        # error back through J_H; the object feedback rejects any small common
        # motion and the paper's J_H null-space optimization term below remains
        # unchanged.
        grasp_pose_error = self._grasp_error(
            data,
            current_position,
            current_rotation,
        )
        if self.grasp_feedback_gain is not None:
            hand_jacobian = self.kinematics.hand_jacobian(data)
            primary_joint_velocity += (
                np.linalg.pinv(
                    hand_jacobian,
                    rcond=self.control_pinv_rcond,
                )
                @ (self.grasp_feedback_gain @ grasp_pose_error)
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
            grasp_pose_error=grasp_pose_error,
            rank_tracking_map=int(np.linalg.matrix_rank(tracking_map)),
            condition_tracking_map=float(np.linalg.cond(tracking_map)),
            primary_joint_velocity=primary_joint_velocity,
            null_space_joint_velocity=null_space_joint_velocity,
        )
        return phi_next, diagnostics
