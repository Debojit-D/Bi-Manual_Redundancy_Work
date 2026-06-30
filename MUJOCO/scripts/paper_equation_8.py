"""Paper mathematics for cooperative manipulation and Equation (8).

This module contains no viewer, waypoint, actuator, or simulation-loop logic.
It owns the grasp matrix, hand Jacobian, object-velocity maps, pose feedback,
velocity-manipulability metric, and Equation (8) joint update.
"""

from dataclasses import dataclass

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


@dataclass(frozen=True)
class Equation8Diagnostics:
    """Quantities useful when comparing baseline and optimized executions."""

    pose_error: np.ndarray
    rank_tracking_map: int
    condition_tracking_map: float
    velocity_manipulability: float
    primary_joint_velocity: np.ndarray
    null_space_joint_velocity: np.ndarray


class CooperativeManipulationMath:
    """Spatial cooperative-manipulation mathematics used by the paper.

    The paper begins with

        J_H phi_dot = G.T q_dot

    and Equation (8) is implemented as

        phi_next = phi + [A^dagger(q_dot_d + K_p e)
                          + (I - J_H^dagger J_H) phi_dot_opt] dt.

    ``phi_dot_opt=None`` disables redundancy optimization and therefore gives
    the baseline controller requested for the current experiment.
    """

    def __init__(
        self,
        model,
        left_arm_dofs,
        right_arm_dofs,
        *,
        control_dt,
        feedback_gain,
        object_reference_site="site_top_middle",
        object_contact_sites=("site_left", "site_right"),
        hand_sites=("attachment_site_left", "attachment_site_right"),
        pinv_rcond=1e-6,
        control_pinv_rcond=1e-5,
    ):
        self.model = model
        self.left_arm_dofs = np.asarray(left_arm_dofs, dtype=int)
        self.right_arm_dofs = np.asarray(right_arm_dofs, dtype=int)
        self.arm_dofs = np.concatenate(
            (self.left_arm_dofs, self.right_arm_dofs)
        )
        self.control_dt = float(control_dt)
        self.feedback_gain = np.asarray(feedback_gain, dtype=float)
        self.object_reference_site = object_reference_site
        self.object_contact_sites = tuple(object_contact_sites)
        self.hand_sites = tuple(hand_sites)
        self.pinv_rcond = float(pinv_rcond)
        self.control_pinv_rcond = float(control_pinv_rcond)

        number_of_arm_joints = self.arm_dofs.size
        if self.feedback_gain.shape != (6, 6):
            raise ValueError("feedback_gain must be a 6x6 matrix")
        if self.left_arm_dofs.size + self.right_arm_dofs.size != number_of_arm_joints:
            raise ValueError("Invalid arm DoF arrays")

    @staticmethod
    def skew(vector):
        """Return the cross-product matrix of a three-vector."""
        x, y, z = np.asarray(vector, dtype=float)
        return np.array(
            [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]],
            dtype=float,
        )

    def object_pose(self, data):
        """Measure the object's world position and orientation."""
        site_id = self.model.site(self.object_reference_site).id
        position = data.site_xpos[site_id].copy()
        rotation = data.site_xmat[site_id].reshape(3, 3).copy()
        return position, rotation

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

    def grasp_matrix(self, data):
        """Return the paper's 6x12 spatial full-contact grasp matrix G."""
        object_position, _ = self.object_pose(data)
        blocks = []
        for site_name in self.object_contact_sites:
            site_id = self.model.site(site_name).id
            contact_offset = data.site_xpos[site_id] - object_position
            blocks.append(
                np.block(
                    [
                        [np.eye(3), np.zeros((3, 3))],
                        [self.skew(contact_offset), np.eye(3)],
                    ]
                )
            )
        return np.hstack(blocks)

    def _site_jacobian(self, data, site_name, dof_indices):
        jacobian = np.zeros((6, self.model.nv))
        mujoco.mj_jacSite(
            self.model,
            data,
            jacobian[:3],
            jacobian[3:],
            self.model.site(site_name).id,
        )
        return jacobian[:, dof_indices]

    def hand_jacobian(self, data):
        """Return the paper's 12x14 block-diagonal hand Jacobian J_H."""
        left = self._site_jacobian(
            data, self.hand_sites[0], self.left_arm_dofs
        )
        right = self._site_jacobian(
            data, self.hand_sites[1], self.right_arm_dofs
        )
        return np.block(
            [
                [left, np.zeros((6, self.right_arm_dofs.size))],
                [np.zeros((6, self.left_arm_dofs.size)), right],
            ]
        )

    def paper_object_velocity_map(self, data):
        """Return A=(G.T)^dagger J_H from Equations (1), (2), and (10).

        This is the map that must be used in the paper's velocity
        manipulability metric W=sqrt(det(A A.T)).
        """
        grasp_matrix = self.grasp_matrix(data)
        hand_jacobian = self.hand_jacobian(data)
        return (
            np.linalg.pinv(grasp_matrix.T, rcond=self.pinv_rcond)
            @ hand_jacobian
        )

    def tracking_object_velocity_map(self, data):
        """Return the stable map used by the physical friction-grasp baseline.

        Solving J_H phi_dot=G.T q_dot directly gives

            phi_dot = J_H^dagger G.T q_dot = B q_dot.

        Defining A_control=B^dagger makes Equation (8)'s A_control^dagger
        recover this compatible minimum-norm joint motion.  This tracking map
        is deliberately distinct from ``paper_object_velocity_map``, which is
        the map used to evaluate the paper's manipulability objective.
        """
        grasp_matrix = self.grasp_matrix(data)
        hand_jacobian = self.hand_jacobian(data)
        object_to_joint_map = (
            np.linalg.pinv(hand_jacobian, rcond=self.pinv_rcond)
            @ grasp_matrix.T
        )
        return np.linalg.pinv(object_to_joint_map, rcond=self.pinv_rcond)

    def velocity_manipulability(self, data):
        """Return W=sqrt(det(A A.T)) from Equation (13)."""
        velocity_map = self.paper_object_velocity_map(data)
        gram_matrix = velocity_map @ velocity_map.T
        sign, log_determinant = np.linalg.slogdet(gram_matrix)
        if sign <= 0:
            return 0.0
        return float(np.exp(0.5 * log_determinant))

    def null_space_projector(self, data):
        """Return I-J_H^dagger J_H, the projector used in Equation (8)."""
        hand_jacobian = self.hand_jacobian(data)
        identity = np.eye(hand_jacobian.shape[1])
        return identity - (
            np.linalg.pinv(hand_jacobian, rcond=self.pinv_rcond)
            @ hand_jacobian
        )

    def equation_8_update(
        self,
        data,
        phi,
        desired_position,
        desired_rotation,
        desired_twist,
        phi_dot_opt=None,
    ):
        """Evaluate one discrete Equation (8) update.

        Passing no ``phi_dot_opt`` makes the null-space contribution exactly
        zero.  Supplying a future objective gradient activates the second term
        without changing the baseline trajectory-tracking implementation.
        """
        phi = np.asarray(phi, dtype=float)
        desired_twist = np.asarray(desired_twist, dtype=float)
        number_of_joints = self.arm_dofs.size
        if phi.shape != (number_of_joints,):
            raise ValueError(
                f"Expected phi shape {(number_of_joints,)}, got {phi.shape}"
            )

        current_position, current_rotation = self.object_pose(data)
        error = self.pose_error(
            desired_position,
            desired_rotation,
            current_position,
            current_rotation,
        )
        closed_loop_twist = desired_twist + self.feedback_gain @ error

        tracking_map = self.tracking_object_velocity_map(data)
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
                raise ValueError(
                    "phi_dot_opt must have the same shape as phi"
                )
            null_space_joint_velocity = (
                self.null_space_projector(data) @ phi_dot_opt
            )

        phi_next = phi + (
            primary_joint_velocity + null_space_joint_velocity
        ) * self.control_dt

        diagnostics = Equation8Diagnostics(
            pose_error=error,
            rank_tracking_map=int(np.linalg.matrix_rank(tracking_map)),
            condition_tracking_map=float(np.linalg.cond(tracking_map)),
            velocity_manipulability=self.velocity_manipulability(data),
            primary_joint_velocity=primary_joint_velocity,
            null_space_joint_velocity=null_space_joint_velocity,
        )
        return phi_next, diagnostics
