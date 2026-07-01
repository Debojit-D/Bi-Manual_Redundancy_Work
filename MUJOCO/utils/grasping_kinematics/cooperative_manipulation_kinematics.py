"""Grasp and hand kinematics for cooperative manipulation."""

import mujoco
import numpy as np


class CooperativeManipulationKinematics:
    """Compute G, J_H, object velocity maps, and the Equation (8) projector."""

    def __init__(
        self,
        model,
        left_arm_dofs,
        right_arm_dofs,
        *,
        object_reference_site="site_top_middle",
        object_contact_sites=("site_left", "site_right"),
        hand_sites=("attachment_site_left", "attachment_site_right"),
        pinv_rcond=1e-6,
    ):
        self.model = model
        self.left_arm_dofs = np.asarray(left_arm_dofs, dtype=int)
        self.right_arm_dofs = np.asarray(right_arm_dofs, dtype=int)
        self.arm_dofs = np.concatenate(
            (self.left_arm_dofs, self.right_arm_dofs)
        )
        self.object_reference_site = object_reference_site
        self.object_contact_sites = tuple(object_contact_sites)
        self.hand_sites = tuple(hand_sites)
        self.pinv_rcond = float(pinv_rcond)

        if len(self.object_contact_sites) != 2 or len(self.hand_sites) != 2:
            raise ValueError("This dual-arm implementation expects two contacts")

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

    def grasp_matrix(self, data):
        """Return the 6x12 spatial full-contact grasp matrix G."""
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
        """Return the 12x14 block-diagonal hand Jacobian J_H."""
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
        """Return A=(G.T)^dagger J_H from Equations (1), (2), and (10)."""
        grasp_matrix = self.grasp_matrix(data)
        hand_jacobian = self.hand_jacobian(data)
        return (
            np.linalg.pinv(grasp_matrix.T, rcond=self.pinv_rcond)
            @ hand_jacobian
        )

    def tracking_object_velocity_map(self, data):
        """Return the compatibility-preserving map used for physical tracking.

        Solving J_H phi_dot=G.T q_dot gives

            phi_dot = J_H^dagger G.T q_dot = B q_dot.

        Defining A_control=B^dagger makes Equation (8)'s A_control^dagger
        recover that compatible minimum-norm joint motion.  This is kept
        separate from the paper map used by the manipulability objectives.
        """
        grasp_matrix = self.grasp_matrix(data)
        hand_jacobian = self.hand_jacobian(data)
        object_to_joint_map = (
            np.linalg.pinv(hand_jacobian, rcond=self.pinv_rcond)
            @ grasp_matrix.T
        )
        return np.linalg.pinv(object_to_joint_map, rcond=self.pinv_rcond)

    def null_space_projector(self, data):
        """Return I-J_H^dagger J_H from Equation (8)."""
        hand_jacobian = self.hand_jacobian(data)
        identity = np.eye(hand_jacobian.shape[1])
        return identity - (
            np.linalg.pinv(hand_jacobian, rcond=self.pinv_rcond)
            @ hand_jacobian
        )
