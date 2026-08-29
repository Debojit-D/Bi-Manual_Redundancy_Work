"""Closed-loop cooperative controller implementing Equation (8)."""

from dataclasses import dataclass
import warnings

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
    unscaled_null_space_joint_velocity: np.ndarray
    commanded_joint_velocity: np.ndarray
    null_space_scale: float
    minimum_joint_limit_distance: float
    unscaled_null_space_leakage: float
    scaled_null_space_leakage: float


def _joint_vector(value, number_of_joints, name):
    """Return a scalar or joint-wise setting as a finite 1-D array."""
    array = np.asarray(value, dtype=float)
    if array.ndim == 0:
        array = np.full(number_of_joints, float(array))
    if array.shape != (number_of_joints,):
        raise ValueError(f"{name} must be scalar or shape {(number_of_joints,)}")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain only finite values")
    return array


def compute_nullspace_scale(
    phi,
    phidot_task,
    u,
    dt,
    q_min,
    q_max,
    qdot_min,
    qdot_max,
    margin,
    d_slow,
    d_stop,
):
    """Return one safe multiplier for the complete projected null-space term.

    Not a manuscript equation: an implementation-only joint-limit safety
    scalar ``alpha in [0, 1]`` applied to the null-space term
    ``(I-J_H^dagger J_H)phi_dot_opt`` of Equation (8). The primary task
    velocity is never modified. If it is already outside the position-derived
    or configured velocity bounds, no null-space motion can make the
    requested primary command safe in general, so this function warns and
    returns zero.
    """
    phi = np.asarray(phi, dtype=float)
    phidot_task = np.asarray(phidot_task, dtype=float)
    u = np.asarray(u, dtype=float)
    if phi.ndim != 1 or phidot_task.shape != phi.shape or u.shape != phi.shape:
        raise ValueError("phi, phidot_task, and u must have the same 1-D shape")
    if dt <= 0.0:
        raise ValueError("dt must be positive")
    if margin < 0.0:
        raise ValueError("margin must be nonnegative")
    if d_stop < 0.0 or d_slow <= d_stop:
        raise ValueError("Require 0 <= d_stop < d_slow")

    number_of_joints = phi.size
    q_min = _joint_vector(q_min, number_of_joints, "q_min")
    q_max = _joint_vector(q_max, number_of_joints, "q_max")
    qdot_min = _joint_vector(qdot_min, number_of_joints, "qdot_min")
    qdot_max = _joint_vector(qdot_max, number_of_joints, "qdot_max")
    if np.any(q_min + margin > q_max - margin):
        raise ValueError("joint-limit margin leaves an empty position range")
    if np.any(qdot_min > qdot_max):
        raise ValueError("qdot_min must not exceed qdot_max")

    lower_position_velocity = (q_min + margin - phi) / dt
    upper_position_velocity = (q_max - margin - phi) / dt
    lower = np.maximum(qdot_min, lower_position_velocity)
    upper = np.minimum(qdot_max, upper_position_velocity)
    tolerance = 1e-9

    primary_infeasible = np.any(phidot_task < lower - tolerance) or np.any(
        phidot_task > upper + tolerance
    )
    if primary_infeasible:
        warnings.warn(
            "Primary Equation (8) task velocity violates joint position or "
            "velocity bounds; disabling the null-space term for this step.",
            RuntimeWarning,
            stacklevel=2,
        )
        return 0.0

    alpha_low = 0.0
    alpha_high = 1.0
    for index in range(number_of_joints):
        if abs(u[index]) < tolerance:
            continue
        if u[index] > 0.0:
            alpha_low_i = (lower[index] - phidot_task[index]) / u[index]
            alpha_high_i = (upper[index] - phidot_task[index]) / u[index]
        else:
            alpha_low_i = (upper[index] - phidot_task[index]) / u[index]
            alpha_high_i = (lower[index] - phidot_task[index]) / u[index]
        alpha_low = max(alpha_low, alpha_low_i)
        alpha_high = min(alpha_high, alpha_high_i)

    if alpha_low > alpha_high:
        alpha_bound = 0.0
    else:
        alpha_bound = float(np.clip(alpha_high, 0.0, 1.0))

    distances = np.minimum(phi - q_min, q_max - phi)
    minimum_distance = float(np.min(distances))
    if minimum_distance <= d_stop:
        alpha_proximity = 0.0
    elif minimum_distance >= d_slow:
        alpha_proximity = 1.0
    else:
        ratio = (minimum_distance - d_stop) / (d_slow - d_stop)
        alpha_proximity = ratio * ratio * (3.0 - 2.0 * ratio)

    return float(np.clip(min(alpha_bound, alpha_proximity), 0.0, 1.0))


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
        joint_position_lower=None,
        joint_position_upper=None,
        joint_velocity_lower=None,
        joint_velocity_upper=None,
        joint_limit_margin=0.05,
        joint_limit_slow_distance=0.25,
        joint_limit_stop_distance=0.07,
    ):
        self.kinematics = kinematics
        self.control_dt = float(control_dt)
        self.feedback_gain = np.asarray(feedback_gain, dtype=float)
        self.control_pinv_rcond = float(control_pinv_rcond)
        self.grasp_feedback_gain = self._make_grasp_feedback_gain(
            grasp_feedback_gain
        )
        self.joint_limit_margin = float(joint_limit_margin)
        self.joint_limit_slow_distance = float(joint_limit_slow_distance)
        self.joint_limit_stop_distance = float(joint_limit_stop_distance)
        self._grasp_reference = None
        if self.feedback_gain.shape != (6, 6):
            raise ValueError("feedback_gain must be a 6x6 matrix")
        number_of_joints = self.kinematics.arm_dofs.size
        supplied_limits = (
            joint_position_lower,
            joint_position_upper,
            joint_velocity_lower,
            joint_velocity_upper,
        )
        if any(value is None for value in supplied_limits) and not all(
            value is None for value in supplied_limits
        ):
            raise ValueError("all joint position and velocity bounds are required")
        self.joint_limit_scaling_enabled = all(
            value is not None for value in supplied_limits
        )
        if self.joint_limit_scaling_enabled:
            self.joint_position_lower = _joint_vector(
                joint_position_lower, number_of_joints, "joint_position_lower"
            )
            self.joint_position_upper = _joint_vector(
                joint_position_upper, number_of_joints, "joint_position_upper"
            )
            self.joint_velocity_lower = _joint_vector(
                joint_velocity_lower, number_of_joints, "joint_velocity_lower"
            )
            self.joint_velocity_upper = _joint_vector(
                joint_velocity_upper, number_of_joints, "joint_velocity_upper"
            )

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

        phi_next = phi + [A_control^dagger(q_dot_d + K_p e)
                          + q_dot_grasp
                          + (I-J_H^dagger J_H)phi_dot_opt]dt

        Matches the manuscript's Equation (8)
        ``phi(t+dt) = phi(t) + A^dagger(q_dot_d + K_p e)dt
        + (I-J_H^dagger J_H)phi_dot_opt dt`` with two implementation notes:

        1. the trajectory-tracking pseudoinverse here uses
           ``A_control = tracking_object_velocity_map(data)``
           (``kinematics.tracking_object_velocity_map``), the
           compatibility-preserving map recovering ``phi_dot =
           J_H^dagger G^T q_dot``, not the paper's manipulability map
           ``A = (G^T)^dagger J_H`` (Eq. 10) used by the objectives in
           ``core.objectives``. Both solve the same constraint
           ``J_H phi_dot = G^T q_dot`` (Eq. 1, 9) but are different
           least-squares solutions of it;
        2. ``q_dot_grasp`` is an additional physical-drift correction term,
           zero unless grasp feedback is enabled (``grasp_feedback_gain``).
           It is not part of Equation (8) itself; it compensates for
           position-level hand/object drift from numerical integration,
           actuator lag, and contact compliance that Equation (8)'s
           instantaneous-velocity form does not model.

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

        hand_jacobian = self.kinematics.hand_jacobian(data)
        if phi_dot_opt is None:
            unscaled_null_space_joint_velocity = np.zeros(number_of_joints)
        else:
            phi_dot_opt = np.asarray(phi_dot_opt, dtype=float)
            if phi_dot_opt.shape != (number_of_joints,):
                raise ValueError("phi_dot_opt must have the same shape as phi")
            unscaled_null_space_joint_velocity = (
                self.kinematics.null_space_projector(data) @ phi_dot_opt
            )

        if self.joint_limit_scaling_enabled:
            null_space_scale = compute_nullspace_scale(
                phi,
                primary_joint_velocity,
                unscaled_null_space_joint_velocity,
                self.control_dt,
                self.joint_position_lower,
                self.joint_position_upper,
                self.joint_velocity_lower,
                self.joint_velocity_upper,
                self.joint_limit_margin,
                self.joint_limit_slow_distance,
                self.joint_limit_stop_distance,
            )
            joint_distances = np.minimum(
                phi - self.joint_position_lower,
                self.joint_position_upper - phi,
            )
            minimum_joint_limit_distance = float(np.min(joint_distances))
        else:
            null_space_scale = 1.0
            minimum_joint_limit_distance = float("inf")

        null_space_joint_velocity = (
            null_space_scale * unscaled_null_space_joint_velocity
        )
        commanded_joint_velocity = (
            primary_joint_velocity + null_space_joint_velocity
        )
        phi_next = phi + commanded_joint_velocity * self.control_dt
        unscaled_null_space_leakage = float(
            np.linalg.norm(hand_jacobian @ unscaled_null_space_joint_velocity)
        )
        scaled_null_space_leakage = float(
            np.linalg.norm(hand_jacobian @ null_space_joint_velocity)
        )
        diagnostics = Equation8Diagnostics(
            pose_error=error,
            grasp_pose_error=grasp_pose_error,
            rank_tracking_map=int(np.linalg.matrix_rank(tracking_map)),
            condition_tracking_map=float(np.linalg.cond(tracking_map)),
            primary_joint_velocity=primary_joint_velocity,
            null_space_joint_velocity=null_space_joint_velocity,
            unscaled_null_space_joint_velocity=(
                unscaled_null_space_joint_velocity
            ),
            commanded_joint_velocity=commanded_joint_velocity,
            null_space_scale=null_space_scale,
            minimum_joint_limit_distance=minimum_joint_limit_distance,
            unscaled_null_space_leakage=unscaled_null_space_leakage,
            scaled_null_space_leakage=scaled_null_space_leakage,
        )
        return phi_next, diagnostics
