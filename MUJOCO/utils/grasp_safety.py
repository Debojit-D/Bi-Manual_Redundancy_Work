"""Object-based safety checks for loss of a rigid cooperative grasp."""

from dataclasses import dataclass

import numpy as np


OBJECT_LOSS_POSITION_THRESHOLD = 0.15
OBJECT_LOSS_ORIENTATION_THRESHOLD = np.deg2rad(40.0)


@dataclass(frozen=True)
class GraspLossStatus:
    """Maximum per-hand pose drift relative to the live object."""

    lost: bool
    maximum_position_error: float
    maximum_orientation_error: float


def assess_grasp_loss(
    grasp_pose_error,
    *,
    position_threshold=OBJECT_LOSS_POSITION_THRESHOLD,
    orientation_threshold=OBJECT_LOSS_ORIENTATION_THRESHOLD,
):
    """Assess loss from per-hand pose drift relative to the grasped object."""
    errors = np.asarray(grasp_pose_error, dtype=float)
    if errors.shape != (12,):
        raise ValueError("grasp_pose_error must contain two six-dimensional errors")
    if position_threshold <= 0.0 or orientation_threshold <= 0.0:
        raise ValueError("grasp-loss thresholds must be positive")
    per_hand_errors = errors.reshape(2, 6)
    maximum_position_error = float(
        np.max(np.linalg.norm(per_hand_errors[:, :3], axis=1))
    )
    maximum_orientation_error = float(
        np.max(np.linalg.norm(per_hand_errors[:, 3:], axis=1))
    )
    return GraspLossStatus(
        lost=(
            maximum_position_error > position_threshold
            or maximum_orientation_error > orientation_threshold
        ),
        maximum_position_error=maximum_position_error,
        maximum_orientation_error=maximum_orientation_error,
    )


def object_grasp_loss_status(
    grasp_pose_error,
    *,
    position_threshold=OBJECT_LOSS_POSITION_THRESHOLD,
    orientation_threshold=OBJECT_LOSS_ORIENTATION_THRESHOLD,
):
    """Return loss only when a hand drifts relative to the live object."""
    status = assess_grasp_loss(
        grasp_pose_error,
        position_threshold=position_threshold,
        orientation_threshold=orientation_threshold,
    )
    return status if status.lost else None


def print_grasp_loss(status):
    """Print the safety-stop reason and measured object/grasp drift."""
    print(
        "OBJECT SAFETY STOP: rigid grasp lost; "
        f"maximum hand/object position drift="
        f"{status.maximum_position_error:.4f} m, "
        f"orientation drift={status.maximum_orientation_error:.4f} rad."
    )
    print("Aborting the remaining trajectory and returning both arms home.")


def finish_after_grasped_motion(scene, viewer, rate, *, grasp_lost):
    """Disengage normally, or go directly home after a grasp-loss stop."""
    if not viewer.is_running():
        return False
    if grasp_lost:
        # The object is no longer safely held, so do not plan a post-grasp
        # retreat relative to its potentially disturbed pose.
        return scene.return_arms_home(viewer, rate)
    return scene.run_grasp_disengagement(viewer, rate)
