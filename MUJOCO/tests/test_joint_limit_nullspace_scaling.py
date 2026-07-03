import numpy as np
import unittest

from MUJOCO.utils.redundancy_optimization import compute_nullspace_scale


def scale_for(
    phi,
    task,
    null,
    *,
    dt=0.1,
    q_min=-1.0,
    q_max=1.0,
    qdot_min=-2.0,
    qdot_max=2.0,
    margin=0.0,
    d_stop=0.1,
    d_slow=0.3,
):
    return compute_nullspace_scale(
        np.asarray(phi, dtype=float),
        np.asarray(task, dtype=float),
        np.asarray(null, dtype=float),
        dt,
        q_min,
        q_max,
        qdot_min,
        qdot_max,
        margin,
        d_slow,
        d_stop,
    )


class NullspaceScalingTests(unittest.TestCase):
    def test_far_from_limits_keeps_full_nullspace_motion(self):
        alpha = scale_for([0.0, 0.0], [0.1, -0.1], [0.2, 0.3])
        self.assertAlmostEqual(alpha, 1.0)

    def test_slow_zone_uses_smoothstep_attenuation(self):
        alpha = scale_for([0.8], [0.0], [0.2])
        self.assertAlmostEqual(alpha, 0.5)

    def test_stop_zone_disables_nullspace_motion(self):
        alpha = scale_for([0.95], [0.0], [-0.2])
        self.assertAlmostEqual(alpha, 0.0)

    def test_scalar_scale_respects_joint_velocity_bound(self):
        alpha = scale_for(
            [0.0],
            [0.1],
            [1.0],
            qdot_min=-0.4,
            qdot_max=0.4,
        )
        self.assertAlmostEqual(alpha, 0.3)
        self.assertLessEqual(0.1 + alpha, 0.4 + 1e-12)

    def test_scalar_scale_preserves_nullspace_and_position_bounds(self):
        hand_jacobian = np.array([[1.0, 1.0]])
        projector = (
            np.eye(2) - np.linalg.pinv(hand_jacobian) @ hand_jacobian
        )
        null_velocity = projector @ np.array([1.0, -0.2])
        phi = np.array([0.85, -0.85])
        task_velocity = np.array([0.1, -0.1])
        dt = 0.1
        alpha = scale_for(
            phi,
            task_velocity,
            null_velocity,
            dt=dt,
            margin=0.1,
            d_stop=0.0,
            d_slow=0.01,
        )
        scaled_null_velocity = alpha * null_velocity
        phi_next = phi + dt * (task_velocity + scaled_null_velocity)

        self.assertLessEqual(
            np.linalg.norm(hand_jacobian @ scaled_null_velocity), 1e-12
        )
        self.assertTrue(np.all(phi_next >= -0.9 - 1e-12))
        self.assertTrue(np.all(phi_next <= 0.9 + 1e-12))

    def test_primary_task_infeasibility_warns_and_disables_nullspace(self):
        with self.assertWarnsRegex(RuntimeWarning, "Primary Equation.*violates"):
            alpha = scale_for(
                [0.0],
                [3.0],
                [0.1],
                qdot_min=-1.0,
                qdot_max=1.0,
            )
        self.assertAlmostEqual(alpha, 0.0)


if __name__ == "__main__":
    unittest.main()
