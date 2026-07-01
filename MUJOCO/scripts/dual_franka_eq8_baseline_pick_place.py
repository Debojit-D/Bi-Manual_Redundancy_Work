"""Minimal dual-Franka demonstration of Equation (8), without optimization."""

import numpy as np
from loop_rate_limiters import RateLimiter

from cooperative_manipulation_kinematics import (
    CooperativeManipulationKinematics,
)
from dual_franka_mujoco_scene import DualFrankaMuJoCoScene
from equation_8_controller import Equation8Controller


# Experiment settings.  Scene implementation and paper mathematics live in
# their respective class files; this file only defines and runs Equation (8).
CONTROL_HZ = 50.0
LIFT_HEIGHT = 0.26
LIFT_DURATION = 6.0
SHOW_MOCAP_TARGETS = False
ENABLE_ARM_BIAS_COMPENSATION = True

# Robot base spawn locations [x, y, z] in the world frame.  These defaults
# reproduce the MJCF exactly; edit only these two vectors to reposition arms.
LEFT_ARM_SPAWN_POSITION = np.array([0.0, -0.2, 0.0])
RIGHT_ARM_SPAWN_POSITION = np.array([0.0, 0.2, 0.0])

K_P = np.diag([8.0, 8.0, 8.0, 2.0, 2.0, 2.0])


def lift_reference(initial_position, final_position, rotation, time):
    """Return a zero-end-velocity Cartesian lift reference."""
    ratio = np.clip(time / LIFT_DURATION, 0.0, 1.0)
    scale = 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5
    scale_rate = (
        30.0 * ratio**2 - 60.0 * ratio**3 + 30.0 * ratio**4
    ) / LIFT_DURATION
    displacement = final_position - initial_position
    desired_position = initial_position + scale * displacement
    desired_twist = np.concatenate(
        (scale_rate * displacement, np.zeros(3))
    )
    return desired_position, rotation, desired_twist


def run_equation_8(scene, kinematics, equation_8, viewer, rate):
    """Run Equation (8) with phi_dot_opt=0, then hold the final pose."""
    initial_position, desired_rotation = kinematics.object_pose(scene.data)
    final_position = initial_position + np.array([0.0, 0.0, LIFT_HEIGHT])
    phi = scene.arm_configuration()

    # Baseline: no manipulability gradient and no null-space optimization.
    phi_dot_opt = np.zeros(14)
    print("Starting Equation (8) baseline (null-space optimization disabled)...")

    number_of_steps = int(LIFT_DURATION * CONTROL_HZ) + 1
    for step in range(number_of_steps):
        time = min(step / CONTROL_HZ, LIFT_DURATION)
        desired_position, rotation, desired_twist = lift_reference(
            initial_position, final_position, desired_rotation, time
        )

        # Equation (8): primary tracking + projected optimization velocity.
        phi, diagnostics = equation_8.update(
            scene.data,
            phi,
            desired_position,
            rotation,
            desired_twist,
            phi_dot_opt,
        )
        phi = scene.clip_arm_configuration(phi)
        scene.command(phi, scene.gripper_closed)
        scene.step(viewer)
        rate.sleep()

        if step % int(CONTROL_HZ) == 0:
            print(
                f"  t={time:4.1f}s, position error="
                f"{np.linalg.norm(diagnostics.pose_error[:3]):.4f} m"
            )

    print("Lift complete. Equation (8) remains active; close the viewer to exit.")
    while viewer.is_running():
        phi, _ = equation_8.update(
            scene.data,
            phi,
            final_position,
            desired_rotation,
            np.zeros(6),
            phi_dot_opt,
        )
        phi = scene.clip_arm_configuration(phi)
        scene.command(phi, scene.gripper_closed)
        scene.step(viewer)
        rate.sleep()


def main():
    scene = DualFrankaMuJoCoScene(
        control_hz=CONTROL_HZ,
        left_arm_base_position=LEFT_ARM_SPAWN_POSITION,
        right_arm_base_position=RIGHT_ARM_SPAWN_POSITION,
        show_mocap_targets=SHOW_MOCAP_TARGETS,
        enable_bias_compensation=ENABLE_ARM_BIAS_COMPENSATION,
    )
    kinematics = CooperativeManipulationKinematics(
        scene.model,
        scene.left_arm_dofs,
        scene.right_arm_dofs,
    )
    equation_8 = Equation8Controller(
        kinematics,
        control_dt=scene.control_dt,
        feedback_gain=K_P,
    )
    rate = RateLimiter(frequency=CONTROL_HZ, warn=False)

    with scene.launch_viewer() as viewer:
        scene.configure_viewer_camera(viewer)
        scene.settle(viewer, rate)
        scene.run_grasp_approach(viewer, rate)
        print("Closing both grippers...")
        scene.close_grippers(viewer, rate)
        run_equation_8(scene, kinematics, equation_8, viewer, rate)


if __name__ == "__main__":
    main()
