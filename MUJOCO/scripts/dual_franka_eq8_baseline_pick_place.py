"""Baseline Equation (8) lift, replace, disengage, and return-home demo."""

import numpy as np
from loop_rate_limiters import RateLimiter

from MUJOCO.utils.grasping_kinematics import (
    CooperativeManipulationKinematics,
)
from MUJOCO.utils.redundancy_optimization import Equation8Controller
from MUJOCO.utils.scene_builder import DualFrankaMuJoCoScene


# Experiment settings.  Scene implementation and paper mathematics live in
# their respective class files; this file only defines and runs Equation (8).
CONTROL_HZ = 50.0
LIFT_HEIGHT = 0.26
LIFT_DURATION = 6.0
LOWER_DURATION = 6.0
FINAL_HOLD_DURATION = 2.0
SHOW_MOCAP_TARGETS = False
ENABLE_ARM_BIAS_COMPENSATION = True

# Robot base poses: world xyz [m] and extrinsic XYZ Euler angles [degrees].
LEFT_ARM_SPAWN_POSITION = np.array([0.0, -0.2, 0.0])
RIGHT_ARM_SPAWN_POSITION = np.array([0.0, 0.2, 0.0])
LEFT_ARM_SPAWN_EULER_XYZ_DEGREES = np.array([0.0, 0.0, 0.0])
RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES = np.array([0.0, 0.0, 0.0])

K_P = np.diag([8.0, 8.0, 8.0, 2.0, 2.0, 2.0])


def _quintic_segment(start_position, goal_position, time, duration):
    """Return position and velocity for a zero-end-velocity segment."""
    ratio = np.clip(time / duration, 0.0, 1.0)
    scale = 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5
    scale_rate = (
        30.0 * ratio**2 - 60.0 * ratio**3 + 30.0 * ratio**4
    ) / duration
    displacement = goal_position - start_position
    return (
        start_position + scale * displacement,
        scale_rate * displacement,
    )


def lift_and_lower_reference(
    initial_position,
    lifted_position,
    rotation,
    time,
):
    """Return the baseline lift-and-return Cartesian reference."""
    if time <= LIFT_DURATION:
        desired_position, linear_velocity = _quintic_segment(
            initial_position,
            lifted_position,
            time,
            LIFT_DURATION,
        )
    else:
        desired_position, linear_velocity = _quintic_segment(
            lifted_position,
            initial_position,
            time - LIFT_DURATION,
            LOWER_DURATION,
        )
    desired_twist = np.concatenate((linear_velocity, np.zeros(3)))
    return desired_position, rotation, desired_twist


def run_equation_8(scene, kinematics, equation_8, viewer, rate):
    """Lift, replace, and briefly hold with phi_dot_opt=0."""
    initial_position, desired_rotation = kinematics.object_pose(scene.data)
    lifted_position = initial_position + np.array([0.0, 0.0, LIFT_HEIGHT])
    phi = scene.arm_configuration()

    # Baseline: no manipulability gradient and no null-space optimization.
    phi_dot_opt = np.zeros(14)
    print("Starting Equation (8) baseline (null-space optimization disabled)...")

    total_duration = LIFT_DURATION + LOWER_DURATION
    number_of_steps = int(total_duration * CONTROL_HZ) + 1
    for step in range(number_of_steps):
        if not viewer.is_running():
            return False
        time = min(step / CONTROL_HZ, total_duration)
        desired_position, rotation, desired_twist = lift_and_lower_reference(
            initial_position,
            lifted_position,
            desired_rotation,
            time,
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
            phase = "lifting" if time < LIFT_DURATION else "lowering"
            print(
                f"  {phase} t={time:4.1f}s, position error="
                f"{np.linalg.norm(diagnostics.pose_error[:3]):.4f} m"
            )

    print("Lift-and-return complete. Briefly holding the placed pose...")
    for _ in range(int(FINAL_HOLD_DURATION * CONTROL_HZ)):
        if not viewer.is_running():
            return False
        phi, _ = equation_8.update(
            scene.data,
            phi,
            initial_position,
            desired_rotation,
            np.zeros(6),
            phi_dot_opt,
        )
        phi = scene.clip_arm_configuration(phi)
        scene.command(phi, scene.gripper_closed)
        scene.step(viewer)
        rate.sleep()
    return True


def main():
    scene = DualFrankaMuJoCoScene(
        control_hz=CONTROL_HZ,
        left_arm_base_position=LEFT_ARM_SPAWN_POSITION,
        right_arm_base_position=RIGHT_ARM_SPAWN_POSITION,
        left_arm_base_euler_xyz_degrees=LEFT_ARM_SPAWN_EULER_XYZ_DEGREES,
        right_arm_base_euler_xyz_degrees=RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES,
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
        completed = run_equation_8(
            scene,
            kinematics,
            equation_8,
            viewer,
            rate,
        )
        if completed and viewer.is_running():
            scene.run_grasp_disengagement(viewer, rate)


if __name__ == "__main__":
    main()
