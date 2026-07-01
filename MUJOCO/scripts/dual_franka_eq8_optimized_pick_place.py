"""Dual-Franka Equation (8) lift with null-space optimization enabled."""

import numpy as np
from loop_rate_limiters import RateLimiter

from MUJOCO.utils.grasping_kinematics import (
    CooperativeManipulationKinematics,
)
from MUJOCO.utils.redundancy_optimization import (
    Equation8Controller,
    ManipulabilityObjective,
    ManipulabilityOptimizer,
)
from MUJOCO.utils.scene_builder import DualFrankaMuJoCoScene


# Match the baseline experiment so the null-space term is the main difference.
CONTROL_HZ = 50.0
LIFT_HEIGHT = 0.26
LIFT_DURATION = 6.0
SHOW_MOCAP_TARGETS = False
ENABLE_ARM_BIAS_COMPENSATION = True
LEFT_ARM_SPAWN_POSITION = np.array([0.0, -0.2, 0.2])
RIGHT_ARM_SPAWN_POSITION = np.array([0.0, 0.2, 0.2])
K_P = np.diag([8.0, 8.0, 8.0, 4.0, 4.0, 4.0])
GRASP_K_P = np.diag([8.0, 8.0, 8.0, 6.0, 6.0, 6.0])

# Change this to FORCE or DIRECTIONAL_FORCE to test the other paper costs.
OBJECTIVE = ManipulabilityObjective.FORCE
OPTIMIZATION_GAIN = 100.0
MAXIMUM_OPTIMIZATION_JOINT_SPEED = 0.15
FINITE_DIFFERENCE_STEP = 1e-4

# MuJoCo-native coarse-sphere soft penalty. This discourages inter-arm
# proximity but is not a hard collision-proof planner.
ENABLE_COLLISION_PENALTY = True
COLLISION_WEIGHT = 1700.0
COLLISION_SAFETY_MARGIN = 0.1
COLLISION_SPHERE_RADIUS = 0.08

DESIRED_WRENCH_DIRECTION = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
CHARACTERISTIC_LENGTH = 0.4


def lift_reference(initial_position, final_position, rotation, time):
    """Return the same zero-end-velocity lift used by the baseline."""
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


def optimized_equation_8_step(
    scene,
    equation_8,
    optimizer,
    phi,
    desired_position,
    desired_rotation,
    desired_twist,
    viewer,
    rate,
):
    """Apply one primary-plus-null-space Equation (8) control step."""
    optimization = optimizer.optimization_velocity(scene.data)
    phi, diagnostics = equation_8.update(
        scene.data,
        phi,
        desired_position,
        desired_rotation,
        desired_twist,
        optimization.phi_dot_opt,
    )
    phi = scene.clip_arm_configuration(phi)
    scene.command(phi, scene.gripper_closed)
    scene.step(viewer)
    rate.sleep()
    return phi, optimization, diagnostics


def run_optimized_lift(
    scene,
    kinematics,
    equation_8,
    optimizer,
    viewer,
    rate,
    hold_duration=None,
):
    """Lift the object and keep optimization active at the final pose."""
    initial_position, desired_rotation = kinematics.object_pose(scene.data)
    final_position = initial_position + np.array([0.0, 0.0, LIFT_HEIGHT])
    phi = scene.arm_configuration()
    initial_objective = optimizer.value(scene.data)

    print(
        f"Starting optimized Equation (8) lift ({optimizer.objective.value}): "
        f"initial objective={initial_objective:.6g}"
    )
    number_of_steps = int(LIFT_DURATION * CONTROL_HZ) + 1
    for step in range(number_of_steps):
        time = min(step / CONTROL_HZ, LIFT_DURATION)
        desired_position, rotation, desired_twist = lift_reference(
            initial_position,
            final_position,
            desired_rotation,
            time,
        )
        phi, optimization, diagnostics = optimized_equation_8_step(
            scene,
            equation_8,
            optimizer,
            phi,
            desired_position,
            rotation,
            desired_twist,
            viewer,
            rate,
        )

        if step % int(CONTROL_HZ) == 0:
            print(
                f"  t={time:4.1f}s, objective={optimization.value:.6g}, "
                f"position error="
                f"{np.linalg.norm(diagnostics.pose_error[:3]):.5f} m, "
                f"grasp error="
                f"{np.linalg.norm(diagnostics.grasp_pose_error):.5f}, "
                f"null speed="
                f"{np.max(np.abs(diagnostics.null_space_joint_velocity)):.4f} rad/s, "
                f"min clearance="
                f"{optimizer.minimum_inter_arm_clearance(scene.data):.4f} m, "
                f"collision cost="
                f"{optimizer.inter_arm_collision_cost(scene.data):.6f}"
            )

    lift_objective = optimizer.value(scene.data)
    print(
        f"Lift complete: objective {initial_objective:.6g} -> "
        f"{lift_objective:.6g}. Optimization remains active; "
        "close the viewer to exit."
    )

    maximum_hold_steps = (
        None
        if hold_duration is None
        else int(hold_duration * CONTROL_HZ)
    )
    hold_step = 0
    while viewer.is_running() and (
        maximum_hold_steps is None or hold_step < maximum_hold_steps
    ):
        phi, optimization, diagnostics = optimized_equation_8_step(
            scene,
            equation_8,
            optimizer,
            phi,
            final_position,
            desired_rotation,
            np.zeros(6),
            viewer,
            rate,
        )
        if hold_step % int(CONTROL_HZ) == 0:
            print(
                f"  hold={hold_step / CONTROL_HZ:5.1f}s, "
                f"objective={optimization.value:.6g}, position error="
                f"{np.linalg.norm(diagnostics.pose_error[:3]):.5f} m, "
                f"grasp error="
                f"{np.linalg.norm(diagnostics.grasp_pose_error):.5f}, "
                f"min clearance="
                f"{optimizer.minimum_inter_arm_clearance(scene.data):.4f} m, "
                f"collision cost="
                f"{optimizer.inter_arm_collision_cost(scene.data):.6f}"
            )
        hold_step += 1

    return {
        "initial_position": initial_position,
        "final_position": final_position,
        "initial_objective": initial_objective,
        "final_objective": optimizer.value(scene.data),
    }


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
        grasp_feedback_gain=GRASP_K_P,
    )
    optimizer = ManipulabilityOptimizer(
        kinematics,
        scene.arm_qpos,
        objective=OBJECTIVE,
        gain=OPTIMIZATION_GAIN,
        finite_difference_step=FINITE_DIFFERENCE_STEP,
        maximum_joint_speed=MAXIMUM_OPTIMIZATION_JOINT_SPEED,
        desired_wrench_direction=DESIRED_WRENCH_DIRECTION,
        characteristic_length=CHARACTERISTIC_LENGTH,
        enable_collision_penalty=ENABLE_COLLISION_PENALTY,
        collision_weight=COLLISION_WEIGHT,
        collision_safety_margin=COLLISION_SAFETY_MARGIN,
        collision_sphere_radius=COLLISION_SPHERE_RADIUS,
    )
    rate = RateLimiter(frequency=CONTROL_HZ, warn=False)

    with scene.launch_viewer() as viewer:
        scene.configure_viewer_camera(viewer)
        scene.settle(viewer, rate)
        scene.run_grasp_approach(viewer, rate)
        print("Closing both grippers...")
        scene.close_grippers(viewer, rate)
        equation_8.capture_grasp_reference(scene.data)
        run_optimized_lift(
            scene,
            kinematics,
            equation_8,
            optimizer,
            viewer,
            rate,
        )


if __name__ == "__main__":
    main()
