"""Static dual-Franka Equation (8) null-space optimization experiment.

The table is grasped and held at its measured pose.  It is not lifted.  The
selected paper objective generates phi_dot_opt while Equation (8)'s primary
term continuously holds the object's position and orientation.
"""

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


CONTROL_HZ = 50.0
SHOW_MOCAP_TARGETS = False
ENABLE_ARM_BIAS_COMPENSATION = True

# Robot base poses: world xyz [m] and extrinsic XYZ Euler angles [degrees].
LEFT_ARM_SPAWN_POSITION = np.array([0.0, -0.25, 0.0])
RIGHT_ARM_SPAWN_POSITION = np.array([0.0, 0.25, 0.0])
LEFT_ARM_SPAWN_EULER_XYZ_DEGREES = np.array([0.0, 0.0, 0.0])
RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES = np.array([0.0, 0.0, 0.0])

# World position [x, y, z] of the table's site_top_middle reference frame.
TABLE_SPAWN_POSITION = np.array([0.30, 0.0, 0.28])

# LEFT_ARM_SPAWN_POSITION = np.array([0.0, -0.5, 0.0])
# RIGHT_ARM_SPAWN_POSITION = np.array([0.0, 0.5, 0.0])

K_P = np.diag([8.0, 8.0, 8.0, 4.0, 4.0, 4.0])
GRASP_K_P = np.diag([8.0, 8.0, 8.0, 6.0, 6.0, 6.0])

# This collision-aware static test exercises force manipulability. Change this
# value to VELOCITY or DIRECTIONAL_FORCE for the other paper objectives.
OBJECTIVE = ManipulabilityObjective.DIRECTIONAL_FORCE
# The raw spatial-gradient scale is small; the joint-speed cap below remains
# the final safety limit after applying this Equation (4) gain Lambda.
OPTIMIZATION_GAIN = 100.0
MAXIMUM_OPTIMIZATION_JOINT_SPEED = 5
FINITE_DIFFERENCE_STEP = 1e-4

# MuJoCo-native coarse-sphere soft penalty. This discourages inter-arm
# proximity but is not a hard collision-proof planner.
ENABLE_COLLISION_PENALTY = True
COLLISION_WEIGHT = 1700.0
# COLLISION_SAFETY_MARGIN = 0.16
# COLLISION_SPHERE_RADIUS = 0.14
COLLISION_SAFETY_MARGIN = 0.009
COLLISION_SPHERE_RADIUS = 0.007

# Used only by DIRECTIONAL_FORCE: world-frame [Fx, Fy, Fz, Mx, My, Mz].
DESIRED_WRENCH_DIRECTION = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
CHARACTERISTIC_LENGTH = 0.4


def run_static_optimization(
    scene,
    kinematics,
    equation_8,
    optimizer,
    viewer,
    rate,
    duration=None,
):
    """Hold q_d constant while continuously applying the null-space term."""
    desired_position, desired_rotation = kinematics.object_pose(scene.data)
    phi = scene.arm_configuration()
    initial_value = optimizer.value(scene.data)
    maximum_steps = None if duration is None else int(duration * CONTROL_HZ)
    step = 0

    print(
        f"Starting static {optimizer.objective.value} optimization: "
        f"initial objective={initial_value:.6g}"
    )
    while viewer.is_running() and (
        maximum_steps is None or step < maximum_steps
    ):
        optimization = optimizer.optimization_velocity(scene.data)

        # Equation (8), now including (I-J_H^dagger J_H) phi_dot_opt.
        phi, diagnostics = equation_8.update(
            scene.data,
            phi,
            desired_position,
            desired_rotation,
            np.zeros(6),
            optimization.phi_dot_opt,
        )
        phi = scene.clip_arm_configuration(phi)
        scene.command(phi, scene.gripper_closed)
        scene.step(viewer)
        rate.sleep()

        if step % int(CONTROL_HZ) == 0:
            print(
                f"  t={step / CONTROL_HZ:5.1f}s, "
                f"objective={optimization.value:.6g}, "
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
        step += 1

    final_value = optimizer.value(scene.data)
    print(
        f"Static optimization finished: {initial_value:.6g} -> "
        f"{final_value:.6g}"
    )
    return initial_value, final_value


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
    scene.set_table_reference_pose(TABLE_SPAWN_POSITION)
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
        run_static_optimization(
            scene,
            kinematics,
            equation_8,
            optimizer,
            viewer,
            rate,
        )


if __name__ == "__main__":
    main()
