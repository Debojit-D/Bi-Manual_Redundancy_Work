"""Static Equation (8) optimization with selectable collision sphere models.

The table is grasped and held at its measured pose; it is not lifted.  The
selected paper objective generates ``phi_dot_opt``, while Equation (8)'s
primary term continuously holds the object's position and orientation.

Run from the repository root according to what you want to inspect::

    # Default experiment: fitted spheres and collision penalty.
    python -m MUJOCO.scripts.dual_franka_eq8_static_optimization

    # See the fitted collision spheres in the viewer.
    python -m MUJOCO.scripts.dual_franka_eq8_static_optimization \\
        --show-collision-spheres

    # Observe redundancy optimization without collision avoidance.
    python -m MUJOCO.scripts.dual_franka_eq8_static_optimization \\
        --disable-collision-penalty

    # Record every control step to an automatically named CSV.
    python -m MUJOCO.scripts.dual_franka_eq8_static_optimization --record-data

    # Record to a chosen path while viewing the fitted spheres.
    python -m MUJOCO.scripts.dual_franka_eq8_static_optimization \\
        --output-csv results/static_eq8.csv --show-collision-spheres

The collision model uses fitted sphere centers and radii from the dual-Franka
sphere MJCF. The collision response can be explored with
``--collision-weight``, ``--collision-safety-margin``, and
``--collision-proximity-scale``. Use ``--collision-sphere-model`` to load a
different fitted-sphere MJCF. Run with ``--help`` for the complete reference.
"""

import argparse
from pathlib import Path

import numpy as np
from loop_rate_limiters import RateLimiter

from MUJOCO.utils.grasping_kinematics import (
    CooperativeManipulationKinematics,
)
from MUJOCO.utils.data_recording import Equation8CSVRecorder
from MUJOCO.utils.redundancy_optimization import (
    Equation8Controller,
    ManipulabilityObjective,
    ManipulabilityOptimizer,
    draw_detailed_collision_spheres,
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
TABLE_SPAWN_POSITION = np.array([0.60, 0.0, 0.28])

# LEFT_ARM_SPAWN_POSITION = np.array([0.0, -0.5, 0.0])
# RIGHT_ARM_SPAWN_POSITION = np.array([0.0, 0.5, 0.0])

K_P = np.diag([8.0, 8.0, 8.0, 4.0, 4.0, 4.0])
GRASP_K_P = np.diag([8.0, 8.0, 8.0, 6.0, 6.0, 6.0])

# This collision-aware static test exercises force manipulability. Change this
# value to VELOCITY or DIRECTIONAL_FORCE for the other paper objectives.
OBJECTIVE = ManipulabilityObjective.FORCE
# The raw spatial-gradient scale is small; the joint-speed cap below remains
# the final safety limit after applying this Equation (4) gain Lambda.
OPTIMIZATION_GAIN = 5000.0
MAXIMUM_OPTIMIZATION_JOINT_SPEED = 5
FINITE_DIFFERENCE_STEP = 1e-4
JOINT_LIMIT_MARGIN = 0.05
JOINT_LIMIT_STOP_DISTANCE = 0.07
JOINT_LIMIT_SLOW_DISTANCE = 0.25

# MuJoCo-native coarse-sphere soft penalty. This discourages inter-arm
# proximity but is not a hard collision-proof planner.
ENABLE_COLLISION_PENALTY = True
# The fitted-sphere model uses a smooth proximity field. These values were
# calibrated on the
# full static force-optimization rollout: 5 cm is the desired sphere-surface
# gap and 1 cm controls how gradually the warning fades in before that gap.
COLLISION_WEIGHT = 5000.0
COLLISION_SAFETY_MARGIN = 0.05
COLLISION_PROXIMITY_SCALE = 0.01
COLLISION_SPHERE_MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "robot_descriptions"
    / "franka_emika_panda"
    / "dual_panda_robots_only_spherefit.xml"
)

# Used only by DIRECTIONAL_FORCE: world-frame [Fx, Fy, Fz, Mx, My, Mz].
DESIRED_WRENCH_DIRECTION = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
CHARACTERISTIC_LENGTH = 0.4


def run_static_optimization(
    scene,
    kinematics,
    equation_8,
    optimizer,
    viewer,
    rate,
    duration=None,
    recorder=None,
    show_collision_spheres=False,
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
        scene.command(phi, scene.gripper_closed)
        if show_collision_spheres:
            draw_detailed_collision_spheres(viewer, optimizer, scene.data)
        scene.step(viewer)
        if recorder is not None:
            recorder.record(
                desired_position,
                desired_rotation,
                optimization,
                diagnostics,
            )
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
                f"alpha={diagnostics.null_space_scale:.3f}, "
                f"joint margin="
                f"{diagnostics.minimum_joint_limit_distance:.4f} rad, "
                f"null leakage="
                f"{diagnostics.scaled_null_space_leakage:.2e}/"
                f"{diagnostics.unscaled_null_space_leakage:.2e}, "
                f"command speed="
                f"{np.max(np.abs(diagnostics.commanded_joint_velocity)):.4f} rad/s, "
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


def main(
    *,
    record_data=False,
    output_csv=None,
    collision_weight=COLLISION_WEIGHT,
    collision_safety_margin=COLLISION_SAFETY_MARGIN,
    collision_proximity_scale=COLLISION_PROXIMITY_SCALE,
    collision_sphere_model_path=COLLISION_SPHERE_MODEL_PATH,
    enable_collision_penalty=ENABLE_COLLISION_PENALTY,
    show_collision_spheres=False,
    optimization_gain=OPTIMIZATION_GAIN,
    maximum_joint_speed=MAXIMUM_OPTIMIZATION_JOINT_SPEED,
    joint_limit_margin=JOINT_LIMIT_MARGIN,
    joint_limit_stop_distance=JOINT_LIMIT_STOP_DISTANCE,
    joint_limit_slow_distance=JOINT_LIMIT_SLOW_DISTANCE,
):
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
    left_limits = scene.model.actuator_ctrlrange[0:7]
    right_limits = scene.model.actuator_ctrlrange[8:15]
    joint_position_lower = np.concatenate(
        (left_limits[:, 0], right_limits[:, 0])
    )
    joint_position_upper = np.concatenate(
        (left_limits[:, 1], right_limits[:, 1])
    )
    equation_8 = Equation8Controller(
        kinematics,
        control_dt=scene.control_dt,
        feedback_gain=K_P,
        grasp_feedback_gain=GRASP_K_P,
        joint_position_lower=joint_position_lower,
        joint_position_upper=joint_position_upper,
        joint_velocity_lower=-maximum_joint_speed,
        joint_velocity_upper=maximum_joint_speed,
        joint_limit_margin=joint_limit_margin,
        joint_limit_stop_distance=joint_limit_stop_distance,
        joint_limit_slow_distance=joint_limit_slow_distance,
    )
    optimizer = ManipulabilityOptimizer(
        kinematics,
        scene.arm_qpos,
        objective=OBJECTIVE,
        gain=optimization_gain,
        finite_difference_step=FINITE_DIFFERENCE_STEP,
        maximum_joint_speed=maximum_joint_speed,
        desired_wrench_direction=DESIRED_WRENCH_DIRECTION,
        characteristic_length=CHARACTERISTIC_LENGTH,
        enable_collision_penalty=enable_collision_penalty,
        collision_weight=collision_weight,
        collision_safety_margin=collision_safety_margin,
        collision_proximity_scale=collision_proximity_scale,
        collision_version="version2",
        collision_sphere_model_path=collision_sphere_model_path,
    )
    print(
        f"Collision model: {optimizer.collision_version.value}, "
        f"weight={optimizer.collision_weight:g}, "
        f"safety margin={optimizer.collision_safety_margin:.3f} m, "
        f"proximity scale={optimizer.collision_proximity_scale:.3f} m"
    )
    print(
        "Detailed spheres: "
        f"left={optimizer.left_detailed_radii.size}, "
        f"right={optimizer.right_detailed_radii.size}, "
        f"model={collision_sphere_model_path}"
    )
    if show_collision_spheres:
        visual_geoms = scene.model.geom_group == 2
        scene.model.geom_rgba[visual_geoms, 3] = 0.25
    rate = RateLimiter(frequency=CONTROL_HZ, warn=False)
    recorder = None
    if record_data or output_csv is not None:
        recorder = Equation8CSVRecorder(
            scene,
            kinematics,
            equation_8,
            optimizer,
            experiment_name="dual_franka_eq8_static_optimization_fitted_spheres",
            output_path=output_csv,
        )
        print(f"Recording data to: {recorder.output_path}")

    try:
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
                recorder=recorder,
                show_collision_spheres=show_collision_spheres,
            )
    except KeyboardInterrupt:
        print("Interrupted by Ctrl+C; preserving recorded samples.")
    finally:
        if recorder is not None:
            recorder.close()
            print(
                f"Saved {recorder.rows_written} rows to: "
                f"{recorder.output_path}"
            )


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--record-data",
        action="store_true",
        help="Record every Equation (8) control step to CSV.",
    )
    parser.add_argument(
        "--output-csv",
        help=(
            "Optional CSV path. Supplying it also enables recording; existing "
            "files receive a collision-safe numeric suffix."
        ),
    )
    parser.add_argument(
        "--collision-weight",
        type=float,
        default=COLLISION_WEIGHT,
        help="fitted-sphere collision cost weight",
    )
    parser.add_argument(
        "--collision-safety-margin",
        type=float,
        default=COLLISION_SAFETY_MARGIN,
        help="extra desired surface clearance in metres",
    )
    parser.add_argument(
        "--collision-proximity-scale",
        type=float,
        default=COLLISION_PROXIMITY_SCALE,
        help=(
            "smooth transition length in metres; smaller values "
            "make the penalty switch on more sharply near the safety margin"
        ),
    )
    parser.add_argument(
        "--collision-sphere-model",
        type=Path,
        default=COLLISION_SPHERE_MODEL_PATH,
        help="sphere-fitted dual-Franka MJCF",
    )
    parser.add_argument(
        "--disable-collision-penalty",
        action="store_true",
        help="disable the soft inter-arm collision term",
    )
    parser.add_argument(
        "--show-collision-spheres",
        action="store_true",
        help="draw fitted left/right spheres in the MuJoCo viewer",
    )
    parser.add_argument(
        "--optimization-gain",
        type=float,
        default=OPTIMIZATION_GAIN,
        help="Equation (4) null-space gradient gain Lambda",
    )
    parser.add_argument(
        "--max-joint-speed",
        type=float,
        default=MAXIMUM_OPTIMIZATION_JOINT_SPEED,
        help="symmetric simulation joint-velocity bound in rad/s",
    )
    parser.add_argument(
        "--joint-limit-margin",
        type=float,
        default=JOINT_LIMIT_MARGIN,
        help="reserved distance inside each position limit in radians",
    )
    parser.add_argument(
        "--joint-limit-stop-distance",
        type=float,
        default=JOINT_LIMIT_STOP_DISTANCE,
        help="distance in radians at which null-space motion is stopped",
    )
    parser.add_argument(
        "--joint-limit-slow-distance",
        type=float,
        default=JOINT_LIMIT_SLOW_DISTANCE,
        help="distance in radians at which null-space slowing begins",
    )
    return parser.parse_args()


if __name__ == "__main__":
    arguments = parse_arguments()
    main(
        record_data=arguments.record_data,
        output_csv=arguments.output_csv,
        collision_weight=arguments.collision_weight,
        collision_safety_margin=arguments.collision_safety_margin,
        collision_proximity_scale=arguments.collision_proximity_scale,
        collision_sphere_model_path=arguments.collision_sphere_model,
        enable_collision_penalty=not arguments.disable_collision_penalty,
        show_collision_spheres=arguments.show_collision_spheres,
        optimization_gain=arguments.optimization_gain,
        maximum_joint_speed=arguments.max_joint_speed,
        joint_limit_margin=arguments.joint_limit_margin,
        joint_limit_stop_distance=arguments.joint_limit_stop_distance,
        joint_limit_slow_distance=arguments.joint_limit_slow_distance,
    )
