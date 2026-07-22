"""Visualize collision-aware Equation (8) during a simple pick-and-place task.

Both Franka arms grasp the table, lift it smoothly, and place it back at its
measured pickup pose.  Throughout the motion, Equation (8)'s primary term
tracks the object's position and orientation while the selected paper
objective generates ``phi_dot_opt`` in the null space.  After placement, the
controller briefly holds the returned pose, opens both grippers, retreats to
a table-relative post-grasp pose, and returns both arms home.

Run from the repository root according to what you want to inspect::

    # Default moving experiment: fitted spheres and collision cost.
    python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place

    # Run one of the six predefined comparison positions.
    python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place --position 1
    python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place --position 2

    # Position map [x, y, z] metres:
    # 1=(0.30, 0.20, 0.28), 2=(0.60, 0.20, 0.28)
    # 3=(0.30, 0.00, 0.28), 4=(0.60, 0.00, 0.28)
    # 5=(0.30,-0.20, 0.28), 6=(0.60,-0.20, 0.28)

    # Or supply any custom site_top_middle world position [x y z] in metres.
    python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place \\
        --table-spawn-position 0.45 -0.10 0.28

    # Visualize the fitted collision spheres throughout pick and place.
    python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place \\
        --show-table-collision-spheres

    # Select a front or overhead interactive camera.
    python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place --front-view
    python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place --top-view

    # Compare against optimization without the soft collision penalty.
    python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place \\
        --disable-table-collision-penalty

    # Record every moving and final-hold control step to an automatic CSV.
    python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place --record-data

    # Record to a chosen file while visualizing the fitted spheres.
    python -m MUJOCO.scripts.dual_franka_eq8_optimized_pick_place \\
        --output-csv results/optimized_pick_place.csv \\
        --show-collision-spheres

Inter-arm and non-adjacent same-arm collision use fitted sphere-to-sphere
clearance. Table collision uses fitted spheres on arm links 1--7 against the
oriented tabletop plane; hands and fingers are excluded so grasp contact
remains possible. All costs act only through the Equation (8) null-space
objective. A different fitted-sphere MJCF can be supplied with
``--collision-sphere-model``.

The collision term is a soft optimization cost, not a hard collision
guarantee. Press Ctrl+C at any time to close active resources and stop cleanly.
Run with ``--help`` for the complete option reference.
"""

import argparse
from contextlib import nullcontext
from pathlib import Path

import numpy as np
from loop_rate_limiters import RateLimiter

from MUJOCO.utils.grasping_kinematics import (
    CooperativeManipulationKinematics,
)
from MUJOCO.utils.data_recording import Equation8CSVRecorder
from MUJOCO.utils.cli import add_camera_view_arguments, run_cli
from MUJOCO.utils.control_timing import timed_equation_8_update
from MUJOCO.utils.grasp_safety import (
    finish_after_grasped_motion,
    object_grasp_loss_status,
    print_grasp_loss,
)
from MUJOCO.utils.redundancy_optimization import (
    draw_detailed_collision_spheres,
    draw_table_collision_spheres,
    Equation8Controller,
    ManipulabilityObjective,
    ManipulabilityOptimizer,
)
from MUJOCO.utils.scene_builder import DualFrankaMuJoCoScene
from MUJOCO.utils.video_recording import TqdmSimulationRate
from MUJOCO.scripts.table_spawn_comparison_positions import (
    TABLE_SPAWN_CASES,
    table_spawn_position_for_number,
)
# TABLE_COLLISION_SAFETY_MARGIN = 0.015


# Match the baseline experiment so the null-space term is the main difference.
CONTROL_HZ = 50.0
LIFT_HEIGHT = 0.26
LIFT_DURATION = 12.0
LOWER_DURATION = 12.0
FINAL_HOLD_DURATION = 2.0
SHOW_MOCAP_TARGETS = False
ENABLE_ARM_BIAS_COMPENSATION = True
# Robot base poses: world xyz [m] and extrinsic XYZ Euler angles [degrees].
LEFT_ARM_SPAWN_POSITION = np.array([0.0, -0.25, 0.0])
RIGHT_ARM_SPAWN_POSITION = np.array([0.0, 0.25, 0.0])
LEFT_ARM_SPAWN_EULER_XYZ_DEGREES = np.array([0.0, 0.0, 0.0])
RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES = np.array([0.0, 0.0, 0.0])

# Default world position [x, y, z] of the table's site_top_middle frame.
TABLE_SPAWN_POSITION = np.array([0.30, 0.15, 0.28])
K_P = np.diag([8.0, 8.0, 8.0, 4.0, 4.0, 4.0])
GRASP_K_P = np.diag([8.0, 8.0, 8.0, 6.0, 6.0, 6.0])

# Match the static experiment so the trajectory is the only controller-level
# difference between the two scripts.
OBJECTIVE = ManipulabilityObjective.FORCE
OPTIMIZATION_GAIN = 5000.0
MAXIMUM_OPTIMIZATION_JOINT_SPEED = 5
FINITE_DIFFERENCE_STEP = 1e-4
JOINT_LIMIT_MARGIN = 0.05
JOINT_LIMIT_STOP_DISTANCE = 0.07
JOINT_LIMIT_SLOW_DISTANCE = 0.25

ENABLE_COLLISION_PENALTY = True
ENABLE_TABLE_COLLISION_PENALTY = True
ENABLE_SELF_COLLISION_PENALTY = True
# The fitted-sphere model uses a smooth proximity field calibrated on the
# full static force-optimization rollout: 5 cm is the desired sphere-surface
# gap and 1 cm controls how gradually the warning fades in before that gap.
COLLISION_WEIGHT = 5000.0
COLLISION_SAFETY_MARGIN = 0.05
COLLISION_PROXIMITY_SCALE = 0.01
TABLE_COLLISION_WEIGHT = 20000.0
# Keep table avoidance local: maintain 1.5 cm clearance and let its smooth
# influence decay over 3 mm so it does not dominate the paper objective.
TABLE_COLLISION_SAFETY_MARGIN = 0.05
TABLE_COLLISION_PROXIMITY_SCALE = 0.003
TABLE_COLLISION_GEOM_NAME = None
# Keep same-arm avoidance equally local. Same-body, directly adjacent, and
# internal gripper/link7 pairs are excluded from the full fitted-sphere model.
SELF_COLLISION_WEIGHT = 20000.0
SELF_COLLISION_SAFETY_MARGIN = 0.01
SELF_COLLISION_PROXIMITY_SCALE = 0.003
COLLISION_SPHERE_MODEL_PATH = (
    Path(__file__).resolve().parents[1]
    / "robot_descriptions"
    / "franka_emika_panda"
    / "dual_panda_robots_only_spherefit.xml"
)

DESIRED_WRENCH_DIRECTION = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])


def quintic_segment_reference(
    start_position,
    goal_position,
    rotation,
    time,
    duration,
):
    """Return a zero-end-velocity Cartesian segment reference."""
    ratio = np.clip(time / duration, 0.0, 1.0)
    scale = 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5
    scale_rate = (
        30.0 * ratio**2 - 60.0 * ratio**3 + 30.0 * ratio**4
    ) / duration
    displacement = goal_position - start_position
    desired_position = start_position + scale * displacement
    desired_twist = np.concatenate(
        (scale_rate * displacement, np.zeros(3))
    )
    return desired_position, rotation, desired_twist


def lift_and_lower_reference(
    initial_position,
    lifted_position,
    rotation,
    time,
):
    """Lift to the apex, then smoothly return to the initial pose."""
    if time <= LIFT_DURATION:
        return quintic_segment_reference(
            initial_position,
            lifted_position,
            rotation,
            time,
            LIFT_DURATION,
        )
    return quintic_segment_reference(
        lifted_position,
        initial_position,
        rotation,
        time - LIFT_DURATION,
        LOWER_DURATION,
    )


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
    trajectory_phase="unspecified",
    trajectory_time=0.0,
    recorder=None,
    show_collision_spheres=False,
    show_table_collision_spheres=False,
    enable_redundancy_optimization=True,
):
    """Apply one primary-plus-null-space Equation (8) control step."""
    phi, optimization, diagnostics, timing = timed_equation_8_update(
        scene,
        equation_8,
        optimizer,
        phi,
        desired_position,
        desired_rotation,
        desired_twist,
        enable_redundancy_optimization=enable_redundancy_optimization,
    )
    scene.command(phi, scene.gripper_closed)
    if show_collision_spheres:
        draw_detailed_collision_spheres(viewer, optimizer, scene.data)
    if show_table_collision_spheres:
        draw_table_collision_spheres(
            viewer,
            optimizer,
            scene.data,
            reset_scene=not show_collision_spheres,
        )
    scene.step(viewer)
    if recorder is not None:
        recorder.record(
            desired_position,
            desired_rotation,
            optimization,
            diagnostics,
            desired_twist=desired_twist,
            trajectory_phase=trajectory_phase,
            trajectory_time=trajectory_time,
            optimizer_time_ms=timing.optimizer_time_ms,
            controller_update_time_ms=timing.controller_update_time_ms,
            control_compute_time_ms=timing.control_compute_time_ms,
        )
    rate.sleep()
    return phi, optimization, diagnostics


def run_optimized_lift(
    scene,
    kinematics,
    equation_8,
    optimizer,
    viewer,
    rate,
    hold_duration=FINAL_HOLD_DURATION,
    recorder=None,
    show_collision_spheres=False,
    show_table_collision_spheres=False,
    enable_redundancy_optimization=True,
):
    """Lift, lower, then keep optimizing at the returned grasped pose."""
    initial_position, desired_rotation = kinematics.object_pose(scene.data)
    lifted_position = initial_position + np.array([0.0, 0.0, LIFT_HEIGHT])
    phi = scene.arm_configuration()
    initial_objective = optimizer.value(scene.data)

    mode = (
        optimizer.objective.value
        if enable_redundancy_optimization
        else "baseline"
    )
    print(
        f"Starting Equation (8) lift-and-lower ({mode}): "
        f"initial objective={initial_objective:.6g}"
    )
    total_duration = LIFT_DURATION + LOWER_DURATION
    number_of_steps = int(total_duration * CONTROL_HZ) + 1
    apex_objective = None
    motion_completed = True
    grasp_lost = False
    for step in range(number_of_steps):
        if not viewer.is_running():
            motion_completed = False
            break
        time = min(step / CONTROL_HZ, total_duration)
        desired_position, rotation, desired_twist = lift_and_lower_reference(
            initial_position,
            lifted_position,
            desired_rotation,
            time,
        )
        trajectory_phase = (
            "lifting" if time < LIFT_DURATION else "lowering"
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
            trajectory_phase=trajectory_phase,
            trajectory_time=time,
            recorder=recorder,
            show_collision_spheres=show_collision_spheres,
            show_table_collision_spheres=show_table_collision_spheres,
            enable_redundancy_optimization=enable_redundancy_optimization,
        )
        loss_status = object_grasp_loss_status(
            diagnostics.grasp_pose_error,
        )
        if loss_status is not None:
            print_grasp_loss(loss_status)
            grasp_lost = True
            motion_completed = False
            break

        if apex_objective is None and time >= LIFT_DURATION:
            apex_objective = optimizer.value(scene.data)
            print(
                f"Lift apex reached at {LIFT_HEIGHT:.3f} m; "
                "starting smooth return."
            )

        if step % int(CONTROL_HZ) == 0:
            phase = "lifting" if time < LIFT_DURATION else "lowering"
            print(
                f"  {phase} t={time:4.1f}s, "
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
                f"inter-arm clearance="
                f"{optimizer.minimum_inter_arm_clearance(scene.data):.4f} m, "
                f"inter-arm cost="
                f"{optimizer.inter_arm_collision_cost(scene.data):.6f}, "
                f"table clearance="
                f"{optimizer.minimum_arm_table_clearance(scene.data):.4f} m, "
                f"table cost="
                f"{optimizer.arm_table_collision_cost(scene.data):.6f}, "
                f"self clearance="
                f"{optimizer.minimum_self_collision_clearance(scene.data):.4f} m, "
                f"self cost="
                f"{optimizer.self_collision_cost(scene.data):.6f}, "
                f"weighted collision cost="
                f"{optimizer.total_collision_cost(scene.data):.6f}"
            )

    returned_objective = optimizer.value(scene.data)
    if not grasp_lost and motion_completed:
        hold_activity = (
            "optimization active"
            if enable_redundancy_optimization
            else "null-space optimization disabled"
        )
        hold_description = (
            "until the viewer closes"
            if hold_duration is None
            else f"for {hold_duration:g} s before disengagement"
        )
        print(
            f"Lift-and-lower complete: objective {initial_objective:.6g} -> "
            f"{returned_objective:.6g}. Holding the returned pose with "
            f"{hold_activity} {hold_description}."
        )
    elif not grasp_lost:
        print("Viewer closed before the lift-and-lower motion completed.")

    maximum_hold_steps = (
        None
        if hold_duration is None
        else int(hold_duration * CONTROL_HZ)
    )
    hold_step = 0
    while not grasp_lost and viewer.is_running() and (
        maximum_hold_steps is None or hold_step < maximum_hold_steps
    ):
        phi, optimization, diagnostics = optimized_equation_8_step(
            scene,
            equation_8,
            optimizer,
            phi,
            initial_position,
            desired_rotation,
            np.zeros(6),
            viewer,
            rate,
            trajectory_phase="final_hold",
            trajectory_time=(total_duration + hold_step / CONTROL_HZ),
            recorder=recorder,
            show_collision_spheres=show_collision_spheres,
            show_table_collision_spheres=show_table_collision_spheres,
            enable_redundancy_optimization=enable_redundancy_optimization,
        )
        loss_status = object_grasp_loss_status(
            diagnostics.grasp_pose_error,
        )
        if loss_status is not None:
            print_grasp_loss(loss_status)
            grasp_lost = True
            break
        if hold_step % int(CONTROL_HZ) == 0:
            print(
                f"  hold={hold_step / CONTROL_HZ:5.1f}s, "
                f"objective={optimization.value:.6g}, position error="
                f"{np.linalg.norm(diagnostics.pose_error[:3]):.5f} m, "
                f"grasp error="
                f"{np.linalg.norm(diagnostics.grasp_pose_error):.5f}, "
                f"alpha={diagnostics.null_space_scale:.3f}, "
                f"joint margin="
                f"{diagnostics.minimum_joint_limit_distance:.4f} rad, "
                f"null leakage="
                f"{diagnostics.scaled_null_space_leakage:.2e}/"
                f"{diagnostics.unscaled_null_space_leakage:.2e}, "
                f"command speed="
                f"{np.max(np.abs(diagnostics.commanded_joint_velocity)):.4f} rad/s, "
                f"inter-arm clearance="
                f"{optimizer.minimum_inter_arm_clearance(scene.data):.4f} m, "
                f"inter-arm cost="
                f"{optimizer.inter_arm_collision_cost(scene.data):.6f}, "
                f"table clearance="
                f"{optimizer.minimum_arm_table_clearance(scene.data):.4f} m, "
                f"table cost="
                f"{optimizer.arm_table_collision_cost(scene.data):.6f}, "
                f"self clearance="
                f"{optimizer.minimum_self_collision_clearance(scene.data):.4f} m, "
                f"self cost="
                f"{optimizer.self_collision_cost(scene.data):.6f}, "
                f"weighted collision cost="
                f"{optimizer.total_collision_cost(scene.data):.6f}"
            )
        hold_step += 1

    return {
        "initial_position": initial_position,
        "lifted_position": lifted_position,
        "final_position": initial_position.copy(),
        "initial_objective": initial_objective,
        "apex_objective": apex_objective,
        "final_objective": optimizer.value(scene.data),
        "grasp_lost": grasp_lost,
    }


def main(
    *,
    record_data=False,
    output_csv=None,
    collision_weight=COLLISION_WEIGHT,
    collision_safety_margin=COLLISION_SAFETY_MARGIN,
    collision_proximity_scale=COLLISION_PROXIMITY_SCALE,
    collision_sphere_model_path=COLLISION_SPHERE_MODEL_PATH,
    enable_collision_penalty=ENABLE_COLLISION_PENALTY,
    enable_table_collision_penalty=ENABLE_TABLE_COLLISION_PENALTY,
    table_collision_weight=TABLE_COLLISION_WEIGHT,
    table_collision_safety_margin=TABLE_COLLISION_SAFETY_MARGIN,
    table_collision_proximity_scale=TABLE_COLLISION_PROXIMITY_SCALE,
    table_collision_geom_name=TABLE_COLLISION_GEOM_NAME,
    enable_self_collision_penalty=ENABLE_SELF_COLLISION_PENALTY,
    self_collision_weight=SELF_COLLISION_WEIGHT,
    self_collision_safety_margin=SELF_COLLISION_SAFETY_MARGIN,
    self_collision_proximity_scale=SELF_COLLISION_PROXIMITY_SCALE,
    show_collision_spheres=False,
    show_table_collision_spheres=False,
    optimization_gain=OPTIMIZATION_GAIN,
    maximum_joint_speed=MAXIMUM_OPTIMIZATION_JOINT_SPEED,
    joint_limit_margin=JOINT_LIMIT_MARGIN,
    joint_limit_stop_distance=JOINT_LIMIT_STOP_DISTANCE,
    joint_limit_slow_distance=JOINT_LIMIT_SLOW_DISTANCE,
    objective=OBJECTIVE,
    characteristic_length=None,
    enable_redundancy_optimization=True,
    hold_duration=FINAL_HOLD_DURATION,
    top_view=False,
    front_view=False,
    video_output_dir=None,
    video_width=1280,
    video_height=720,
    video_fps=30,
    video_views=None,
    table_spawn_position=TABLE_SPAWN_POSITION,
):
    objective = ManipulabilityObjective(objective)
    if hold_duration is not None and hold_duration < 0.0:
        raise ValueError("hold_duration cannot be negative")
    optimization_mode = (
        objective.value if enable_redundancy_optimization else "baseline"
    )
    scene = DualFrankaMuJoCoScene(
        control_hz=CONTROL_HZ,
        left_arm_base_position=LEFT_ARM_SPAWN_POSITION,
        right_arm_base_position=RIGHT_ARM_SPAWN_POSITION,
        left_arm_base_euler_xyz_degrees=LEFT_ARM_SPAWN_EULER_XYZ_DEGREES,
        right_arm_base_euler_xyz_degrees=RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES,
        show_mocap_targets=SHOW_MOCAP_TARGETS,
        enable_bias_compensation=ENABLE_ARM_BIAS_COMPENSATION,
    )
    scene.set_table_reference_pose(
        np.asarray(table_spawn_position, dtype=float)
    )
    kinematics = CooperativeManipulationKinematics(
        scene.model,
        scene.left_arm_dofs,
        scene.right_arm_dofs,
    )
    # Resolve once from the initialized rigid grasp; keep it fixed for the run.
    (
        selected_characteristic_length,
        computed_characteristic_length,
        contact_midpoint,
        midpoint_reference_distance,
    ) = kinematics.resolve_characteristic_length(
        scene.data,
        characteristic_length,
    )
    print(
        "Computed grasp characteristic length: "
        f"{computed_characteristic_length:.9f} m"
    )
    print(f"Contact midpoint: {contact_midpoint} m")
    print(
        "Contact midpoint to object reference distance: "
        f"{midpoint_reference_distance:.9g} m"
    )
    print(
        "Characteristic length used: "
        f"{selected_characteristic_length:.9f} m "
        f"({'automatic' if characteristic_length is None else 'manual override'})"
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
        objective=objective,
        gain=optimization_gain,
        finite_difference_step=FINITE_DIFFERENCE_STEP,
        maximum_joint_speed=maximum_joint_speed,
        desired_wrench_direction=DESIRED_WRENCH_DIRECTION,
        characteristic_length=selected_characteristic_length,
        enable_collision_penalty=enable_collision_penalty,
        collision_weight=collision_weight,
        collision_safety_margin=collision_safety_margin,
        collision_proximity_scale=collision_proximity_scale,
        collision_version="version2",
        collision_sphere_model_path=collision_sphere_model_path,
        enable_table_collision_penalty=enable_table_collision_penalty,
        table_collision_weight=table_collision_weight,
        table_collision_safety_margin=table_collision_safety_margin,
        table_collision_proximity_scale=table_collision_proximity_scale,
        table_collision_geom_name=table_collision_geom_name,
        enable_self_collision_penalty=enable_self_collision_penalty,
        self_collision_weight=self_collision_weight,
        self_collision_safety_margin=self_collision_safety_margin,
        self_collision_proximity_scale=self_collision_proximity_scale,
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
    print(
        "Table collision spheres: "
        f"left={optimizer.left_table_radii.size}, "
        f"right={optimizer.right_table_radii.size}, "
        f"geom_id={optimizer.table_collision_geom_id}, "
        f"weight={optimizer.table_collision_weight:g}, "
        f"safety margin={optimizer.table_collision_safety_margin:.3f} m, "
        f"proximity scale={optimizer.table_collision_proximity_scale:.3f} m"
    )
    print(
        "Same-arm non-adjacent collision pairs: "
        f"left={optimizer.left_self_collision_pairs.shape[0]}, "
        f"right={optimizer.right_self_collision_pairs.shape[0]}, "
        f"weight={optimizer.self_collision_weight:g}, "
        f"safety margin={optimizer.self_collision_safety_margin:.3f} m, "
        f"proximity scale={optimizer.self_collision_proximity_scale:.3f} m"
    )
    if show_collision_spheres or show_table_collision_spheres:
        # Make visual meshes translucent so body-internal fitted spheres
        # remain visible in the overlay. This does not affect contacts.
        visual_geoms = scene.model.geom_group == 2
        scene.model.geom_rgba[visual_geoms, 3] = 0.25
    video_mode = video_output_dir is not None
    if video_mode and show_collision_spheres:
        raise ValueError(
            "collision-sphere overlays are unavailable with headless video"
        )
    if video_mode:
        viewer_context = scene.launch_video_recorder(
            video_output_dir,
            width=video_width,
            height=video_height,
            fps=video_fps,
            views=video_views,
        )
        rate_context = TqdmSimulationRate(
            f"Recording pick/place {optimization_mode}"
        )
    else:
        viewer_context = scene.launch_viewer()
        rate_context = nullcontext(
            RateLimiter(frequency=CONTROL_HZ, warn=False)
        )
    recorder = None
    if record_data or output_csv is not None:
        recorder = Equation8CSVRecorder(
            scene,
            kinematics,
            equation_8,
            optimizer,
            experiment_name=(
                "dual_franka_eq8_pick_place_"
                f"{optimization_mode}_fitted_spheres"
            ),
            output_path=output_csv,
            optimization_mode=optimization_mode,
        )
        print(f"Recording data to: {recorder.output_path}")

    try:
        with rate_context as rate, viewer_context as viewer:
            if not video_mode:
                scene.configure_viewer_camera(
                    viewer,
                    top_view=top_view,
                    front_view=front_view,
                )
            scene.settle(viewer, rate)
            scene.run_grasp_approach(viewer, rate)
            print("Closing both grippers...")
            scene.close_grippers(viewer, rate)
            equation_8.capture_grasp_reference(scene.data)
            result = run_optimized_lift(
                scene,
                kinematics,
                equation_8,
                optimizer,
                viewer,
                rate,
                recorder=recorder,
                show_collision_spheres=show_collision_spheres,
                show_table_collision_spheres=show_table_collision_spheres,
                hold_duration=hold_duration,
                enable_redundancy_optimization=(
                    enable_redundancy_optimization
                ),
            )
            finish_after_grasped_motion(
                scene,
                viewer,
                rate,
                grasp_lost=result["grasp_lost"],
            )
    finally:
        if recorder is not None:
            recorder.close()
            print(
                f"Saved {recorder.rows_written} rows to: "
                f"{recorder.output_path}"
            )
        if video_mode:
            print(f"Saved perspective and top-view videos to: {video_output_dir}")


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
        "--objective",
        choices=[objective.value for objective in ManipulabilityObjective],
        default=OBJECTIVE.value,
        help="null-space objective (default: %(default)s)",
    )
    parser.add_argument(
        "--characteristic-length",
        type=float,
        default=None,
        help=(
            "manual spatial characteristic length in metres; by default "
            "compute it once from the rigid object contact sites"
        ),
    )
    parser.add_argument(
        "--baseline",
        action="store_true",
        help="disable null-space optimization while tracking the same motion",
    )
    parser.add_argument(
        "--hold-duration",
        type=float,
        default=FINAL_HOLD_DURATION,
        help=(
            "placed-pose hold before disengagement in seconds "
            "(default: %(default)s)"
        ),
    )
    parser.add_argument(
        "--output-csv",
        help=(
            "Optional CSV path. Supplying it also enables recording; existing "
            "files receive a collision-safe numeric suffix."
        ),
    )
    position_group = parser.add_mutually_exclusive_group()
    position_group.add_argument(
        "--position",
        type=int,
        choices=range(1, len(TABLE_SPAWN_CASES) + 1),
        help="use predefined table position 1 through 6",
    )
    position_group.add_argument(
        "--table-spawn-position",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="custom table site_top_middle world position in metres",
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
        "--disable-table-collision-penalty",
        action="store_true",
        help="disable the arm-to-table soft collision term",
    )
    parser.add_argument(
        "--table-collision-weight",
        type=float,
        default=TABLE_COLLISION_WEIGHT,
        help="arm-table collision cost weight",
    )
    parser.add_argument(
        "--table-collision-safety-margin",
        type=float,
        default=TABLE_COLLISION_SAFETY_MARGIN,
        help="desired arm-sphere clearance above the table plane in metres",
    )
    parser.add_argument(
        "--table-collision-proximity-scale",
        type=float,
        default=TABLE_COLLISION_PROXIMITY_SCALE,
        help="arm-table softplus transition scale in metres",
    )
    parser.add_argument(
        "--table-collision-geom",
        default=TABLE_COLLISION_GEOM_NAME,
        help="optional box geom whose top face defines the table plane",
    )
    parser.add_argument(
        "--disable-self-collision-penalty",
        action="store_true",
        help="disable non-adjacent same-arm link collision avoidance",
    )
    parser.add_argument(
        "--self-collision-weight",
        type=float,
        default=SELF_COLLISION_WEIGHT,
        help="same-arm self-collision cost weight",
    )
    parser.add_argument(
        "--self-collision-safety-margin",
        type=float,
        default=SELF_COLLISION_SAFETY_MARGIN,
        help="desired non-adjacent same-arm clearance in metres",
    )
    parser.add_argument(
        "--self-collision-proximity-scale",
        type=float,
        default=SELF_COLLISION_PROXIMITY_SCALE,
        help="same-arm self-collision softplus transition scale in metres",
    )
    parser.add_argument(
        "--show-collision-spheres",
        action="store_true",
        help="draw fitted left/right spheres in the MuJoCo viewer",
    )
    parser.add_argument(
        "--show-table-collision-spheres",
        action="store_true",
        help="draw only the link1-link7 spheres used against the table",
    )
    add_camera_view_arguments(parser, scope="interactive run")
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


def cli():
    """Parse CLI arguments and run one optimized pick-and-place task."""
    arguments = parse_arguments()
    selected_table_position = (
        table_spawn_position_for_number(arguments.position)
        if arguments.position is not None
        else (
            arguments.table_spawn_position
            if arguments.table_spawn_position is not None
            else TABLE_SPAWN_POSITION
        )
    )
    main(
        record_data=arguments.record_data,
        output_csv=arguments.output_csv,
        collision_weight=arguments.collision_weight,
        collision_safety_margin=arguments.collision_safety_margin,
        collision_proximity_scale=arguments.collision_proximity_scale,
        collision_sphere_model_path=arguments.collision_sphere_model,
        enable_collision_penalty=not arguments.disable_collision_penalty,
        enable_table_collision_penalty=(
            not arguments.disable_table_collision_penalty
        ),
        table_collision_weight=arguments.table_collision_weight,
        table_collision_safety_margin=(
            arguments.table_collision_safety_margin
        ),
        table_collision_proximity_scale=(
            arguments.table_collision_proximity_scale
        ),
        table_collision_geom_name=arguments.table_collision_geom,
        enable_self_collision_penalty=(
            not arguments.disable_self_collision_penalty
        ),
        self_collision_weight=arguments.self_collision_weight,
        self_collision_safety_margin=(
            arguments.self_collision_safety_margin
        ),
        self_collision_proximity_scale=(
            arguments.self_collision_proximity_scale
        ),
        show_collision_spheres=arguments.show_collision_spheres,
        show_table_collision_spheres=(
            arguments.show_table_collision_spheres
        ),
        optimization_gain=arguments.optimization_gain,
        maximum_joint_speed=arguments.max_joint_speed,
        joint_limit_margin=arguments.joint_limit_margin,
        joint_limit_stop_distance=arguments.joint_limit_stop_distance,
        joint_limit_slow_distance=arguments.joint_limit_slow_distance,
        objective=arguments.objective,
        characteristic_length=arguments.characteristic_length,
        enable_redundancy_optimization=not arguments.baseline,
        hold_duration=arguments.hold_duration,
        top_view=arguments.top_view,
        front_view=arguments.front_view,
        table_spawn_position=selected_table_position,
    )


if __name__ == "__main__":
    run_cli(cli)
