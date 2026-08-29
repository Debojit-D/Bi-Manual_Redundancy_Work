"""Collision-aware Equation (8) through three configurable SE(3) poses.

The two Franka arms grasp the table and follow one continuous spatial path
from the measured pickup pose through an intermediate pose to a final pose.
Both translation and orientation vary along the path. Equation (8) preserves
the primary object motion while the paper objective acts through the
joint-limit-safe projected null-space term used by the static and simple
pick-and-place experiments. After the final hold, the grippers open, retreat
to table-relative post-grasp poses, and both arms return home.

Run from the repository root according to what you want to inspect::

    # Default spatial pick-and-place with fitted collision spheres.
    python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place

    # Run the complete predefined 6D path for pose 1.
    python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place --pose 1

    # Pose map (each entry includes position + XYZ Euler orientation):
    # all six predefined pose paths are complete.
    # Pickup positions [x, y, z] metres:
    # 1=(0.30, 0.18, 0.28), 2=(0.60, 0.18, 0.28)
    # 3=(0.30, 0.00, 0.28), 4=(0.60, 0.00, 0.28)
    # 5=(0.30,-0.18, 0.28), 6=(0.60,-0.18, 0.28)

    # Complete a blank pose from the CLI (orientations are radians).
    python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place \\
        --pose 2 \\
        --intermediate-position 0.40 0.00 0.52 \\
        --intermediate-euler-xyz 0 0 1.9708 \\
        --goal-position 0.20 0.45 0.269 \\
        --goal-euler-xyz 0 0 2.3708

    # A fully custom pickup position remains available.
    python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place \\
        --start-position 0.45 -0.10 0.28 \\
        --goal-position 0.25 0.35 0.30

    # Draw the fitted spheres during the complete SE(3) motion.
    python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place \\
        --show-table-collision-spheres

    # Select a front or overhead interactive camera.
    python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place --front-view
    python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place --top-view

    # Record every Equation (8) step, including the final hold.
    python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place \\
        --output-csv results/optimized_6d_pick_place.csv

    # Override the spatial waypoint and final pose from the CLI.
    python -m bimanual_redundancy.experiments.dual_franka_eq8_optimized_6d_pick_place \\
        --intermediate-position 0.40 0.00 0.52 \\
        --goal-position 0.20 0.45 0.269

Inter-arm and non-adjacent same-arm collision use fitted sphere-to-sphere
clearance. Table collision uses fitted spheres on arm links 1--7 against the
oriented tabletop plane; hands and fingers are excluded so grasp contact
remains possible. All soft costs act only through the Equation (8) null-space
objective and are not hard collision guarantees. Run with ``--help`` for all
options. Press Ctrl+C at any time to close active resources and stop cleanly.
"""

import argparse
from contextlib import nullcontext
from pathlib import Path

from bimanual_redundancy import paths

import numpy as np
from loop_rate_limiters import RateLimiter
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation

from bimanual_redundancy.core import (
    CooperativeManipulationKinematics,
)
from bimanual_redundancy.simulation.recording import Equation8CSVRecorder
from bimanual_redundancy.simulation.cli import add_camera_view_arguments, run_cli
from bimanual_redundancy.simulation.control_timing import timed_equation_8_update
from bimanual_redundancy.simulation.grasp_safety import (
    finish_after_grasped_motion,
    object_grasp_loss_status,
    print_grasp_loss,
)
from bimanual_redundancy.core import (
    Equation8Controller,
    ManipulabilityObjective,
    ManipulabilityOptimizer,
)
from bimanual_redundancy.simulation import (
    DualFrankaMuJoCoScene,
    draw_detailed_collision_spheres,
    draw_table_collision_spheres,
)
from bimanual_redundancy.simulation.recording import TqdmSimulationRate
from bimanual_redundancy.experiments.table_spawn_comparison_positions import (
    SIX_D_TRAJECTORY_CASES,
    six_d_trajectory_case_for_number,
)


CONTROL_HZ = 50.0
SHOW_MOCAP_TARGETS = False
ENABLE_ARM_BIAS_COMPENSATION = True
# Robot base poses: world xyz [m] and extrinsic XYZ Euler angles [degrees].
LEFT_ARM_SPAWN_POSITION = np.array([0.0, -0.25, 0.0])
RIGHT_ARM_SPAWN_POSITION = np.array([0.0, 0.25, 0.0])
LEFT_ARM_SPAWN_EULER_XYZ_DEGREES = np.array([0.0, 0.0, 0.0])
RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES = np.array([0.0, 0.0, 0.0])
K_P = np.diag([8.0, 8.0, 8.0, 4.0, 4.0, 4.0])
GRASP_K_P = np.diag([8.0, 8.0, 8.0, 6.0, 6.0, 6.0])

# Editable table reference poses. Positions are world-frame metres and
# orientations are extrinsic XYZ Euler angles in radians. The controlled frame
# is the table's ``site_top_middle`` site, not its hidden free-joint body frame.
TABLE_START_POSITION = np.array([0.40, -0.15, 0.269])
TABLE_START_EULER_XYZ = np.array([0.0, 0.0, np.pi / 2.0])

TABLE_INTERMEDIATE_POSITION = np.array([0.40, 0.00, 0.52])
TABLE_INTERMEDIATE_EULER_XYZ = np.array([0.0, 0.0, np.pi / 2.0 + 0.40])

TABLE_GOAL_POSITION = np.array([0.20, 0.45, 0.269])
TABLE_GOAL_EULER_XYZ = np.array([0.0, 0.0, np.pi / 2.0 + 0.80])

START_TO_INTERMEDIATE_DURATION = 15.0
INTERMEDIATE_TO_GOAL_DURATION = 15.0
FINAL_HOLD_DURATION = 2.0

# Null-space optimization remains active throughout both trajectory segments
# and continuously at the final goal pose.
OBJECTIVE = ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT
OPTIMIZATION_GAIN = 5000.0
MAXIMUM_OPTIMIZATION_JOINT_SPEED = 5
FINITE_DIFFERENCE_STEP = 1e-4
JOINT_LIMIT_MARGIN = 0.05
JOINT_LIMIT_STOP_DISTANCE = 0.07
JOINT_LIMIT_SLOW_DISTANCE = 0.25

ENABLE_COLLISION_PENALTY = True
ENABLE_TABLE_COLLISION_PENALTY = True
ENABLE_SELF_COLLISION_PENALTY = True
COLLISION_WEIGHT = 5000.0
COLLISION_SAFETY_MARGIN = 0.05
COLLISION_PROXIMITY_SCALE = 0.01
TABLE_COLLISION_WEIGHT = 20000.0
# Keep table avoidance local: maintain 1.5 cm clearance and let its smooth
# influence decay over 3 mm so it does not dominate the paper objective.
TABLE_COLLISION_SAFETY_MARGIN = 0.05
TABLE_COLLISION_PROXIMITY_SCALE = 0.003
TABLE_COLLISION_GEOM_NAME = None
# Same-body, directly adjacent, and internal gripper/link7 pairs are excluded
# from this full-model local cost.
SELF_COLLISION_WEIGHT = 20000.0
SELF_COLLISION_SAFETY_MARGIN = 0.01
SELF_COLLISION_PROXIMITY_SCALE = 0.003
COLLISION_SPHERE_MODEL_PATH = (
    paths.ROBOT_MODELS_DIR / "franka_emika_panda" / "dual_panda_robots_only_spherefit.xml"
)

DESIRED_WRENCH_DIRECTION = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])


def rotation_matrix(euler_xyz):
    """Convert an editable XYZ Euler vector to a rotation matrix."""
    euler_xyz = np.asarray(euler_xyz, dtype=float)
    if euler_xyz.shape != (3,):
        raise ValueError("Euler orientation must have shape (3,)")
    return Rotation.from_euler("xyz", euler_xyz).as_matrix()


def set_table_reference_pose(scene, position, rotation):
    """Set the table free joint so ``site_top_middle`` has the requested pose."""
    scene.set_table_reference_pose(position, rotation)


class ContinuousSE3WaypointTrajectory:
    """C2-continuous trajectory that passes through an intermediate pose."""

    def __init__(self, poses, segment_durations):
        if len(poses) != 3 or len(segment_durations) != 2:
            raise ValueError("Expected start, intermediate, and goal poses")
        durations = np.asarray(segment_durations, dtype=float)
        if np.any(durations <= 0.0):
            raise ValueError("Trajectory durations must be positive")

        self.times = np.array([0.0, durations[0], np.sum(durations)])
        self.total_duration = float(self.times[-1])
        positions = np.stack([pose[0] for pose in poses])
        rotations = np.stack([pose[1] for pose in poses])
        zero_endpoint_velocity = (
            (1, np.zeros(3)),
            (1, np.zeros(3)),
        )
        self.position_spline = CubicSpline(
            self.times,
            positions,
            axis=0,
            bc_type=zero_endpoint_velocity,
        )

        # Use one world-frame exponential-coordinate chart so the orientation
        # and angular velocity pass continuously through the waypoint. The
        # configured rotations are all well inside the pi chart boundary.
        self.start_rotation = rotations[0]
        rotation_vectors = np.stack(
            [
                Rotation.from_matrix(
                    rotation @ self.start_rotation.T
                ).as_rotvec()
                for rotation in rotations
            ]
        )
        self.rotation_vector_spline = CubicSpline(
            self.times,
            rotation_vectors,
            axis=0,
            bc_type=zero_endpoint_velocity,
        )

    @staticmethod
    def _so3_left_jacobian(rotation_vector):
        """Map exponential-coordinate rate to world angular velocity."""
        rotation_vector = np.asarray(rotation_vector, dtype=float)
        theta = np.linalg.norm(rotation_vector)
        skew = np.array(
            [
                [0.0, -rotation_vector[2], rotation_vector[1]],
                [rotation_vector[2], 0.0, -rotation_vector[0]],
                [-rotation_vector[1], rotation_vector[0], 0.0],
            ]
        )
        if theta < 1e-7:
            return np.eye(3) + 0.5 * skew + (skew @ skew) / 6.0
        return (
            np.eye(3)
            + ((1.0 - np.cos(theta)) / theta**2) * skew
            + ((theta - np.sin(theta)) / theta**3) * (skew @ skew)
        )

    def sample(self, time):
        """Return position, rotation, and world-frame twist at ``time``."""
        time = float(np.clip(time, self.times[0], self.times[-1]))
        desired_position = self.position_spline(time)
        linear_velocity = self.position_spline(time, 1)
        rotation_vector = self.rotation_vector_spline(time)
        rotation_vector_rate = self.rotation_vector_spline(time, 1)
        desired_rotation = (
            Rotation.from_rotvec(rotation_vector).as_matrix()
            @ self.start_rotation
        )
        angular_velocity = (
            self._so3_left_jacobian(rotation_vector)
            @ rotation_vector_rate
        )
        desired_twist = np.concatenate((linear_velocity, angular_velocity))
        return desired_position, desired_rotation, desired_twist


def optimized_control_step(
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
    """Apply one joint-limit-safe primary-plus-null-space control step."""
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


def run_trajectory(
    scene,
    equation_8,
    optimizer,
    phi,
    trajectory,
    viewer,
    rate,
    recorder=None,
    show_collision_spheres=False,
    show_table_collision_spheres=False,
    enable_redundancy_optimization=True,
):
    """Execute one continuous start-through-waypoint-to-goal trajectory."""
    number_of_steps = int(trajectory.total_duration * CONTROL_HZ) + 1
    waypoint_time = trajectory.times[1]
    _, _, waypoint_twist = trajectory.sample(waypoint_time)
    print(
        "Starting continuous 6D trajectory: "
        f"waypoint linear speed={np.linalg.norm(waypoint_twist[:3]):.4f} m/s, "
        f"angular speed={np.linalg.norm(waypoint_twist[3:]):.4f} rad/s"
    )

    grasp_lost = False
    for step in range(number_of_steps):
        if not viewer.is_running():
            print("Viewer closed before the spatial trajectory completed.")
            break
        time = min(step / CONTROL_HZ, trajectory.total_duration)
        desired_position, desired_rotation, desired_twist = trajectory.sample(
            time
        )
        trajectory_phase = (
            "start_to_intermediate"
            if time < waypoint_time
            else "intermediate_to_goal"
        )
        phi, optimization, diagnostics = optimized_control_step(
            scene,
            equation_8,
            optimizer,
            phi,
            desired_position,
            desired_rotation,
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
            break

        if step % int(CONTROL_HZ) == 0:
            phase = (
                "before-waypoint" if time < waypoint_time else "after-waypoint"
            )
            print(
                f"  {phase} t={time:4.1f}s, "
                f"speed={np.linalg.norm(desired_twist[:3]):.4f} m/s, "
                f"objective={optimization.value:.6g}, "
                f"position error="
                f"{np.linalg.norm(diagnostics.pose_error[:3]):.5f} m, "
                f"orientation error="
                f"{np.linalg.norm(diagnostics.pose_error[3:]):.5f} rad, "
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
    return phi, grasp_lost


def hold_goal_pose(
    scene,
    equation_8,
    optimizer,
    phi,
    goal_pose,
    viewer,
    rate,
    duration=FINAL_HOLD_DURATION,
    trajectory_time_offset=0.0,
    recorder=None,
    show_collision_spheres=False,
    show_table_collision_spheres=False,
    enable_redundancy_optimization=True,
):
    """Keep Equation (8) and optimization active at the final grasped pose."""
    goal_position, goal_rotation = goal_pose
    maximum_steps = None if duration is None else int(duration * CONTROL_HZ)
    step = 0
    hold_activity = (
        "optimization active"
        if enable_redundancy_optimization
        else "null-space optimization disabled"
    )
    print(f"Goal reached. Holding the grasp with {hold_activity}...")
    grasp_lost = False
    while viewer.is_running() and (
        maximum_steps is None or step < maximum_steps
    ):
        phi, optimization, diagnostics = optimized_control_step(
            scene,
            equation_8,
            optimizer,
            phi,
            goal_position,
            goal_rotation,
            np.zeros(6),
            viewer,
            rate,
            trajectory_phase="final_hold",
            trajectory_time=(trajectory_time_offset + step / CONTROL_HZ),
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
        if step % int(CONTROL_HZ) == 0:
            print(
                f"  hold={step / CONTROL_HZ:5.1f}s, "
                f"objective={optimization.value:.6g}, position error="
                f"{np.linalg.norm(diagnostics.pose_error[:3]):.5f} m, "
                f"orientation error="
                f"{np.linalg.norm(diagnostics.pose_error[3:]):.5f} rad, "
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
        step += 1
    return phi, grasp_lost


def run_pick_and_place(
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
    intermediate_position=TABLE_INTERMEDIATE_POSITION,
    intermediate_euler_xyz=TABLE_INTERMEDIATE_EULER_XYZ,
    goal_position=TABLE_GOAL_POSITION,
    goal_euler_xyz=TABLE_GOAL_EULER_XYZ,
    start_to_intermediate_duration=START_TO_INTERMEDIATE_DURATION,
    intermediate_to_goal_duration=INTERMEDIATE_TO_GOAL_DURATION,
    enable_redundancy_optimization=True,
):
    """Execute start -> intermediate -> goal and continuously hold the grasp."""
    measured_start_pose = kinematics.object_pose(scene.data)
    intermediate_pose = (
        np.asarray(intermediate_position, dtype=float).copy(),
        rotation_matrix(intermediate_euler_xyz),
    )
    goal_pose = (
        np.asarray(goal_position, dtype=float).copy(),
        rotation_matrix(goal_euler_xyz),
    )
    phi = scene.arm_configuration()
    trajectory = ContinuousSE3WaypointTrajectory(
        (measured_start_pose, intermediate_pose, goal_pose),
        (start_to_intermediate_duration, intermediate_to_goal_duration),
    )
    phi, grasp_lost = run_trajectory(
        scene,
        equation_8,
        optimizer,
        phi,
        trajectory,
        viewer,
        rate,
        recorder=recorder,
        show_collision_spheres=show_collision_spheres,
        show_table_collision_spheres=show_table_collision_spheres,
        enable_redundancy_optimization=enable_redundancy_optimization,
    )
    if not grasp_lost:
        phi, grasp_lost = hold_goal_pose(
            scene,
            equation_8,
            optimizer,
            phi,
            goal_pose,
            viewer,
            rate,
            duration=hold_duration,
            trajectory_time_offset=trajectory.total_duration,
            recorder=recorder,
            show_collision_spheres=show_collision_spheres,
            show_table_collision_spheres=show_table_collision_spheres,
            enable_redundancy_optimization=enable_redundancy_optimization,
        )

    held_position, held_rotation = kinematics.object_pose(scene.data)
    return {
        "goal_position": goal_pose[0],
        "goal_rotation": goal_pose[1],
        "held_position": held_position,
        "held_rotation": held_rotation,
        "final_objective": optimizer.value(scene.data),
        "minimum_clearance": optimizer.minimum_inter_arm_clearance(scene.data),
        "minimum_table_clearance": (
            optimizer.minimum_arm_table_clearance(scene.data)
        ),
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
    start_position=TABLE_START_POSITION,
    start_euler_xyz=TABLE_START_EULER_XYZ,
    intermediate_position=TABLE_INTERMEDIATE_POSITION,
    intermediate_euler_xyz=TABLE_INTERMEDIATE_EULER_XYZ,
    goal_position=TABLE_GOAL_POSITION,
    goal_euler_xyz=TABLE_GOAL_EULER_XYZ,
    start_to_intermediate_duration=START_TO_INTERMEDIATE_DURATION,
    intermediate_to_goal_duration=INTERMEDIATE_TO_GOAL_DURATION,
    hold_duration=FINAL_HOLD_DURATION,
    objective=OBJECTIVE,
    characteristic_length=None,
    enable_redundancy_optimization=True,
    top_view=False,
    front_view=False,
    video_output_dir=None,
    video_width=1280,
    video_height=720,
    video_fps=30,
    video_views=None,
    video_encoder="x264",
    video_nvenc_view_limit=None,
    keep_viewer_open=False,
    use_alternate_grasp_orientation=False,
    headless=False,
):
    objective = ManipulabilityObjective(objective)
    if hold_duration is not None and hold_duration < 0.0:
        raise ValueError("hold_duration cannot be negative")
    optimization_mode = (
        objective.value if enable_redundancy_optimization else "baseline"
    )
    if use_alternate_grasp_orientation:
        print(
            "Using the alternate gripper orientation for the pregrasp "
            "IK branch."
        )
    scene = DualFrankaMuJoCoScene(
        control_hz=CONTROL_HZ,
        left_arm_base_position=LEFT_ARM_SPAWN_POSITION,
        right_arm_base_position=RIGHT_ARM_SPAWN_POSITION,
        left_arm_base_euler_xyz_degrees=LEFT_ARM_SPAWN_EULER_XYZ_DEGREES,
        right_arm_base_euler_xyz_degrees=RIGHT_ARM_SPAWN_EULER_XYZ_DEGREES,
        show_mocap_targets=SHOW_MOCAP_TARGETS,
        enable_bias_compensation=ENABLE_ARM_BIAS_COMPENSATION,
        use_alternate_grasp_orientation=use_alternate_grasp_orientation,
    )
    set_table_reference_pose(
        scene,
        np.asarray(start_position, dtype=float),
        rotation_matrix(start_euler_xyz),
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
        visual_geoms = scene.model.geom_group == 2
        scene.model.geom_rgba[visual_geoms, 3] = 0.25
    video_mode = video_output_dir is not None
    if headless and video_mode:
        raise ValueError("headless no-render mode cannot record video")
    if video_mode and show_collision_spheres:
        raise ValueError(
            "collision-sphere overlays are unavailable with headless video"
        )
    if headless:
        viewer_context = scene.launch_headless()
        rate_context = TqdmSimulationRate(
            f"Smoke 6D pick/place {optimization_mode}"
        )
    elif video_mode:
        viewer_context = scene.launch_video_recorder(
            video_output_dir,
            width=video_width,
            height=video_height,
            fps=video_fps,
            views=video_views,
            encoder=video_encoder,
            nvenc_view_limit=video_nvenc_view_limit,
        )
        rate_context = TqdmSimulationRate(
            f"Recording 6D pick/place {optimization_mode}"
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
                "dual_franka_eq8_6d_pick_place_"
                f"{optimization_mode}_fitted_spheres"
            ),
            output_path=output_csv,
            optimization_mode=optimization_mode,
        )
        print(f"Recording data to: {recorder.output_path}")

    try:
        with rate_context as rate, viewer_context as viewer:
            if not video_mode and not headless:
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
            result = run_pick_and_place(
                scene,
                kinematics,
                equation_8,
                optimizer,
                viewer,
                rate,
                hold_duration=hold_duration,
                recorder=recorder,
                show_collision_spheres=show_collision_spheres,
                show_table_collision_spheres=show_table_collision_spheres,
                intermediate_position=intermediate_position,
                intermediate_euler_xyz=intermediate_euler_xyz,
                goal_position=goal_position,
                goal_euler_xyz=goal_euler_xyz,
                start_to_intermediate_duration=(
                    start_to_intermediate_duration
                ),
                intermediate_to_goal_duration=intermediate_to_goal_duration,
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
            if (
                keep_viewer_open
                and enable_redundancy_optimization
                and not video_mode
                and viewer.is_running()
            ):
                print(
                    "Run complete. Keeping the viewer open; close it or "
                    "press Ctrl+C to exit."
                )
                while viewer.is_running():
                    viewer.sync()
                    rate.sleep()
            return result
    finally:
        if recorder is not None:
            recorder.close()
            print(
                f"Saved {recorder.rows_written} rows to: "
                f"{recorder.output_path}"
            )
        if video_mode:
            print(f"Saved requested video view(s) to: {video_output_dir}")


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--record-data",
        action="store_true",
        help="record every Equation (8) control step to CSV",
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
        help="disable null-space optimization while tracking the same path",
    )
    parser.add_argument(
        "--output-csv",
        help="optional CSV path; supplying it also enables recording",
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
        help="desired extra sphere-surface clearance in metres",
    )
    parser.add_argument(
        "--collision-proximity-scale",
        type=float,
        default=COLLISION_PROXIMITY_SCALE,
        help="smooth proximity transition length in metres",
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
        help="disable the soft inter-arm collision objective",
    )
    parser.add_argument(
        "--disable-table-collision-penalty",
        action="store_true",
        help="disable the arm-to-table soft collision objective",
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
        help="draw fitted spheres throughout the spatial motion",
    )
    parser.add_argument(
        "--show-table-collision-spheres",
        action="store_true",
        help="draw only link1-link7 spheres used against the table",
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
        help="distance in radians at which null-space motion stops",
    )
    parser.add_argument(
        "--joint-limit-slow-distance",
        type=float,
        default=JOINT_LIMIT_SLOW_DISTANCE,
        help="distance in radians at which null-space slowing begins",
    )
    start_position_group = parser.add_mutually_exclusive_group()
    start_position_group.add_argument(
        "--pose",
        "--position",
        dest="pose",
        type=int,
        choices=range(1, len(SIX_D_TRAJECTORY_CASES) + 1),
        help=(
            "use predefined complete 6D pose path 1 through 6; --position "
            "is retained as a compatibility alias"
        ),
    )
    start_position_group.add_argument(
        "--start-position",
        type=float,
        nargs=3,
        metavar=("X", "Y", "Z"),
        help="custom initial table reference position in world metres",
    )
    parser.add_argument(
        "--start-euler-xyz",
        type=float,
        nargs=3,
        default=None,
        metavar=("RX", "RY", "RZ"),
        help="initial extrinsic XYZ orientation in radians",
    )
    parser.add_argument(
        "--intermediate-position",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="intermediate table reference position in world metres",
    )
    parser.add_argument(
        "--intermediate-euler-xyz",
        type=float,
        nargs=3,
        default=None,
        metavar=("RX", "RY", "RZ"),
        help="intermediate extrinsic XYZ orientation in radians",
    )
    parser.add_argument(
        "--goal-position",
        type=float,
        nargs=3,
        default=None,
        metavar=("X", "Y", "Z"),
        help="final table reference position in world metres",
    )
    parser.add_argument(
        "--goal-euler-xyz",
        type=float,
        nargs=3,
        default=None,
        metavar=("RX", "RY", "RZ"),
        help="final extrinsic XYZ orientation in radians",
    )
    parser.add_argument(
        "--start-to-intermediate-duration",
        type=float,
        default=START_TO_INTERMEDIATE_DURATION,
        help="first trajectory-segment duration in seconds",
    )
    parser.add_argument(
        "--intermediate-to-goal-duration",
        type=float,
        default=INTERMEDIATE_TO_GOAL_DURATION,
        help="second trajectory-segment duration in seconds",
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
        "--close-on-completion",
        action="store_true",
        help=(
            "close the standalone viewer when the run ends instead of "
            "keeping the default optimized result visible"
        ),
    )
    return parser.parse_args()


def resolve_cli_trajectory(arguments):
    """Resolve one complete 6D path from a numbered pose or CLI values."""
    if arguments.pose is not None:
        configured = six_d_trajectory_case_for_number(arguments.pose)
        pose_name = configured[0]
        configured_values = configured[1:]
    else:
        pose_name = "custom_pose"
        configured_values = (
            TABLE_START_POSITION,
            TABLE_START_EULER_XYZ,
            TABLE_INTERMEDIATE_POSITION,
            TABLE_INTERMEDIATE_EULER_XYZ,
            TABLE_GOAL_POSITION,
            TABLE_GOAL_EULER_XYZ,
        )

    overrides = (
        arguments.start_position,
        arguments.start_euler_xyz,
        arguments.intermediate_position,
        arguments.intermediate_euler_xyz,
        arguments.goal_position,
        arguments.goal_euler_xyz,
    )
    values = tuple(
        override if override is not None else configured_value
        for override, configured_value in zip(overrides, configured_values)
    )
    component_names = (
        "start position",
        "start orientation",
        "intermediate position",
        "intermediate orientation",
        "goal position",
        "goal orientation",
    )
    missing = [
        name for name, value in zip(component_names, values) if value is None
    ]
    if missing:
        raise ValueError(
            f"{pose_name} is incomplete; provide " + ", ".join(missing)
        )
    return values


def cli():
    """Parse CLI arguments and run one spatial pick-and-place task."""
    arguments = parse_arguments()
    (
        start_position,
        start_euler_xyz,
        intermediate_position,
        intermediate_euler_xyz,
        goal_position,
        goal_euler_xyz,
    ) = resolve_cli_trajectory(arguments)
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
        start_position=start_position,
        start_euler_xyz=start_euler_xyz,
        intermediate_position=intermediate_position,
        intermediate_euler_xyz=intermediate_euler_xyz,
        goal_position=goal_position,
        goal_euler_xyz=goal_euler_xyz,
        start_to_intermediate_duration=(
            arguments.start_to_intermediate_duration
        ),
        intermediate_to_goal_duration=(
            arguments.intermediate_to_goal_duration
        ),
        hold_duration=arguments.hold_duration,
        objective=arguments.objective,
        characteristic_length=arguments.characteristic_length,
        enable_redundancy_optimization=not arguments.baseline,
        top_view=arguments.top_view,
        front_view=arguments.front_view,
        keep_viewer_open=not arguments.close_on_completion,
        # Pose 4 now starts at -pi/2 roll. Its nominal contact-site frame is
        # the reachable pregrasp branch; the old alternate branch was only
        # needed for the former +pi/2 orientation.
        use_alternate_grasp_orientation=False,
    )


if __name__ == "__main__":
    run_cli(cli)
