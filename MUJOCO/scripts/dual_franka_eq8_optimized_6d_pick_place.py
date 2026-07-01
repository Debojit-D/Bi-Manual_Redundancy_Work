"""Optimized dual-Franka pick-and-place through three configurable SE(3) poses."""

import mujoco
import numpy as np
from loop_rate_limiters import RateLimiter
from scipy.interpolate import CubicSpline
from scipy.spatial.transform import Rotation

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
LEFT_ARM_SPAWN_POSITION = np.array([0.0, -0.2, 0.2])
RIGHT_ARM_SPAWN_POSITION = np.array([0.0, 0.2, 0.2])
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

# Null-space optimization remains active throughout both trajectory segments
# and continuously at the final goal pose.
OBJECTIVE = ManipulabilityObjective.FORCE
OPTIMIZATION_GAIN = 100.0
MAXIMUM_OPTIMIZATION_JOINT_SPEED = 10
FINITE_DIFFERENCE_STEP = 1e-4

ENABLE_COLLISION_PENALTY = True
COLLISION_WEIGHT = 1700.0
COLLISION_SAFETY_MARGIN = 0.08
COLLISION_SPHERE_RADIUS = 0.07

DESIRED_WRENCH_DIRECTION = np.array([0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
CHARACTERISTIC_LENGTH = 0.4


def rotation_matrix(euler_xyz):
    """Convert an editable XYZ Euler vector to a rotation matrix."""
    euler_xyz = np.asarray(euler_xyz, dtype=float)
    if euler_xyz.shape != (3,):
        raise ValueError("Euler orientation must have shape (3,)")
    return Rotation.from_euler("xyz", euler_xyz).as_matrix()


def set_table_reference_pose(scene, position, rotation):
    """Set the table free joint so ``site_top_middle`` has the requested pose."""
    position = np.asarray(position, dtype=float)
    rotation = np.asarray(rotation, dtype=float)
    if position.shape != (3,) or rotation.shape != (3, 3):
        raise ValueError("Table pose must contain a 3D position and 3x3 rotation")

    model = scene.model
    data = scene.data
    body_id = model.body("vention_table").id
    site_id = model.site("site_top_middle").id
    joint_id = model.joint("table_joint").id

    # Recover the fixed body-to-reference-site transform from the loaded model,
    # then invert it to obtain the free-body pose for the desired site pose.
    current_body_rotation = data.xmat[body_id].reshape(3, 3)
    current_site_rotation = data.site_xmat[site_id].reshape(3, 3)
    body_to_site_rotation = current_body_rotation.T @ current_site_rotation
    body_to_site_position = current_body_rotation.T @ (
        data.site_xpos[site_id] - data.xpos[body_id]
    )

    desired_body_rotation = rotation @ body_to_site_rotation.T
    desired_body_position = position - (
        desired_body_rotation @ body_to_site_position
    )

    qpos_address = model.jnt_qposadr[joint_id]
    dof_address = model.jnt_dofadr[joint_id]
    body_quaternion_xyzw = Rotation.from_matrix(
        desired_body_rotation
    ).as_quat()
    data.qpos[qpos_address : qpos_address + 3] = desired_body_position
    data.qpos[qpos_address + 3 : qpos_address + 7] = np.roll(
        body_quaternion_xyzw, 1
    )
    data.qvel[dof_address : dof_address + 6] = 0.0
    mujoco.mj_forward(model, data)

    scene.configuration.update(data.qpos)
    scene.posture_task.set_target_from_configuration(scene.configuration)


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
):
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


def run_trajectory(
    scene,
    equation_8,
    optimizer,
    phi,
    trajectory,
    viewer,
    rate,
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

    for step in range(number_of_steps):
        time = min(step / CONTROL_HZ, trajectory.total_duration)
        desired_position, desired_rotation, desired_twist = trajectory.sample(
            time
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
        )

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
                f"clearance="
                f"{optimizer.minimum_inter_arm_clearance(scene.data):.4f} m, "
                f"collision cost="
                f"{optimizer.inter_arm_collision_cost(scene.data):.6f}"
            )
    return phi


def hold_goal_pose(
    scene,
    equation_8,
    optimizer,
    phi,
    goal_pose,
    viewer,
    rate,
    duration=None,
):
    """Keep Equation (8) and optimization active at the final grasped pose."""
    goal_position, goal_rotation = goal_pose
    maximum_steps = None if duration is None else int(duration * CONTROL_HZ)
    step = 0
    print("Goal reached. Holding the grasp with optimization active...")
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
        )
        if step % int(CONTROL_HZ) == 0:
            print(
                f"  hold={step / CONTROL_HZ:5.1f}s, "
                f"objective={optimization.value:.6g}, position error="
                f"{np.linalg.norm(diagnostics.pose_error[:3]):.5f} m, "
                f"orientation error="
                f"{np.linalg.norm(diagnostics.pose_error[3:]):.5f} rad, "
                f"grasp error="
                f"{np.linalg.norm(diagnostics.grasp_pose_error):.5f}"
            )
        step += 1
    return phi


def run_pick_and_place(
    scene,
    kinematics,
    equation_8,
    optimizer,
    viewer,
    rate,
    hold_duration=None,
):
    """Execute start -> intermediate -> goal and continuously hold the grasp."""
    measured_start_pose = kinematics.object_pose(scene.data)
    intermediate_pose = (
        TABLE_INTERMEDIATE_POSITION.copy(),
        rotation_matrix(TABLE_INTERMEDIATE_EULER_XYZ),
    )
    goal_pose = (
        TABLE_GOAL_POSITION.copy(),
        rotation_matrix(TABLE_GOAL_EULER_XYZ),
    )
    phi = scene.arm_configuration()
    trajectory = ContinuousSE3WaypointTrajectory(
        (measured_start_pose, intermediate_pose, goal_pose),
        (START_TO_INTERMEDIATE_DURATION, INTERMEDIATE_TO_GOAL_DURATION),
    )
    phi = run_trajectory(
        scene,
        equation_8,
        optimizer,
        phi,
        trajectory,
        viewer,
        rate,
    )
    hold_goal_pose(
        scene,
        equation_8,
        optimizer,
        phi,
        goal_pose,
        viewer,
        rate,
        hold_duration,
    )

    held_position, held_rotation = kinematics.object_pose(scene.data)
    return {
        "goal_position": goal_pose[0],
        "goal_rotation": goal_pose[1],
        "held_position": held_position,
        "held_rotation": held_rotation,
        "final_objective": optimizer.value(scene.data),
        "minimum_clearance": optimizer.minimum_inter_arm_clearance(scene.data),
    }


def main():
    scene = DualFrankaMuJoCoScene(
        control_hz=CONTROL_HZ,
        left_arm_base_position=LEFT_ARM_SPAWN_POSITION,
        right_arm_base_position=RIGHT_ARM_SPAWN_POSITION,
        show_mocap_targets=SHOW_MOCAP_TARGETS,
        enable_bias_compensation=ENABLE_ARM_BIAS_COMPENSATION,
    )
    set_table_reference_pose(
        scene,
        TABLE_START_POSITION,
        rotation_matrix(TABLE_START_EULER_XYZ),
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
        run_pick_and_place(
            scene,
            kinematics,
            equation_8,
            optimizer,
            viewer,
            rate,
        )


if __name__ == "__main__":
    main()
