"""Dual-Franka pick-and-lift baseline using Equation (8) without optimization.

The approach and grasp sequence follows ``dual_franka_ikV6_mink_proposal.py``.
Once both grippers are closed, object motion is controlled with the primary term
of Equation (8) from the paper:

    phi[k+1] = phi[k] + A^dagger (q_dot_d + K_p e) dt

The null-space term is deliberately zero in this baseline.  Future comparisons
can add velocity, force, or directional-force objectives at the marked location
without changing the primary task controller or pick-and-place sequence.
"""

from pathlib import Path

import mink
import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation


MODEL_PATH = (
    Path(__file__).resolve().parent.parent
    / "robot_descriptions"
    / "franka_emika_panda"
    / "dual_panda_scene.xml"
)

SOLVER = "daqp"
CONTROL_HZ = 50.0
CONTROL_DT = 1.0 / CONTROL_HZ
APPROACH_TIMEOUT = 20.0
APPROACH_SEGMENT_DURATION = 3.0
MAX_APPROACH_JOINT_SPEED = 0.6
LIFT_HEIGHT = 0.26
LIFT_DURATION = 6.0
GRIPPER_OPEN = 255.0
GRIPPER_CLOSED = 0.0
ENABLE_ARM_BIAS_COMPENSATION = True
SHOW_MOCAP_TARGETS = False

# Closed-loop gains in object twist coordinates [linear; angular].  These were
# selected from full grasp-lift-hold simulations: stronger position correction
# removes lift lag, while a lower rotation gain avoids slowly unloading the
# frictional fingertip contacts.
K_P = np.diag([8.0, 8.0, 8.0, 2.0, 2.0, 2.0])

LEFT_JOINT_NAMES = tuple(f"joint{i}_l" for i in range(1, 8))
RIGHT_JOINT_NAMES = tuple(f"joint{i}_r" for i in range(1, 8))


def set_mocap_target_visibility(model, visible):
    """Show or hide the two mocap target-body geoms in the viewer."""
    alpha = 1.0 if visible else 0.0
    target_body_ids = {
        model.body("target_left").id,
        model.body("target_right").id,
    }
    for geom_id in range(model.ngeom):
        if model.geom_bodyid[geom_id] in target_body_ids:
            model.geom_rgba[geom_id, 3] = alpha


def initialize_model():
    model = mujoco.MjModel.from_xml_path(MODEL_PATH.as_posix())
    set_mocap_target_visibility(model, SHOW_MOCAP_TARGETS)
    data = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, data, model.key("home1").id)
    mujoco.mj_forward(model, data)
    return model, data, mink.Configuration(model)


def joint_indices(model, joint_names):
    """Return the scalar qpos and DoF indices for a sequence of hinge joints."""
    joint_ids = np.array([model.joint(name).id for name in joint_names], dtype=int)
    return model.jnt_qposadr[joint_ids], model.jnt_dofadr[joint_ids]


def arm_indices(model):
    left_qpos, left_dof = joint_indices(model, LEFT_JOINT_NAMES)
    right_qpos, right_dof = joint_indices(model, RIGHT_JOINT_NAMES)
    return (
        np.concatenate((left_qpos, right_qpos)),
        np.concatenate((left_dof, right_dof)),
    )


def arm_configuration(model, data):
    qpos_indices, _ = arm_indices(model)
    return data.qpos[qpos_indices].copy()


def clip_arm_configuration(model, phi):
    """Clip a 14-joint arm command to the MJCF position-servo ranges."""
    left_range = model.actuator_ctrlrange[0:7]
    right_range = model.actuator_ctrlrange[8:15]
    lower = np.concatenate((left_range[:, 0], right_range[:, 0]))
    upper = np.concatenate((left_range[:, 1], right_range[:, 1]))
    return np.clip(phi, lower, upper)


def command_arm_configuration(model, data, phi, gripper_command):
    """Send 14 arm position targets and two gripper commands."""
    phi = np.asarray(phi, dtype=float)
    if phi.shape != (14,):
        raise ValueError(f"Expected 14 arm positions, got {phi.shape}")

    # Respect the position-servo control ranges from the MJCF.
    phi = clip_arm_configuration(model, phi)
    data.ctrl[0:7] = phi[:7]
    data.ctrl[8:15] = phi[7:]
    data.ctrl[7] = gripper_command
    data.ctrl[15] = gripper_command


def step_control_period(model, data, viewer=None):
    """Advance one controller period with arm gravity/bias compensation.

    The arm actuators in the MJCF are position servos, not direct-torque
    motors.  Feedforward compensation is therefore applied as generalized
    force through ``qfrc_applied``.  MuJoCo's ``qfrc_bias`` contains gravity
    and velocity-dependent bias forces; updating it every physics substep
    prevents the position servos from having to generate those forces through
    steady-state position error.
    """
    substeps = max(1, round(CONTROL_DT / model.opt.timestep))
    _, controlled_dofs = arm_indices(model)
    for _ in range(substeps):
        if ENABLE_ARM_BIAS_COMPENSATION:
            data.qfrc_applied[controlled_dofs] = data.qfrc_bias[controlled_dofs]
        else:
            data.qfrc_applied[controlled_dofs] = 0.0
        mujoco.mj_step(model, data)
    if viewer is not None:
        viewer.sync()


def make_mink_tasks(model):
    left_task = mink.FrameTask(
        "attachment_site_left",
        "site",
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1.0,
    )
    right_task = mink.FrameTask(
        "attachment_site_right",
        "site",
        position_cost=1.0,
        orientation_cost=1.0,
        lm_damping=1.0,
    )
    posture_task = mink.PostureTask(model=model, cost=1e-2)
    return left_task, right_task, posture_task


def site_quaternion(data, site_id):
    quat = np.empty(4)
    mujoco.mju_mat2Quat(quat, data.site_xmat[site_id])
    return quat


def initialize_mocap_targets(model, data):
    left_object_site = model.site("site_left").id
    right_object_site = model.site("site_right").id
    left_mocap = model.body("target_left").mocapid
    right_mocap = model.body("target_right").mocapid

    data.mocap_pos[left_mocap] = data.site_xpos[left_object_site].copy()
    data.mocap_pos[right_mocap] = data.site_xpos[right_object_site].copy()
    data.mocap_quat[left_mocap] = site_quaternion(data, left_object_site)
    data.mocap_quat[right_mocap] = site_quaternion(data, right_object_site)
    return data.mocap_quat[left_mocap].copy(), data.mocap_quat[right_mocap].copy()


def define_approach_waypoints(data, model, left_quat, right_quat):
    """Use the same two-stage approach geometry as the original demonstration."""
    left_site = model.site("site_left").id
    right_site = model.site("site_right").id

    # The two object contact sites are separated along world y.  Approach each
    # side along its outward contact normal, not along x or z.
    left_1 = data.site_xpos[left_site].copy()
    left_1[1] -= 0.05
    right_1 = data.site_xpos[right_site].copy()
    right_1[1] += 0.05

    left_2 = left_1.copy()
    left_2[1] += 0.07
    right_2 = right_1.copy()
    right_2[1] -= 0.07

    return (
        [(left_1, left_quat.copy()), (left_2, left_quat.copy())],
        [(right_1, right_quat.copy()), (right_2, right_quat.copy())],
    )


def set_mocap_targets(model, data, left_target, right_target):
    left_pos, left_quat = left_target
    right_pos, right_quat = right_target
    data.mocap_pos[model.body("target_left").mocapid] = left_pos
    data.mocap_quat[model.body("target_left").mocapid] = left_quat
    data.mocap_pos[model.body("target_right").mocapid] = right_pos
    data.mocap_quat[model.body("target_right").mocapid] = right_quat


def quaternion_distance(q1, q2):
    return min(np.linalg.norm(q1 - q2), np.linalg.norm(q1 + q2))


def quintic_scale(ratio):
    ratio = np.clip(ratio, 0.0, 1.0)
    return 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5


def interpolate_quaternion_wxyz(start, target, scale):
    """Interpolate MuJoCo-order quaternions along the shortest rotation."""
    start_rotation = Rotation.from_quat(np.roll(start, -1))
    target_rotation = Rotation.from_quat(np.roll(target, -1))
    relative_rotvec = (target_rotation * start_rotation.inv()).as_rotvec()
    interpolated = Rotation.from_rotvec(scale * relative_rotvec) * start_rotation
    return np.roll(interpolated.as_quat(), 1)


def interpolate_pose(start, target, scale):
    start_position, start_quaternion = start
    target_position, target_quaternion = target
    position = start_position + scale * (target_position - start_position)
    quaternion = interpolate_quaternion_wxyz(
        start_quaternion, target_quaternion, scale
    )
    return position, quaternion


def approach_target_reached(model, data, left_target, right_target):
    left_site = model.site("attachment_site_left").id
    right_site = model.site("attachment_site_right").id
    left_pos, left_quat = left_target
    right_pos, right_quat = right_target
    return (
        np.linalg.norm(data.site_xpos[left_site] - left_pos) <= 0.008
        and quaternion_distance(site_quaternion(data, left_site), left_quat) <= 0.008
        and np.linalg.norm(data.site_xpos[right_site] - right_pos) <= 0.008
        and quaternion_distance(site_quaternion(data, right_site), right_quat)
        <= 0.008
    )


def mink_approach_control_step(
    model, data, configuration, tasks, viewer, rate
):
    """Take one measured-state IK step with a uniform arm-speed limit."""
    configuration.update(data.qpos)
    velocity = mink.solve_ik(
        configuration,
        tasks,
        CONTROL_DT,
        SOLVER,
        damping=5e-3,
    )

    # Only the 14 arm DoFs belong to this approach controller.  Uniform scaling
    # preserves the IK direction while preventing a dead-beat target jump.
    _, controlled_dofs = arm_indices(model)
    controlled_velocity = np.zeros_like(velocity)
    arm_velocity = velocity[controlled_dofs]
    peak_speed = np.max(np.abs(arm_velocity))
    if peak_speed > MAX_APPROACH_JOINT_SPEED:
        arm_velocity *= MAX_APPROACH_JOINT_SPEED / peak_speed
    controlled_velocity[controlled_dofs] = arm_velocity

    configuration.integrate_inplace(controlled_velocity, CONTROL_DT)
    phi_command = np.concatenate((configuration.q[0:7], configuration.q[9:16]))
    command_arm_configuration(model, data, phi_command, GRIPPER_OPEN)
    step_control_period(model, data, viewer)
    rate.sleep()


def run_mink_approach(
    model,
    data,
    configuration,
    tasks,
    left_waypoints,
    right_waypoints,
    viewer,
    rate,
):
    """Move both open grippers through smooth Cartesian grasp waypoints."""
    left_task, right_task, _ = tasks
    max_cycles = int(APPROACH_TIMEOUT * CONTROL_HZ)
    trajectory_steps = int(APPROACH_SEGMENT_DURATION * CONTROL_HZ)
    left_site = model.site("attachment_site_left").id
    right_site = model.site("attachment_site_right").id

    for waypoint_index, (left_target, right_target) in enumerate(
        zip(left_waypoints, right_waypoints), start=1
    ):
        print(f"Smoothly approaching grasp waypoint {waypoint_index}...")

        left_start = (
            data.site_xpos[left_site].copy(),
            site_quaternion(data, left_site),
        )
        right_start = (
            data.site_xpos[right_site].copy(),
            site_quaternion(data, right_site),
        )

        # Move the Cartesian targets with a zero-velocity quintic profile rather
        # than exposing the IK solver to the complete waypoint error at once.
        for step in range(1, trajectory_steps + 1):
            scale = quintic_scale(step / trajectory_steps)
            left_intermediate = interpolate_pose(left_start, left_target, scale)
            right_intermediate = interpolate_pose(right_start, right_target, scale)
            set_mocap_targets(
                model, data, left_intermediate, right_intermediate
            )
            left_task.set_target(
                mink.SE3.from_mocap_name(model, data, "target_left")
            )
            right_task.set_target(
                mink.SE3.from_mocap_name(model, data, "target_right")
            )
            mink_approach_control_step(
                model, data, configuration, tasks, viewer, rate
            )

        # Keep tracking the final target until the physical position servos have
        # settled inside the original pose tolerance.
        set_mocap_targets(model, data, left_target, right_target)
        left_task.set_target(mink.SE3.from_mocap_name(model, data, "target_left"))
        right_task.set_target(mink.SE3.from_mocap_name(model, data, "target_right"))

        for _ in range(max_cycles):
            if approach_target_reached(model, data, left_target, right_target):
                print(f"Grasp waypoint {waypoint_index} reached.")
                break
            mink_approach_control_step(
                model, data, configuration, tasks, viewer, rate
            )
        else:
            raise RuntimeError(
                f"Timed out reaching grasp waypoint {waypoint_index}."
            )


def skew(vector):
    x, y, z = vector
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def compute_grasp_matrix(model, data):
    """Return the 6x12 full-contact grasp matrix in the world frame."""
    object_reference, _ = object_pose(model, data)
    contact_site_ids = [model.site("site_left").id, model.site("site_right").id]
    blocks = []
    for site_id in contact_site_ids:
        r = data.site_xpos[site_id] - object_reference
        block = np.block(
            [[np.eye(3), np.zeros((3, 3))], [skew(r), np.eye(3)]]
        )
        blocks.append(block)
    return np.hstack(blocks)


def compute_hand_jacobian(model, data):
    """Return the 12x14 block hand Jacobian in world coordinates."""
    _, arm_dofs = arm_indices(model)
    left_dofs = arm_dofs[:7]
    right_dofs = arm_dofs[7:]

    def site_jacobian(site_name, dof_indices):
        jacobian = np.zeros((6, model.nv))
        site_id = model.site(site_name).id
        mujoco.mj_jacSite(
            model,
            data,
            jacobian[:3],
            jacobian[3:],
            site_id,
        )
        return jacobian[:, dof_indices]

    left = site_jacobian("attachment_site_left", left_dofs)
    right = site_jacobian("attachment_site_right", right_dofs)
    return np.block([[left, np.zeros((6, 7))], [np.zeros((6, 7)), right]])


def compute_object_velocity_map(model, data):
    """Construct A so its pseudoinverse preserves grasp compatibility.

    The paper starts from ``J_H phi_dot = G^T q_dot`` and uses ``A^dagger``
    in Equation (8).  We first solve that original compatibility equation for
    the minimum-norm joint motion,

        B = J_H^dagger G^T,  phi_dot = B q_dot,

    and then define ``A = B^dagger``.  Consequently, the ``A^dagger`` used by
    Equation (8) recovers B.  Forming A directly as ``(G^T)^dagger J_H`` and
    pseudoinverting the product does not generally recover B; in this
    frictional grasp it accumulated incompatible hand motion and eventually
    lost contact during long holds.
    """
    grasp_matrix = compute_grasp_matrix(model, data)
    hand_jacobian = compute_hand_jacobian(model, data)
    object_to_joint_map = (
        np.linalg.pinv(hand_jacobian, rcond=1e-6) @ grasp_matrix.T
    )
    return np.linalg.pinv(object_to_joint_map, rcond=1e-6)


def object_pose(model, data):
    reference_site = model.site("site_top_middle").id
    position = data.site_xpos[reference_site].copy()
    rotation = data.site_xmat[reference_site].reshape(3, 3).copy()
    return position, rotation


def object_pose_error(desired_position, desired_rotation, current_position, current_rotation):
    position_error = desired_position - current_position
    # R_des R_cur^T expresses the correction in the world frame, matching A.
    orientation_error = Rotation.from_matrix(
        desired_rotation @ current_rotation.T
    ).as_rotvec()
    return np.concatenate((position_error, orientation_error))


def equation_8_primary_update(
    model,
    data,
    desired_position,
    desired_rotation,
    desired_twist,
    phi_reference=None,
):
    """Apply Equation (8) with the null-space term explicitly disabled."""
    if phi_reference is None:
        phi_reference = arm_configuration(model, data)
    current_position, current_rotation = object_pose(model, data)
    error = object_pose_error(
        desired_position,
        desired_rotation,
        current_position,
        current_rotation,
    )
    object_velocity_map = compute_object_velocity_map(model, data)
    closed_loop_object_twist = desired_twist + K_P @ error

    # Equation (8), primary closed-loop trajectory tracking term.
    primary_delta = (
        np.linalg.pinv(object_velocity_map, rcond=1e-5)
        @ closed_loop_object_twist
        * CONTROL_DT
    )

    # Baseline for future objective comparisons.  Keep this exactly zero here.
    null_space_delta = np.zeros_like(primary_delta)
    # Integrate from the persistent commanded state, as Equation (8) specifies.
    # Restarting from lagging measured joints on every cycle accumulates
    # kinematic integration error and appears visually as wrist-frame drift.
    phi_next = clip_arm_configuration(
        model, phi_reference + primary_delta + null_space_delta
    )

    diagnostics = {
        "error": error,
        "rank_A": np.linalg.matrix_rank(object_velocity_map),
        "condition_A": np.linalg.cond(object_velocity_map),
    }
    return phi_next, diagnostics


def quintic_lift_reference(initial_position, final_position, rotation, time):
    """Return a smooth desired pose and twist for the lift phase."""
    ratio = np.clip(time / LIFT_DURATION, 0.0, 1.0)
    scale = 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5
    scale_rate = (
        30.0 * ratio**2 - 60.0 * ratio**3 + 30.0 * ratio**4
    ) / LIFT_DURATION
    displacement = final_position - initial_position
    position = initial_position + scale * displacement
    linear_velocity = scale_rate * displacement
    return position, rotation, np.concatenate((linear_velocity, np.zeros(3)))


def run_equation_8_lift(model, data, viewer, rate):
    """Lift the grasped object using only Equation (8)'s primary term."""
    initial_position, desired_rotation = object_pose(model, data)
    final_position = initial_position + np.array([0.0, 0.0, LIFT_HEIGHT])
    number_of_steps = int(LIFT_DURATION / CONTROL_DT) + 1
    phi_reference = arm_configuration(model, data)

    print("Starting Equation (8) lift with null-space optimization disabled...")
    for step in range(number_of_steps):
        time = min(step * CONTROL_DT, LIFT_DURATION)
        desired_position, rotation, desired_twist = quintic_lift_reference(
            initial_position,
            final_position,
            desired_rotation,
            time,
        )
        phi_next, diagnostics = equation_8_primary_update(
            model,
            data,
            desired_position,
            rotation,
            desired_twist,
            phi_reference,
        )
        phi_reference = phi_next
        command_arm_configuration(model, data, phi_next, GRIPPER_CLOSED)
        step_control_period(model, data, viewer)
        rate.sleep()

        if step % int(CONTROL_HZ) == 0:
            print(
                f"  t={time:4.1f}s, position error="
                f"{np.linalg.norm(diagnostics['error'][:3]):.4f} m, "
                f"rank(A)={diagnostics['rank_A']}"
            )

    return final_position, desired_rotation, phi_reference


def hold_object_pose(
    model,
    data,
    viewer,
    rate,
    position,
    rotation,
    phi_reference,
    duration=None,
):
    """Hold the final object pose with zero desired object velocity."""
    steps = None if duration is None else int(duration / CONTROL_DT)
    step = 0
    while viewer.is_running() and (steps is None or step < steps):
        phi_next, _ = equation_8_primary_update(
            model,
            data,
            position,
            rotation,
            np.zeros(6),
            phi_reference,
        )
        phi_reference = phi_next
        command_arm_configuration(model, data, phi_next, GRIPPER_CLOSED)
        step_control_period(model, data, viewer)
        rate.sleep()
        step += 1
    return phi_reference


def main():
    model, data, configuration = initialize_model()
    left_task, right_task, posture_task = make_mink_tasks(model)
    configuration.update(data.qpos)
    posture_task.set_target_from_configuration(configuration)
    tasks = [left_task, right_task, posture_task]

    left_quat, right_quat = initialize_mocap_targets(model, data)
    left_waypoints, right_waypoints = define_approach_waypoints(
        data, model, left_quat, right_quat
    )
    rate = RateLimiter(frequency=CONTROL_HZ, warn=False)

    with mujoco.viewer.launch_passive(
        model=model,
        data=data,
        show_left_ui=False,
        show_right_ui=False,
    ) as viewer:
        viewer.cam.lookat[:] = [0.1, 0.0, 0.1]
        viewer.cam.azimuth = 70
        viewer.cam.elevation = -20
        viewer.cam.distance = 2.5

        # Let the position servos settle at the home configuration.
        for _ in range(int(1.0 / CONTROL_DT)):
            command_arm_configuration(
                model, data, arm_configuration(model, data), GRIPPER_OPEN
            )
            step_control_period(model, data, viewer)
            rate.sleep()

        run_mink_approach(
            model,
            data,
            configuration,
            tasks,
            left_waypoints,
            right_waypoints,
            viewer,
            rate,
        )

        print("Closing both grippers...")
        for _ in range(int(1.0 / CONTROL_DT)):
            command_arm_configuration(
                model, data, arm_configuration(model, data), GRIPPER_CLOSED
            )
            step_control_period(model, data, viewer)
            rate.sleep()

        final_position, final_rotation, phi_reference = run_equation_8_lift(
            model, data, viewer, rate
        )
        print(
            "Lift complete. Equation (8) remains active at the final pose; "
            "close the viewer to exit."
        )
        hold_object_pose(
            model,
            data,
            viewer,
            rate,
            final_position,
            final_rotation,
            phi_reference,
        )


if __name__ == "__main__":
    main()
