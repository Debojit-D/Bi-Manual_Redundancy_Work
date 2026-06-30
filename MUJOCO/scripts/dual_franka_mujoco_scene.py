"""MuJoCo scene and robot-control utilities for the dual-Franka demo."""

from pathlib import Path

import mink
import mujoco
import mujoco.viewer
import numpy as np
from scipy.spatial.transform import Rotation


class DualFrankaMuJoCoScene:
    """Own the MuJoCo model, actuators, viewer targets, and grasp approach."""

    DEFAULT_MODEL_PATH = (
        Path(__file__).resolve().parent.parent
        / "robot_descriptions"
        / "franka_emika_panda"
        / "dual_panda_scene.xml"
    )
    LEFT_JOINT_NAMES = tuple(f"joint{i}_l" for i in range(1, 8))
    RIGHT_JOINT_NAMES = tuple(f"joint{i}_r" for i in range(1, 8))

    def __init__(
        self,
        *,
        model_path=None,
        control_hz=50.0,
        solver="daqp",
        enable_bias_compensation=True,
        show_mocap_targets=False,
        gripper_open=255.0,
        gripper_closed=0.0,
        approach_timeout=20.0,
        approach_segment_duration=3.0,
        maximum_approach_joint_speed=0.6,
    ):
        self.model_path = Path(model_path or self.DEFAULT_MODEL_PATH)
        self.control_hz = float(control_hz)
        self.control_dt = 1.0 / self.control_hz
        self.solver = solver
        self.enable_bias_compensation = bool(enable_bias_compensation)
        self.gripper_open = float(gripper_open)
        self.gripper_closed = float(gripper_closed)
        self.approach_timeout = float(approach_timeout)
        self.approach_segment_duration = float(approach_segment_duration)
        self.maximum_approach_joint_speed = float(
            maximum_approach_joint_speed
        )

        self.model = mujoco.MjModel.from_xml_path(self.model_path.as_posix())
        self._set_mocap_target_visibility(show_mocap_targets)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(
            self.model, self.data, self.model.key("home1").id
        )
        mujoco.mj_forward(self.model, self.data)

        self.configuration = mink.Configuration(self.model)
        self.configuration.update(self.data.qpos)
        self.left_arm_qpos, self.left_arm_dofs = self._joint_indices(
            self.LEFT_JOINT_NAMES
        )
        self.right_arm_qpos, self.right_arm_dofs = self._joint_indices(
            self.RIGHT_JOINT_NAMES
        )
        self.arm_qpos = np.concatenate(
            (self.left_arm_qpos, self.right_arm_qpos)
        )
        self.arm_dofs = np.concatenate(
            (self.left_arm_dofs, self.right_arm_dofs)
        )
        self.left_task, self.right_task, self.posture_task = self._make_tasks()
        self.tasks = [self.left_task, self.right_task, self.posture_task]

    def _joint_indices(self, joint_names):
        joint_ids = np.array(
            [self.model.joint(name).id for name in joint_names], dtype=int
        )
        return (
            self.model.jnt_qposadr[joint_ids],
            self.model.jnt_dofadr[joint_ids],
        )

    def _set_mocap_target_visibility(self, visible):
        alpha = 1.0 if visible else 0.0
        target_body_ids = {
            self.model.body("target_left").id,
            self.model.body("target_right").id,
        }
        for geom_id in range(self.model.ngeom):
            if self.model.geom_bodyid[geom_id] in target_body_ids:
                self.model.geom_rgba[geom_id, 3] = alpha

    def _make_tasks(self):
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
        posture_task = mink.PostureTask(model=self.model, cost=1e-2)
        posture_task.set_target_from_configuration(self.configuration)
        return left_task, right_task, posture_task

    def arm_configuration(self):
        return self.data.qpos[self.arm_qpos].copy()

    def clip_arm_configuration(self, phi):
        left_range = self.model.actuator_ctrlrange[0:7]
        right_range = self.model.actuator_ctrlrange[8:15]
        lower = np.concatenate((left_range[:, 0], right_range[:, 0]))
        upper = np.concatenate((left_range[:, 1], right_range[:, 1]))
        return np.clip(phi, lower, upper)

    def command(self, phi, gripper_command):
        """Send arm position targets and the two gripper commands."""
        phi = np.asarray(phi, dtype=float)
        if phi.shape != (14,):
            raise ValueError(f"Expected 14 arm positions, got {phi.shape}")
        phi = self.clip_arm_configuration(phi)
        self.data.ctrl[0:7] = phi[:7]
        self.data.ctrl[8:15] = phi[7:]
        self.data.ctrl[7] = gripper_command
        self.data.ctrl[15] = gripper_command

    def step(self, viewer=None):
        """Advance one control period with model-bias compensation."""
        substeps = max(
            1, round(self.control_dt / self.model.opt.timestep)
        )
        for _ in range(substeps):
            if self.enable_bias_compensation:
                self.data.qfrc_applied[self.arm_dofs] = (
                    self.data.qfrc_bias[self.arm_dofs]
                )
            else:
                self.data.qfrc_applied[self.arm_dofs] = 0.0
            mujoco.mj_step(self.model, self.data)
        if viewer is not None:
            viewer.sync()

    def settle(self, viewer, rate, duration=1.0):
        for _ in range(int(duration / self.control_dt)):
            self.command(self.arm_configuration(), self.gripper_open)
            self.step(viewer)
            rate.sleep()

    def close_grippers(self, viewer, rate, duration=1.0):
        for _ in range(int(duration / self.control_dt)):
            self.command(self.arm_configuration(), self.gripper_closed)
            self.step(viewer)
            rate.sleep()

    def configure_viewer_camera(self, viewer):
        viewer.cam.lookat[:] = [0.1, 0.0, 0.1]
        viewer.cam.azimuth = 70
        viewer.cam.elevation = -20
        viewer.cam.distance = 2.5

    def launch_viewer(self):
        """Create the passive viewer used by the demonstration."""
        return mujoco.viewer.launch_passive(
            model=self.model,
            data=self.data,
            show_left_ui=False,
            show_right_ui=False,
        )

    def _site_quaternion(self, site_name):
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(
            quaternion, self.data.site_xmat[self.model.site(site_name).id]
        )
        return quaternion

    def _initialize_mocap_targets(self):
        pairs = (
            ("target_left", "site_left"),
            ("target_right", "site_right"),
        )
        quaternions = []
        for body_name, site_name in pairs:
            mocap_id = self.model.body(body_name).mocapid
            site_id = self.model.site(site_name).id
            self.data.mocap_pos[mocap_id] = self.data.site_xpos[site_id]
            quaternion = self._site_quaternion(site_name)
            self.data.mocap_quat[mocap_id] = quaternion
            quaternions.append(quaternion.copy())
        return tuple(quaternions)

    def _approach_waypoints(self):
        left_quaternion, right_quaternion = self._initialize_mocap_targets()
        left_position = self.data.site_xpos[
            self.model.site("site_left").id
        ].copy()
        right_position = self.data.site_xpos[
            self.model.site("site_right").id
        ].copy()

        left_pregrasp = left_position.copy()
        left_pregrasp[1] -= 0.05
        right_pregrasp = right_position.copy()
        right_pregrasp[1] += 0.05

        left_grasp = left_pregrasp.copy()
        left_grasp[1] += 0.07
        right_grasp = right_pregrasp.copy()
        right_grasp[1] -= 0.07

        return (
            [
                (left_pregrasp, left_quaternion.copy()),
                (left_grasp, left_quaternion.copy()),
            ],
            [
                (right_pregrasp, right_quaternion.copy()),
                (right_grasp, right_quaternion.copy()),
            ],
        )

    def _set_mocap_targets(self, left_target, right_target):
        for body_name, target in zip(
            ("target_left", "target_right"),
            (left_target, right_target),
        ):
            position, quaternion = target
            mocap_id = self.model.body(body_name).mocapid
            self.data.mocap_pos[mocap_id] = position
            self.data.mocap_quat[mocap_id] = quaternion

    @staticmethod
    def _quintic_scale(ratio):
        ratio = np.clip(ratio, 0.0, 1.0)
        return 10.0 * ratio**3 - 15.0 * ratio**4 + 6.0 * ratio**5

    @staticmethod
    def _interpolate_quaternion(start, target, scale):
        start_rotation = Rotation.from_quat(np.roll(start, -1))
        target_rotation = Rotation.from_quat(np.roll(target, -1))
        relative_rotvec = (
            target_rotation * start_rotation.inv()
        ).as_rotvec()
        interpolated = (
            Rotation.from_rotvec(scale * relative_rotvec) * start_rotation
        )
        return np.roll(interpolated.as_quat(), 1)

    @classmethod
    def _interpolate_pose(cls, start, target, scale):
        start_position, start_quaternion = start
        target_position, target_quaternion = target
        return (
            start_position + scale * (target_position - start_position),
            cls._interpolate_quaternion(
                start_quaternion, target_quaternion, scale
            ),
        )

    def _target_reached(self, left_target, right_target):
        def quaternion_distance(first, second):
            return min(
                np.linalg.norm(first - second),
                np.linalg.norm(first + second),
            )

        left_position, left_quaternion = left_target
        right_position, right_quaternion = right_target
        left_site = self.model.site("attachment_site_left").id
        right_site = self.model.site("attachment_site_right").id
        return (
            np.linalg.norm(self.data.site_xpos[left_site] - left_position)
            <= 0.008
            and quaternion_distance(
                self._site_quaternion("attachment_site_left"),
                left_quaternion,
            )
            <= 0.008
            and np.linalg.norm(
                self.data.site_xpos[right_site] - right_position
            )
            <= 0.008
            and quaternion_distance(
                self._site_quaternion("attachment_site_right"),
                right_quaternion,
            )
            <= 0.008
        )

    def _mink_control_step(self, viewer, rate):
        self.configuration.update(self.data.qpos)
        velocity = mink.solve_ik(
            self.configuration,
            self.tasks,
            self.control_dt,
            self.solver,
            damping=5e-3,
        )

        controlled_velocity = np.zeros_like(velocity)
        arm_velocity = velocity[self.arm_dofs]
        peak_speed = np.max(np.abs(arm_velocity))
        if peak_speed > self.maximum_approach_joint_speed:
            arm_velocity *= self.maximum_approach_joint_speed / peak_speed
        controlled_velocity[self.arm_dofs] = arm_velocity

        self.configuration.integrate_inplace(
            controlled_velocity, self.control_dt
        )
        phi = np.concatenate(
            (self.configuration.q[0:7], self.configuration.q[9:16])
        )
        self.command(phi, self.gripper_open)
        self.step(viewer)
        rate.sleep()

    def run_grasp_approach(self, viewer, rate):
        """Move both open grippers through the smooth grasp waypoints."""
        left_waypoints, right_waypoints = self._approach_waypoints()
        trajectory_steps = int(
            self.approach_segment_duration * self.control_hz
        )
        maximum_cycles = int(self.approach_timeout * self.control_hz)

        for index, (left_target, right_target) in enumerate(
            zip(left_waypoints, right_waypoints), start=1
        ):
            print(f"Smoothly approaching grasp waypoint {index}...")
            left_start = (
                self.data.site_xpos[
                    self.model.site("attachment_site_left").id
                ].copy(),
                self._site_quaternion("attachment_site_left"),
            )
            right_start = (
                self.data.site_xpos[
                    self.model.site("attachment_site_right").id
                ].copy(),
                self._site_quaternion("attachment_site_right"),
            )

            for step in range(1, trajectory_steps + 1):
                scale = self._quintic_scale(step / trajectory_steps)
                self._set_mocap_targets(
                    self._interpolate_pose(left_start, left_target, scale),
                    self._interpolate_pose(right_start, right_target, scale),
                )
                self._update_mink_targets()
                self._mink_control_step(viewer, rate)

            self._set_mocap_targets(left_target, right_target)
            self._update_mink_targets()
            for _ in range(maximum_cycles):
                if self._target_reached(left_target, right_target):
                    print(f"Grasp waypoint {index} reached.")
                    break
                self._mink_control_step(viewer, rate)
            else:
                raise RuntimeError(
                    f"Timed out reaching grasp waypoint {index}."
                )

    def _update_mink_targets(self):
        self.left_task.set_target(
            mink.SE3.from_mocap_name(
                self.model, self.data, "target_left"
            )
        )
        self.right_task.set_target(
            mink.SE3.from_mocap_name(
                self.model, self.data, "target_right"
            )
        )
