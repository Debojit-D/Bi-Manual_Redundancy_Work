"""MuJoCo scene and robot-control utilities for the dual-Franka demo.

Standalone preview examples::

    python -m bimanual_redundancy.simulation.scene
    python -m bimanual_redundancy.simulation.scene --front-view
    python -m bimanual_redundancy.simulation.scene --top-view

Press Ctrl+C at any time to close the viewer and stop cleanly.
"""

import argparse
from contextlib import AbstractContextManager
from pathlib import Path
from types import SimpleNamespace

import glfw
import mink
import mujoco
import mujoco.viewer
import numpy as np
from loop_rate_limiters import RateLimiter
from scipy.spatial.transform import Rotation

from bimanual_redundancy.systems import DUAL_FRANKA_PANDA
from bimanual_redundancy.simulation import cameras as camera_presets
from bimanual_redundancy.simulation.cli import add_camera_view_arguments, run_cli
from bimanual_redundancy.simulation.recording import HeadlessDualViewRecorder


class HeadlessSimulationViewer(AbstractContextManager):
    """Viewer-compatible no-render sink for automated controller smoke runs."""

    user_scn = None

    def __init__(self):
        self.cam = SimpleNamespace(
            distance=0.0,
            lookat=np.zeros(3),
            azimuth=0.0,
            elevation=0.0,
        )
        self._running = True

    def is_running(self):
        return self._running

    def sync(self):
        pass

    def close(self):
        self._running = False

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.close()


class CooperativeMuJoCoScene:
    """Own a specified MuJoCo model, actuators, targets, and grasp approach."""

    PERSPECTIVE_CAMERA_LOOKAT = np.array([0.1, 0.0, 0.1])
    FRONT_CAMERA_LOOKAT = np.array([0.3, 0.0, 0.25])
    TOP_CAMERA_LOOKAT = np.array([0.3, 0.0, 0.15])
    PERSPECTIVE_CAMERA_AZIMUTH = 140
    PERSPECTIVE_CAMERA_ELEVATION = -30
    FRONT_CAMERA_AZIMUTH = 180
    FRONT_CAMERA_ELEVATION = 0
    TOP_CAMERA_AZIMUTH = 180
    TOP_CAMERA_ELEVATION = -90
    PERSPECTIVE_CAMERA_DISTANCE = camera_presets.PERSPECTIVE_CAMERA_DISTANCE
    TOP_CAMERA_DISTANCE = camera_presets.TOP_CAMERA_DISTANCE
    FRONT_CAMERA_DISTANCE = camera_presets.FRONT_CAMERA_DISTANCE
    HEADLIGHT_AMBIENT = np.array([0.27, 0.27, 0.27])
    HEADLIGHT_DIFFUSE = np.array([0.55, 0.55, 0.55])
    MODEL_LIGHT_INTENSITY_SCALE = 0.90

    DEFAULT_SYSTEM_SPEC = DUAL_FRANKA_PANDA
    DEFAULT_MODEL_PATH = DEFAULT_SYSTEM_SPEC.model_path
    LEFT_JOINT_NAMES = DEFAULT_SYSTEM_SPEC.left_arm.joint_names
    RIGHT_JOINT_NAMES = DEFAULT_SYSTEM_SPEC.right_arm.joint_names
    DEFAULT_LEFT_ARM_BASE_POSITION = np.array([0.0, -0.2, 0.0])
    DEFAULT_RIGHT_ARM_BASE_POSITION = np.array([0.0, 0.2, 0.0])
    DEFAULT_LEFT_ARM_BASE_EULER_XYZ = np.zeros(3)
    DEFAULT_RIGHT_ARM_BASE_EULER_XYZ = np.zeros(3)
    MOUNTING_PLATFORM_HALF_SIZE_XY = np.array([0.16, 0.58])
    MOUNTING_PLATFORM_RGBA = np.array([0.28, 0.30, 0.34, 1.0])
    MOUNTING_PLATFORM_GEOM = DEFAULT_SYSTEM_SPEC.mounting_platform_geom

    def __init__(
        self,
        *,
        system_spec=None,
        model_path=None,
        left_arm_base_position=None,
        right_arm_base_position=None,
        left_arm_base_euler_xyz=None,
        right_arm_base_euler_xyz=None,
        left_arm_base_euler_xyz_degrees=None,
        right_arm_base_euler_xyz_degrees=None,
        control_hz=50.0,
        solver="daqp",
        enable_bias_compensation=True,
        show_mocap_targets=False,
        gripper_open=255.0,
        gripper_closed=0.0,
        approach_timeout=20.0,
        approach_segment_duration=3.0,
        maximum_approach_joint_speed=0.6,
        pregrasp_distance=0.10,
        grasp_penetration=0.02,
        postgrasp_outward_distance=0.14,
        postgrasp_vertical_offset=0.09,
        use_alternate_grasp_orientation=False,
    ):
        self.system_spec = system_spec or self.DEFAULT_SYSTEM_SPEC
        self.model_path = Path(model_path or self.system_spec.model_path)
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
        self.pregrasp_distance = float(pregrasp_distance)
        self.grasp_penetration = float(grasp_penetration)
        self.postgrasp_outward_distance = float(
            postgrasp_outward_distance
        )
        self.postgrasp_vertical_offset = float(postgrasp_vertical_offset)
        self.use_alternate_grasp_orientation = bool(
            use_alternate_grasp_orientation
        )
        if (
            self.pregrasp_distance < 0.0
            or self.grasp_penetration < 0.0
            or self.postgrasp_outward_distance < 0.0
            or self.postgrasp_vertical_offset < 0.0
        ):
            raise ValueError(
                "pregrasp_distance, grasp_penetration, and "
                "postgrasp offsets must be nonnegative"
            )

        self.model = mujoco.MjModel.from_xml_path(self.model_path.as_posix())
        self.left_arm_actuators = self._actuator_indices(
            self.system_spec.left_arm.actuator_names
        )
        self.right_arm_actuators = self._actuator_indices(
            self.system_spec.right_arm.actuator_names
        )
        self.left_gripper_actuator = self.model.actuator(
            self.system_spec.left_arm.gripper_actuator
        ).id
        self.right_gripper_actuator = self.model.actuator(
            self.system_spec.right_arm.gripper_actuator
        ).id
        self._configure_lighting()
        self._set_arm_base_poses(
            left_arm_base_position,
            right_arm_base_position,
            left_arm_base_euler_xyz,
            right_arm_base_euler_xyz,
            left_arm_base_euler_xyz_degrees,
            right_arm_base_euler_xyz_degrees,
        )
        self._set_mocap_target_visibility(show_mocap_targets)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(
            self.model, self.data, self.model.key(self.system_spec.home_keyframe).id
        )
        # Establish the intended open-gripper command before the first forward
        # dynamics evaluation.  This also protects against future keyframes
        # that omit actuator controls.
        self.data.ctrl[self.left_gripper_actuator] = self.gripper_open
        self.data.ctrl[self.right_gripper_actuator] = self.gripper_open
        mujoco.mj_forward(self.model, self.data)

        self.configuration = mink.Configuration(self.model)
        self.configuration.update(self.data.qpos)
        self.left_arm_qpos, self.left_arm_dofs = self._joint_indices(
            self.system_spec.left_arm.joint_names
        )
        self.right_arm_qpos, self.right_arm_dofs = self._joint_indices(
            self.system_spec.right_arm.joint_names
        )
        self.arm_qpos = np.concatenate(
            (self.left_arm_qpos, self.right_arm_qpos)
        )
        self.arm_dofs = np.concatenate(
            (self.left_arm_dofs, self.right_arm_dofs)
        )
        self.joint_position_limits = np.vstack(
            (
                self.model.actuator_ctrlrange[self.left_arm_actuators],
                self.model.actuator_ctrlrange[self.right_arm_actuators],
            )
        )
        self.home_arm_configuration = self.arm_configuration()
        self.left_task, self.right_task, self.posture_task = self._make_tasks()
        self.tasks = [self.left_task, self.right_task, self.posture_task]

    def set_table_reference_pose(self, position, rotation=None):
        """Place the specified object reference at a requested world pose."""
        position = np.asarray(position, dtype=float)
        if position.shape != (3,):
            raise ValueError("Table reference position must have shape (3,)")
        object_spec = self.system_spec.object
        site_id = self.model.site(object_spec.reference_site).id
        body_id = self.model.body(object_spec.body).id
        joint_id = self.model.joint(object_spec.joint).id

        current_body_rotation = self.data.xmat[body_id].reshape(3, 3)
        current_site_rotation = self.data.site_xmat[site_id].reshape(3, 3)
        body_to_site_rotation = current_body_rotation.T @ current_site_rotation
        body_to_site_position = current_body_rotation.T @ (
            self.data.site_xpos[site_id] - self.data.xpos[body_id]
        )

        if rotation is None:
            rotation = current_site_rotation.copy()
        rotation = np.asarray(rotation, dtype=float)
        if rotation.shape != (3, 3):
            raise ValueError("Table reference rotation must have shape (3, 3)")

        desired_body_rotation = rotation @ body_to_site_rotation.T
        desired_body_position = position - (
            desired_body_rotation @ body_to_site_position
        )
        qpos_address = self.model.jnt_qposadr[joint_id]
        dof_address = self.model.jnt_dofadr[joint_id]
        quaternion_xyzw = Rotation.from_matrix(
            desired_body_rotation
        ).as_quat()
        self.data.qpos[qpos_address : qpos_address + 3] = desired_body_position
        self.data.qpos[qpos_address + 3 : qpos_address + 7] = np.roll(
            quaternion_xyzw,
            1,
        )
        self.data.qvel[dof_address : dof_address + 6] = 0.0
        mujoco.mj_forward(self.model, self.data)
        self.configuration.update(self.data.qpos)
        self.posture_task.set_target_from_configuration(self.configuration)

    def _set_arm_base_poses(
        self,
        left_arm_base_position,
        right_arm_base_position,
        left_arm_base_euler_xyz,
        right_arm_base_euler_xyz,
        left_arm_base_euler_xyz_degrees,
        right_arm_base_euler_xyz_degrees,
    ):
        """Set fixed robot-base positions and extrinsic XYZ Euler rotations."""
        left_position = (
            self.DEFAULT_LEFT_ARM_BASE_POSITION
            if left_arm_base_position is None
            else np.asarray(left_arm_base_position, dtype=float)
        )
        right_position = (
            self.DEFAULT_RIGHT_ARM_BASE_POSITION
            if right_arm_base_position is None
            else np.asarray(right_arm_base_position, dtype=float)
        )
        left_euler = self._resolve_euler_xyz(
            left_arm_base_euler_xyz,
            left_arm_base_euler_xyz_degrees,
            self.DEFAULT_LEFT_ARM_BASE_EULER_XYZ,
            "left",
        )
        right_euler = self._resolve_euler_xyz(
            right_arm_base_euler_xyz,
            right_arm_base_euler_xyz_degrees,
            self.DEFAULT_RIGHT_ARM_BASE_EULER_XYZ,
            "right",
        )
        if left_position.shape != (3,) or right_position.shape != (3,):
            raise ValueError(
                "Arm base positions must be xyz vectors of shape (3,)"
            )
        if left_euler.shape != (3,) or right_euler.shape != (3,):
            raise ValueError(
                "Arm base Euler orientations must have shape (3,)"
            )

        left_body_id = self.model.body(self.system_spec.left_arm.base_body).id
        right_body_id = self.model.body(self.system_spec.right_arm.base_body).id
        self.model.body_pos[left_body_id] = left_position
        self.model.body_pos[right_body_id] = right_position
        self.model.body_quat[left_body_id] = np.roll(
            Rotation.from_euler("xyz", left_euler).as_quat(),
            1,
        )
        self.model.body_quat[right_body_id] = np.roll(
            Rotation.from_euler("xyz", right_euler).as_quat(),
            1,
        )
        self._set_mounting_platform(left_position, right_position)

    @staticmethod
    def _resolve_euler_xyz(radians, degrees, default, side):
        """Return one Euler XYZ vector in radians from either accepted unit."""
        if radians is not None and degrees is not None:
            raise ValueError(
                f"Specify the {side} arm Euler orientation in either radians "
                "or degrees, not both"
            )
        if degrees is not None:
            return np.deg2rad(np.asarray(degrees, dtype=float))
        if radians is not None:
            return np.asarray(radians, dtype=float)
        return np.asarray(default, dtype=float)

    def _set_mounting_platform(self, left_position, right_position):
        """Place one fixed-footprint base below both elevated robots."""
        geom_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_GEOM,
            self.system_spec.mounting_platform_geom or "",
        )
        # Keep custom scene files without the optional pedestal geoms usable.
        if geom_id < 0:
            return

        left_height = float(left_position[2])
        right_height = float(right_position[2])
        if not np.isclose(left_height, right_height, atol=1e-9):
            raise ValueError(
                "A shared mounting platform requires equal left and right "
                "arm spawn z coordinates"
            )

        height = left_height
        center_xy = 0.5 * (left_position[:2] + right_position[:2])
        if height <= 0.0:
            self.model.geom_pos[geom_id] = [
                center_xy[0],
                center_xy[1],
                -1.0,
            ]
            self.model.geom_rgba[geom_id, 3] = 0.0
            self.model.geom_contype[geom_id] = 0
            self.model.geom_conaffinity[geom_id] = 0
            return

        self.model.geom_pos[geom_id] = [
            center_xy[0],
            center_xy[1],
            0.5 * height,
        ]
        self.model.geom_size[geom_id] = [
            self.MOUNTING_PLATFORM_HALF_SIZE_XY[0],
            self.MOUNTING_PLATFORM_HALF_SIZE_XY[1],
            0.5 * height,
        ]
        # geom_size is runtime-configurable, while these broad-phase bounds
        # are compile-time fields. Keep them synchronized with the new box.
        self.model.geom_aabb[geom_id, :3] = 0.0
        self.model.geom_aabb[geom_id, 3:] = self.model.geom_size[geom_id]
        self.model.geom_rbound[geom_id] = np.linalg.norm(
            self.model.geom_size[geom_id]
        )
        self.model.geom_rgba[geom_id] = self.MOUNTING_PLATFORM_RGBA
        self.model.geom_contype[geom_id] = 1
        self.model.geom_conaffinity[geom_id] = 1

    def _joint_indices(self, joint_names):
        joint_ids = np.array(
            [self.model.joint(name).id for name in joint_names], dtype=int
        )
        return (
            self.model.jnt_qposadr[joint_ids],
            self.model.jnt_dofadr[joint_ids],
        )

    def _actuator_indices(self, actuator_names):
        return np.array(
            [self.model.actuator(name).id for name in actuator_names],
            dtype=int,
        )

    def _set_mocap_target_visibility(self, visible):
        alpha = 1.0 if visible else 0.0
        target_body_ids = {
            self.model.body(self.system_spec.left_arm.target_body).id,
            self.model.body(self.system_spec.right_arm.target_body).id,
        }
        for geom_id in range(self.model.ngeom):
            if self.model.geom_bodyid[geom_id] in target_body_ids:
                self.model.geom_rgba[geom_id, 3] = alpha

    def _make_tasks(self):
        left_task = mink.FrameTask(
            self.system_spec.left_arm.hand_site,
            "site",
            position_cost=1.0,
            orientation_cost=1.0,
            lm_damping=1.0,
        )
        right_task = mink.FrameTask(
            self.system_spec.right_arm.hand_site,
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

    def make_kinematics(self):
        """Build generic cooperative kinematics from this scene's spec."""
        from bimanual_redundancy.core import CooperativeManipulationKinematics

        return CooperativeManipulationKinematics(
            self.model,
            self.left_arm_dofs,
            self.right_arm_dofs,
            system_spec=self.system_spec,
        )

    def clip_arm_configuration(self, phi):
        lower = self.joint_position_limits[:, 0]
        upper = self.joint_position_limits[:, 1]
        return np.clip(phi, lower, upper)

    def command(self, phi, gripper_command):
        """Send arm position targets and the two gripper commands."""
        phi = np.asarray(phi, dtype=float)
        if phi.shape != (14,):
            raise ValueError(f"Expected 14 arm positions, got {phi.shape}")
        phi = self.clip_arm_configuration(phi)
        split = self.left_arm_qpos.size
        self.data.ctrl[self.left_arm_actuators] = phi[:split]
        self.data.ctrl[self.right_arm_actuators] = phi[split:]
        self.data.ctrl[self.left_gripper_actuator] = gripper_command
        self.data.ctrl[self.right_gripper_actuator] = gripper_command

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

    def open_grippers(self, viewer, rate, duration=1.0):
        """Open both grippers while holding the current arm configuration."""
        for _ in range(int(duration / self.control_dt)):
            if not viewer.is_running():
                return False
            self.command(self.arm_configuration(), self.gripper_open)
            self.step(viewer)
            rate.sleep()
        return True

    def configure_viewer_camera(
        self,
        viewer,
        *,
        top_view=False,
        front_view=False,
    ):
        """Apply the camera preset and keep the infinite background dark."""
        if top_view and front_view:
            raise ValueError("top_view and front_view are mutually exclusive")
        if viewer.user_scn is not None:
            viewer.user_scn.flags[mujoco.mjtRndFlag.mjRND_SKYBOX] = 0
        if top_view:
            viewer.cam.distance = self.TOP_CAMERA_DISTANCE
            viewer.cam.lookat[:] = self.TOP_CAMERA_LOOKAT
            # Rotate the overhead image 90 degrees anticlockwise in-plane.
            viewer.cam.azimuth = self.TOP_CAMERA_AZIMUTH
            viewer.cam.elevation = self.TOP_CAMERA_ELEVATION
        elif front_view:
            viewer.cam.distance = self.FRONT_CAMERA_DISTANCE
            viewer.cam.lookat[:] = self.FRONT_CAMERA_LOOKAT
            viewer.cam.azimuth = self.FRONT_CAMERA_AZIMUTH
            viewer.cam.elevation = self.FRONT_CAMERA_ELEVATION
        else:
            viewer.cam.distance = self.PERSPECTIVE_CAMERA_DISTANCE
            viewer.cam.lookat[:] = self.PERSPECTIVE_CAMERA_LOOKAT
            viewer.cam.azimuth = self.PERSPECTIVE_CAMERA_AZIMUTH
            viewer.cam.elevation = self.PERSPECTIVE_CAMERA_ELEVATION

    def _configure_lighting(self):
        """Apply one slightly dimmer lighting treatment to every scene."""
        self.model.vis.headlight.ambient[:] = self.HEADLIGHT_AMBIENT
        self.model.vis.headlight.diffuse[:] = self.HEADLIGHT_DIFFUSE
        self.model.light_diffuse[:] *= self.MODEL_LIGHT_INTENSITY_SCALE
        self.model.light_specular[:] *= self.MODEL_LIGHT_INTENSITY_SCALE

    def launch_viewer(self, *, maximized=True):
        """Create the passive viewer, maximized by default.

        MuJoCo's passive-viewer handle does not expose its native window, so
        the initial window state must be requested through GLFW before the
        viewer creates it. ``launch_passive`` returns only after that window
        exists, allowing the global hint to be reset safely afterwards.
        """
        if maximized:
            if not glfw.init():
                raise RuntimeError("GLFW initialization failed")
            glfw.window_hint(glfw.MAXIMIZED, glfw.TRUE)
        try:
            return mujoco.viewer.launch_passive(
                model=self.model,
                data=self.data,
                show_left_ui=False,
                show_right_ui=False,
            )
        finally:
            if maximized:
                glfw.default_window_hints()

    def launch_headless(self):
        """Advance dynamics without OpenGL; intended for smoke validation."""
        return HeadlessSimulationViewer()

    def launch_video_recorder(
        self,
        output_dir,
        *,
        width=1280,
        height=720,
        fps=30,
        views=None,
        encoder="x264",
        nvenc_view_limit=None,
    ):
        """Create a headless recorder for perspective and overhead views."""
        return HeadlessDualViewRecorder(
            self.model,
            self.data,
            output_dir,
            perspective_lookat=self.PERSPECTIVE_CAMERA_LOOKAT,
            top_lookat=self.TOP_CAMERA_LOOKAT,
            front_lookat=self.FRONT_CAMERA_LOOKAT,
            perspective_azimuth=self.PERSPECTIVE_CAMERA_AZIMUTH,
            perspective_elevation=self.PERSPECTIVE_CAMERA_ELEVATION,
            top_azimuth=self.TOP_CAMERA_AZIMUTH,
            top_elevation=self.TOP_CAMERA_ELEVATION,
            front_azimuth=self.FRONT_CAMERA_AZIMUTH,
            front_elevation=self.FRONT_CAMERA_ELEVATION,
            perspective_distance=self.PERSPECTIVE_CAMERA_DISTANCE,
            top_distance=self.TOP_CAMERA_DISTANCE,
            front_distance=self.FRONT_CAMERA_DISTANCE,
            control_hz=self.control_hz,
            width=width,
            height=height,
            fps=fps,
            views=views,
            encoder=encoder,
            nvenc_view_limit=nvenc_view_limit,
        )

    def _site_quaternion(self, site_name):
        quaternion = np.empty(4)
        mujoco.mju_mat2Quat(
            quaternion, self.data.site_xmat[self.model.site(site_name).id]
        )
        return quaternion

    def _initialize_mocap_targets(self):
        pairs = (
            (
                self.system_spec.left_arm.target_body,
                self.system_spec.object.contact_sites[0],
            ),
            (
                self.system_spec.right_arm.target_body,
                self.system_spec.object.contact_sites[1],
            ),
        )
        quaternions = []
        for body_name, site_name in pairs:
            mocap_id = self.model.body(body_name).mocapid
            site_id = self.model.site(site_name).id
            self.data.mocap_pos[mocap_id] = self.data.site_xpos[site_id]
            if self.use_alternate_grasp_orientation:
                # A parallel-jaw grasp is unchanged when the hand is rotated
                # 180 degrees about its local approach axis. This equivalent
                # frame supports legacy/custom positive-roll table poses.
                site_rotation = self.data.site_xmat[site_id].reshape(3, 3)
                alternate_rotation = site_rotation @ Rotation.from_rotvec(
                    [0.0, 0.0, np.pi]
                ).as_matrix()
                quaternion = np.roll(
                    Rotation.from_matrix(alternate_rotation).as_quat(),
                    1,
                )
            else:
                quaternion = self._site_quaternion(site_name)
            self.data.mocap_quat[mocap_id] = quaternion
            quaternions.append(quaternion.copy())
        return tuple(quaternions)

    def _approach_waypoints(self):
        left_quaternion, right_quaternion = self._initialize_mocap_targets()
        object_position = self.data.site_xpos[
            self.model.site(self.system_spec.object.reference_site).id
        ].copy()
        left_position = self.data.site_xpos[
            self.model.site(self.system_spec.object.contact_sites[0]).id
        ].copy()
        right_position = self.data.site_xpos[
            self.model.site(self.system_spec.object.contact_sites[1]).id
        ].copy()

        # Derive each approach direction from the live object geometry. These
        # vectors therefore translate and rotate with arbitrary object poses,
        # unlike fixed offsets along a world axis.
        left_outward = left_position - object_position
        right_outward = right_position - object_position
        left_norm = np.linalg.norm(left_outward)
        right_norm = np.linalg.norm(right_outward)
        if left_norm <= 1e-9 or right_norm <= 1e-9:
            raise ValueError("Object contact sites must differ from its reference site")
        left_outward /= left_norm
        right_outward /= right_norm

        left_pregrasp = (
            left_position + self.pregrasp_distance * left_outward
        )
        right_pregrasp = (
            right_position + self.pregrasp_distance * right_outward
        )
        left_grasp = (
            left_position - self.grasp_penetration * left_outward
        )
        right_grasp = (
            right_position - self.grasp_penetration * right_outward
        )

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
            (
                self.system_spec.left_arm.target_body,
                self.system_spec.right_arm.target_body,
            ),
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
        left_hand_site = self.system_spec.left_arm.hand_site
        right_hand_site = self.system_spec.right_arm.hand_site
        left_site = self.model.site(left_hand_site).id
        right_site = self.model.site(right_hand_site).id
        return (
            np.linalg.norm(self.data.site_xpos[left_site] - left_position)
            <= 0.008
            and quaternion_distance(
                self._site_quaternion(left_hand_site),
                left_quaternion,
            )
            <= 0.008
            and np.linalg.norm(
                self.data.site_xpos[right_site] - right_position
            )
            <= 0.008
            and quaternion_distance(
                self._site_quaternion(right_hand_site),
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
        phi = self.configuration.q[self.arm_qpos].copy()
        self.command(phi, self.gripper_open)
        self.step(viewer)
        rate.sleep()

    def _run_gripper_waypoints(
        self,
        viewer,
        rate,
        left_waypoints,
        right_waypoints,
        *,
        action,
    ):
        """Move both open grippers through paired Cartesian waypoints."""
        trajectory_steps = int(
            self.approach_segment_duration * self.control_hz
        )
        maximum_cycles = int(self.approach_timeout * self.control_hz)

        for index, (left_target, right_target) in enumerate(
            zip(left_waypoints, right_waypoints), start=1
        ):
            if not viewer.is_running():
                return False
            print(f"{action} waypoint {index}...")
            left_start = (
                self.data.site_xpos[
                    self.model.site(self.system_spec.left_arm.hand_site).id
                ].copy(),
                self._site_quaternion(self.system_spec.left_arm.hand_site),
            )
            right_start = (
                self.data.site_xpos[
                    self.model.site(self.system_spec.right_arm.hand_site).id
                ].copy(),
                self._site_quaternion(self.system_spec.right_arm.hand_site),
            )

            for step in range(1, trajectory_steps + 1):
                if not viewer.is_running():
                    return False
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
                if not viewer.is_running():
                    return False
                if self._target_reached(left_target, right_target):
                    print(f"{action} waypoint {index} reached.")
                    break
                self._mink_control_step(viewer, rate)
            else:
                raise RuntimeError(
                    f"Timed out reaching {action.lower()} waypoint {index}."
                )
        return True

    def run_grasp_approach(self, viewer, rate):
        """Move both open grippers through the smooth grasp waypoints."""
        left_waypoints, right_waypoints = self._approach_waypoints()
        return self._run_gripper_waypoints(
            viewer,
            rate,
            left_waypoints,
            right_waypoints,
            action="Smoothly approaching grasp",
        )

    def return_arms_home(
        self,
        viewer,
        rate,
        *,
        duration=4.0,
        settle_duration=0.5,
    ):
        """Return both open-gripper arms smoothly to the home1 keyframe."""
        if duration <= 0.0:
            raise ValueError("Home-return duration must be positive")
        if settle_duration < 0.0:
            raise ValueError("Home settle duration cannot be negative")
        start = self.arm_configuration()
        number_of_steps = max(1, int(round(duration * self.control_hz)))
        print("Returning both arms to the home configuration...")
        for step in range(1, number_of_steps + 1):
            if not viewer.is_running():
                return False
            scale = self._quintic_scale(step / number_of_steps)
            target = start + scale * (self.home_arm_configuration - start)
            self.command(target, self.gripper_open)
            self.step(viewer)
            rate.sleep()
        for _ in range(int(round(settle_duration * self.control_hz))):
            if not viewer.is_running():
                return False
            self.command(self.home_arm_configuration, self.gripper_open)
            self.step(viewer)
            rate.sleep()
        self.configuration.update(self.data.qpos)
        self.posture_task.set_target_from_configuration(self.configuration)
        print("Home configuration reached.")
        return True

    def run_grasp_disengagement(self, viewer, rate):
        """Release, retreat outward and upward, then return both arms home."""
        if not viewer.is_running():
            return False
        if getattr(viewer, "user_scn", None) is not None:
            viewer.user_scn.ngeom = 0
        left_waypoints, right_waypoints = self._approach_waypoints()
        # Build a distinct post-grasp target from the live contact geometry,
        # then lift both grippers in world Z to clear the tabletop.
        vertical_offset = np.array(
            [0.0, 0.0, self.postgrasp_vertical_offset]
        )
        object_position = self.data.site_xpos[
            self.model.site(self.system_spec.object.reference_site).id
        ]
        left_contact = self.data.site_xpos[
            self.model.site(self.system_spec.object.contact_sites[0]).id
        ]
        right_contact = self.data.site_xpos[
            self.model.site(self.system_spec.object.contact_sites[1]).id
        ]
        left_outward = left_contact - object_position
        right_outward = right_contact - object_position
        left_outward /= np.linalg.norm(left_outward)
        right_outward /= np.linalg.norm(right_outward)
        _, left_quaternion = left_waypoints[0]
        _, right_quaternion = right_waypoints[0]
        left_postgrasp = [
            (
                left_contact
                + self.postgrasp_outward_distance * left_outward
                + vertical_offset,
                left_quaternion.copy(),
            )
        ]
        right_postgrasp = [
            (
                right_contact
                + self.postgrasp_outward_distance * right_outward
                + vertical_offset,
                right_quaternion.copy(),
            )
        ]
        print("Opening both grippers at the placed pose...")
        if not self.open_grippers(viewer, rate):
            return False
        completed = self._run_gripper_waypoints(
            viewer,
            rate,
            left_postgrasp,
            right_postgrasp,
            action="Retreating to post-grasp",
        )
        if not completed:
            return False
        return self.return_arms_home(viewer, rate)

    def _update_mink_targets(self):
        self.left_task.set_target(
            mink.SE3.from_mocap_name(
                self.model, self.data, self.system_spec.left_arm.target_body
            )
        )
        self.right_task.set_target(
            mink.SE3.from_mocap_name(
                self.model, self.data, self.system_spec.right_arm.target_body
            )
        )


# Historical public name retained for the reference paper runners.
DualFrankaMuJoCoScene = CooperativeMuJoCoScene


# Standalone scene-preview settings [x, y, z] in the world frame.
PREVIEW_LEFT_ARM_SPAWN_POSITION = np.array([0.0, -0.2, 0.0])
PREVIEW_RIGHT_ARM_SPAWN_POSITION = np.array([0.0, 0.2, 0.0])
PREVIEW_SHOW_MOCAP_TARGETS = False
PREVIEW_CONTROL_HZ = 50.0


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Preview the shared dual-Franka scene until the viewer closes."
        )
    )
    add_camera_view_arguments(parser, scope="scene preview")
    return parser.parse_args()


def main():
    """Open the shared scene at home and keep it active until viewer closure."""
    arguments = parse_arguments()
    scene = DualFrankaMuJoCoScene(
        control_hz=PREVIEW_CONTROL_HZ,
        left_arm_base_position=PREVIEW_LEFT_ARM_SPAWN_POSITION,
        right_arm_base_position=PREVIEW_RIGHT_ARM_SPAWN_POSITION,
        show_mocap_targets=PREVIEW_SHOW_MOCAP_TARGETS,
        enable_bias_compensation=True,
    )
    rate = RateLimiter(frequency=PREVIEW_CONTROL_HZ, warn=False)
    home_configuration = scene.arm_configuration()

    with scene.launch_viewer() as viewer:
        scene.configure_viewer_camera(
            viewer,
            top_view=arguments.top_view,
            front_view=arguments.front_view,
        )
        while viewer.is_running():
            scene.command(home_configuration, scene.gripper_open)
            scene.step(viewer)
            rate.sleep()


if __name__ == "__main__":
    run_cli(main)
