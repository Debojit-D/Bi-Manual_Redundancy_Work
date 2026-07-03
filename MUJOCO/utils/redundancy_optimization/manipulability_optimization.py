"""Task-specific manipulability objectives from Equations (13)-(15)."""

from dataclasses import dataclass
from enum import Enum

import mujoco
import numpy as np


LEFT_COLLISION_BODIES = (
    "link3_l",
    "link4_l",
    "link5_l",
    "link6_l",
    "link7_l",
    "hand_l",
)

RIGHT_COLLISION_BODIES = (
    "link3_r",
    "link4_r",
    "link5_r",
    "link6_r",
    "link7_r",
    "hand_r",
)


class ManipulabilityObjective(str, Enum):
    """The three redundancy objectives compared in the paper."""

    VELOCITY = "velocity"
    FORCE = "force"
    DIRECTIONAL_FORCE = "directional_force"


class CollisionModelVersion(str, Enum):
    """Selectable inter-arm collision approximations."""

    VERSION1 = "version1"
    VERSION2 = "version2"


@dataclass(frozen=True)
class OptimizationResult:
    objective: ManipulabilityObjective
    value: float
    gradient: np.ndarray
    phi_dot_opt: np.ndarray


class ManipulabilityOptimizer:
    """Evaluate paper costs and produce phi_dot_opt=Lambda*dW/dphi.

    Velocity and force manipulability are maximized.  Directional-force
    distance is minimized, so its gradient sign is reversed automatically.
    Gradients are evaluated with central joint-space finite differences.
    """

    def __init__(
        self,
        kinematics,
        arm_qpos_indices,
        *,
        objective=ManipulabilityObjective.VELOCITY,
        gain=1.0,
        finite_difference_step=1e-4,
        maximum_joint_speed=0.15,
        desired_wrench_direction=(0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        characteristic_length=0.4,
        determinant_floor=1e-18,
        enable_collision_penalty=False,
        collision_weight=25.0,
        collision_safety_margin=0.08,
        collision_proximity_scale=0.01,
        collision_sphere_radius=0.07,
        collision_version=CollisionModelVersion.VERSION1,
        collision_sphere_model_path=None,
    ):
        self.kinematics = kinematics
        self.model = kinematics.model
        self.arm_qpos_indices = np.asarray(arm_qpos_indices, dtype=int)
        self.objective = ManipulabilityObjective(objective)
        self.gain = float(gain)
        self.finite_difference_step = float(finite_difference_step)
        self.maximum_joint_speed = float(maximum_joint_speed)
        self.characteristic_length = float(characteristic_length)
        self.determinant_floor = float(determinant_floor)
        self.enable_collision_penalty = bool(enable_collision_penalty)
        self.collision_weight = float(collision_weight)
        self.collision_safety_margin = float(collision_safety_margin)
        self.collision_proximity_scale = float(collision_proximity_scale)
        self.collision_sphere_radius = float(collision_sphere_radius)
        self.collision_version = CollisionModelVersion(collision_version)
        # Resolve names once: finite-difference evaluations reuse these IDs.
        self.left_collision_body_ids = np.array(
            [self.model.body(name).id for name in LEFT_COLLISION_BODIES],
            dtype=int,
        )
        self.right_collision_body_ids = np.array(
            [self.model.body(name).id for name in RIGHT_COLLISION_BODIES],
            dtype=int,
        )
        self.left_detailed_body_ids = np.empty(0, dtype=int)
        self.left_detailed_centers = np.empty((0, 3))
        self.left_detailed_radii = np.empty(0)
        self.right_detailed_body_ids = np.empty(0, dtype=int)
        self.right_detailed_centers = np.empty((0, 3))
        self.right_detailed_radii = np.empty(0)
        if self.collision_version is CollisionModelVersion.VERSION2:
            self._load_detailed_collision_spheres(collision_sphere_model_path)
        self.desired_force_matrix = self._make_direction_matrix(
            desired_wrench_direction
        )

        if self.arm_qpos_indices.size != self.kinematics.arm_dofs.size:
            raise ValueError("Expected one qpos index for every controlled DoF")
        if self.finite_difference_step <= 0.0:
            raise ValueError("finite_difference_step must be positive")
        if self.characteristic_length <= 0.0:
            raise ValueError("characteristic_length must be positive")
        if self.collision_weight < 0.0:
            raise ValueError("collision_weight must be nonnegative")
        if self.collision_safety_margin < 0.0:
            raise ValueError("collision_safety_margin must be nonnegative")
        if self.collision_proximity_scale <= 0.0:
            raise ValueError("collision_proximity_scale must be positive")
        if self.collision_sphere_radius < 0.0:
            raise ValueError("collision_sphere_radius must be nonnegative")

    @staticmethod
    def _is_descendant(model, body_id, root_id):
        while body_id > 0:
            if body_id == root_id:
                return True
            body_id = int(model.body_parentid[body_id])
        return False

    def _load_detailed_collision_spheres(self, model_path):
        """Load fitted body-local spheres and map them onto this MuJoCo model."""
        if model_path is None:
            raise ValueError(
                "collision_sphere_model_path is required for collision version2"
            )
        sphere_model = mujoco.MjModel.from_xml_path(str(model_path))
        left_root = sphere_model.body("franka1").id
        right_root = sphere_model.body("franka2").id
        # Fixed bases cannot move away from one another, so exclude them from
        # the optimization cost and retain all articulated arm/hand spheres.
        excluded_bodies = {"franka1", "franka2", "link0_l", "link0_r"}
        detailed = {"left": [], "right": []}

        for geom_id in range(sphere_model.ngeom):
            geom_name = sphere_model.geom(geom_id).name
            if not geom_name.startswith("spherefit_"):
                continue
            body_id = int(sphere_model.geom_bodyid[geom_id])
            body_name = sphere_model.body(body_id).name
            if body_name in excluded_bodies:
                continue
            if self._is_descendant(sphere_model, body_id, left_root):
                side = "left"
            elif self._is_descendant(sphere_model, body_id, right_root):
                side = "right"
            else:
                continue

            target_body_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_BODY,
                body_name,
            )
            if target_body_id < 0:
                raise ValueError(
                    f"Sphere body {body_name!r} is absent from the target model"
                )
            detailed[side].append(
                (
                    target_body_id,
                    sphere_model.geom_pos[geom_id].copy(),
                    float(sphere_model.geom_size[geom_id, 0]),
                )
            )

        if not detailed["left"] or not detailed["right"]:
            raise ValueError(
                f"No dual-arm fitted spheres found in {model_path}"
            )

        for side in ("left", "right"):
            entries = detailed[side]
            setattr(
                self,
                f"{side}_detailed_body_ids",
                np.array([entry[0] for entry in entries], dtype=int),
            )
            setattr(
                self,
                f"{side}_detailed_centers",
                np.array([entry[1] for entry in entries], dtype=float),
            )
            setattr(
                self,
                f"{side}_detailed_radii",
                np.array([entry[2] for entry in entries], dtype=float),
            )

    @staticmethod
    def _sphere_world_positions(data, body_ids, local_centers):
        rotations = data.xmat[body_ids].reshape(-1, 3, 3)
        return data.xpos[body_ids] + np.einsum(
            "nij,nj->ni",
            rotations,
            local_centers,
        )

    def _make_direction_matrix(self, wrench_direction):
        weights = np.asarray(wrench_direction, dtype=float).copy()
        if weights.shape != (6,):
            raise ValueError("desired_wrench_direction must contain six values")
        # Equation (15) defines F=diag(Fx,Fy,Fz,Mx,My,Mz).  Manipulability
        # ellipsoids are sign-symmetric, so load-axis weights are nonnegative.
        # Spatial moment entries are converted to force-equivalent units first.
        weights = np.abs(weights)
        weights[3:] /= self.characteristic_length
        if np.sum(weights) <= 0.0:
            raise ValueError("desired_wrench_direction cannot be zero")
        return np.diag(weights)

    def velocity_manipulability(self, data):
        """Equation (13): sqrt(det(A A.T))."""
        velocity_map = self.kinematics.paper_object_velocity_map(data)
        return self._sqrt_determinant(velocity_map @ velocity_map.T)

    def force_manipulability(self, data):
        """Equation (14): sqrt(det((A A.T)^dagger))."""
        velocity_map = self.kinematics.paper_object_velocity_map(data)
        force_matrix = np.linalg.pinv(
            velocity_map @ velocity_map.T,
            rcond=self.kinematics.pinv_rcond,
        )
        return self._sqrt_determinant(force_matrix)


    ## Old Cost (Mostly Incorrect)
    # def directional_force_cost(self, data):
    #     """Equation (15): normalized Frobenius directional distance."""
    #     velocity_map = self.kinematics.paper_object_velocity_map(data)
    #     capability = velocity_map @ velocity_map.T
    #     capability_trace = np.trace(capability)
    #     desired_trace = np.trace(self.desired_force_matrix)
    #     if capability_trace <= 0.0 or desired_trace <= 0.0:
    #         return float("inf")
    #     difference = (
    #         capability / capability_trace
    #         - self.desired_force_matrix / desired_trace
    #     )
    #     return float(np.linalg.norm(difference, ord="fro"))
    
    ## Fixed Cost (Mostly Correct)
    def directional_force_cost(self, data):
        """Equation (15): normalized Frobenius directional force-capability distance.

        This uses the same force-side matrix as Eq. (14), namely
        (A A.T)^dagger. Therefore the objective is minimized in
        optimization_velocity() for DIRECTIONAL_FORCE.
        """
        velocity_map = self.kinematics.paper_object_velocity_map(data)

        # Force-capability matrix, consistent with Eq. (14).
        force_capability = np.linalg.pinv(
            velocity_map @ velocity_map.T,
            rcond=self.kinematics.pinv_rcond,
        )

        capability_trace = np.trace(force_capability)
        desired_trace = np.trace(self.desired_force_matrix)

        if capability_trace <= 0.0 or desired_trace <= 0.0:
            return float("inf")

        difference = (
            force_capability / capability_trace
            - self.desired_force_matrix / desired_trace
        )

        return float(np.linalg.norm(difference, ord="fro"))

    def _inter_arm_clearances(self, data):
        """Return every left–right sphere surface clearance."""
        if self.collision_version is CollisionModelVersion.VERSION1:
            left_positions = data.xpos[self.left_collision_body_ids]
            right_positions = data.xpos[self.right_collision_body_ids]
            left_radii = np.full(
                left_positions.shape[0],
                self.collision_sphere_radius,
            )
            right_radii = np.full(
                right_positions.shape[0],
                self.collision_sphere_radius,
            )
        else:
            left_positions = self._sphere_world_positions(
                data,
                self.left_detailed_body_ids,
                self.left_detailed_centers,
            )
            right_positions = self._sphere_world_positions(
                data,
                self.right_detailed_body_ids,
                self.right_detailed_centers,
            )
            left_radii = self.left_detailed_radii
            right_radii = self.right_detailed_radii

        center_distances = np.linalg.norm(
            left_positions[:, np.newaxis, :]
            - right_positions[np.newaxis, :, :],
            axis=2,
        )
        combined_radius = (
            left_radii[:, np.newaxis] + right_radii[np.newaxis, :]
        )
        return center_distances - combined_radius

    def detailed_collision_spheres(self, data):
        """Return v2 world spheres as ``(left, right)`` arrays of xyz-radius."""
        if self.collision_version is not CollisionModelVersion.VERSION2:
            return np.empty((0, 4)), np.empty((0, 4))
        left_positions = self._sphere_world_positions(
            data,
            self.left_detailed_body_ids,
            self.left_detailed_centers,
        )
        right_positions = self._sphere_world_positions(
            data,
            self.right_detailed_body_ids,
            self.right_detailed_centers,
        )
        return (
            np.column_stack((left_positions, self.left_detailed_radii)),
            np.column_stack((right_positions, self.right_detailed_radii)),
        )

    def minimum_inter_arm_clearance(self, data):
        """Return the closest coarse-sphere inter-arm clearance in metres."""
        return float(np.min(self._inter_arm_clearances(data)))

    def inter_arm_collision_cost(self, data):
        """Return a distance-weighted inter-arm proximity cost.

        This is a soft optimization bias around body spheres, not a
        hard collision constraint or a collision-proof motion planner.

        Version 1 retains its original squared hinge for reproducibility.
        Version 2 uses a squared softplus potential.  Its influence decays
        exponentially outside the requested clearance, grows smoothly as the
        arms approach, and keeps growing through sphere overlap.
        """
        clearances = self._inter_arm_clearances(data)
        signed_violations = self.collision_safety_margin - clearances
        if self.collision_version is CollisionModelVersion.VERSION1:
            violations = np.maximum(0.0, signed_violations)
        else:
            scale = self.collision_proximity_scale
            # scale*softplus(x/scale) is a smooth approximation of max(0, x).
            # np.logaddexp keeps the evaluation stable for deep overlap.
            violations = scale * np.logaddexp(
                0.0,
                signed_violations / scale,
            )
        return float(np.sum(violations**2))

    def _sqrt_determinant(self, matrix):
        sign, log_determinant = np.linalg.slogdet(matrix)
        if sign <= 0:
            return float(np.sqrt(self.determinant_floor))
        value = np.exp(0.5 * log_determinant)
        return float(max(value, np.sqrt(self.determinant_floor)))

    def value(self, data, objective=None):
        selected = ManipulabilityObjective(objective or self.objective)
        if selected is ManipulabilityObjective.VELOCITY:
            value = self.velocity_manipulability(data)
        elif selected is ManipulabilityObjective.FORCE:
            value = self.force_manipulability(data)
        else:
            # optimization_velocity() reverses this gradient to minimize the
            # directional distance. Adding collision cost here therefore
            # minimizes both directional error and collision violations.
            value = self.directional_force_cost(data)
            if self.enable_collision_penalty:
                value += (
                    self.collision_weight
                    * self.inter_arm_collision_cost(data)
                )
            return value

        if self.enable_collision_penalty and selected in (
            ManipulabilityObjective.VELOCITY,
            ManipulabilityObjective.FORCE,
        ):
            value -= self.collision_weight * self.inter_arm_collision_cost(data)
        return value

    def gradient(self, data, objective=None):
        """Return the central-difference derivative with respect to 14 joints."""
        selected = ManipulabilityObjective(objective or self.objective)
        step = self.finite_difference_step
        gradient = np.zeros(self.arm_qpos_indices.size)
        original_qpos = data.qpos.copy()

        try:
            for column, qpos_index in enumerate(self.arm_qpos_indices):
                center = original_qpos[qpos_index]

                data.qpos[qpos_index] = center + step
                mujoco.mj_forward(self.model, data)
                value_plus = self.value(data, selected)

                data.qpos[qpos_index] = center - step
                mujoco.mj_forward(self.model, data)
                value_minus = self.value(data, selected)

                gradient[column] = (value_plus - value_minus) / (2.0 * step)
                data.qpos[qpos_index] = center
        finally:
            data.qpos[:] = original_qpos
            mujoco.mj_forward(self.model, data)

        return gradient

    def optimization_velocity(self, data):
        """Return the signed and speed-limited phi_dot_opt from Equation (4)."""
        objective_value = self.value(data)
        gradient = self.gradient(data)
        direction_sign = (
            -1.0
            if self.objective is ManipulabilityObjective.DIRECTIONAL_FORCE
            else 1.0
        )
        phi_dot_opt = direction_sign * self.gain * gradient

        peak_speed = np.max(np.abs(phi_dot_opt))
        if peak_speed > self.maximum_joint_speed:
            phi_dot_opt *= self.maximum_joint_speed / peak_speed

        return OptimizationResult(
            objective=self.objective,
            value=objective_value,
            gradient=gradient,
            phi_dot_opt=phi_dot_opt,
        )
