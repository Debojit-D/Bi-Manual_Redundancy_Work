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

LEFT_TABLE_COLLISION_BODIES = (
    "link1_l",
    "link2",
    "link3_l",
    "link4_l",
    "link5_l",
    "link6_l",
    "link7_l",
)

RIGHT_TABLE_COLLISION_BODIES = (
    "link1_r",
    "link2_r",
    "link3_r",
    "link4_r",
    "link5_r",
    "link6_r",
    "link7_r",
)

TABLE_TOP_HALF_SIZE = np.array([0.202, 0.134, 0.005])
KNOWN_TABLE_TOP_GEOM_NAMES = (
    "vention_table_top_collision",
    "table_top_collision",
)


class ManipulabilityObjective(str, Enum):
    """The supported redundancy objectives."""

    VELOCITY = "velocity"
    FORCE = "force"
    DIRECTIONAL_FORCE = "directional_force"
    DIRECTIONAL_FORCE_INDIRECT = "directional_force_indirect"


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
    """Evaluate paper objectives and produce null motion.

    The raw spatial map is ``A_raw = A``.  With characteristic length ``l``,
    ``S_l = diag(1, 1, 1, l, l, l)`` and ``A_scaled = S_l A_raw``.  The active
    controller objectives remain the original raw quantities. Scaled spatial
    capabilities are retained as diagnostics for CSV analysis only.

    Velocity and force manipulability are maximized. Direct directional-force
    distance compares the force-capability matrix with the desired wrench
    direction and is minimized. Indirect directional-force distance compares
    the velocity-capability matrix with that direction and is maximized.
    Gradients are evaluated with central joint-space finite differences.

    Unsuffixed public objective methods preserve the original raw behavior.
    Explicit ``*_raw`` and ``*_scaled`` methods support comparative reporting.
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
        enable_table_collision_penalty=False,
        table_collision_weight=20000.0,
        table_collision_safety_margin=0.015,
        table_collision_proximity_scale=0.003,
        table_collision_geom_name=None,
        enable_self_collision_penalty=False,
        self_collision_weight=20000.0,
        self_collision_safety_margin=0.01,
        self_collision_proximity_scale=0.003,
    ):
        self.kinematics = kinematics
        self.model = kinematics.model
        self.arm_qpos_indices = np.asarray(arm_qpos_indices, dtype=int)
        self.objective = ManipulabilityObjective(objective)
        self.gain = float(gain)
        self.finite_difference_step = float(finite_difference_step)
        self.maximum_joint_speed = float(maximum_joint_speed)
        self.characteristic_length = float(characteristic_length)
        if self.characteristic_length <= 0.0:
            raise ValueError("characteristic_length must be positive")
        self.determinant_floor = float(determinant_floor)
        self.enable_collision_penalty = bool(enable_collision_penalty)
        self.collision_weight = float(collision_weight)
        self.collision_safety_margin = float(collision_safety_margin)
        self.collision_proximity_scale = float(collision_proximity_scale)
        self.collision_sphere_radius = float(collision_sphere_radius)
        self.collision_version = CollisionModelVersion(collision_version)
        self.enable_table_collision_penalty = bool(
            enable_table_collision_penalty
        )
        self.table_collision_weight = float(table_collision_weight)
        self.table_collision_safety_margin = float(
            table_collision_safety_margin
        )
        self.table_collision_proximity_scale = float(
            table_collision_proximity_scale
        )
        self.table_collision_geom_name = table_collision_geom_name
        self.table_collision_geom_id = -1
        self.enable_self_collision_penalty = bool(
            enable_self_collision_penalty
        )
        self.self_collision_weight = float(self_collision_weight)
        self.self_collision_safety_margin = float(
            self_collision_safety_margin
        )
        self.self_collision_proximity_scale = float(
            self_collision_proximity_scale
        )
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
        self.left_table_body_ids = np.empty(0, dtype=int)
        self.left_table_centers = np.empty((0, 3))
        self.left_table_radii = np.empty(0)
        self.right_table_body_ids = np.empty(0, dtype=int)
        self.right_table_centers = np.empty((0, 3))
        self.right_table_radii = np.empty(0)
        self.left_self_collision_pairs = np.empty((0, 2), dtype=int)
        self.right_self_collision_pairs = np.empty((0, 2), dtype=int)
        if (
            self.collision_version is CollisionModelVersion.VERSION2
            or self.enable_table_collision_penalty
            or self.enable_self_collision_penalty
        ):
            self._load_detailed_collision_spheres(collision_sphere_model_path)
        if self.enable_table_collision_penalty:
            self.table_collision_geom_id = self._resolve_table_collision_geom(
                table_collision_geom_name
            )
        (
            self.desired_force_matrix_raw,
            self.desired_force_matrix_scaled,
        ) = self._make_direction_matrices(
            desired_wrench_direction
        )
        # Backward-compatible name used by the raw controller objective.
        self.desired_force_matrix = self.desired_force_matrix_raw

        if self.arm_qpos_indices.size != self.kinematics.arm_dofs.size:
            raise ValueError("Expected one qpos index for every controlled DoF")
        if self.finite_difference_step <= 0.0:
            raise ValueError("finite_difference_step must be positive")
        if self.collision_weight < 0.0:
            raise ValueError("collision_weight must be nonnegative")
        if self.collision_safety_margin < 0.0:
            raise ValueError("collision_safety_margin must be nonnegative")
        if self.collision_proximity_scale <= 0.0:
            raise ValueError("collision_proximity_scale must be positive")
        if self.collision_sphere_radius < 0.0:
            raise ValueError("collision_sphere_radius must be nonnegative")
        if self.table_collision_weight < 0.0:
            raise ValueError("table_collision_weight must be nonnegative")
        if self.table_collision_safety_margin < 0.0:
            raise ValueError(
                "table_collision_safety_margin must be nonnegative"
            )
        if self.table_collision_proximity_scale <= 0.0:
            raise ValueError(
                "table_collision_proximity_scale must be positive"
            )
        if self.self_collision_weight < 0.0:
            raise ValueError("self_collision_weight must be nonnegative")
        if self.self_collision_safety_margin < 0.0:
            raise ValueError(
                "self_collision_safety_margin must be nonnegative"
            )
        if self.self_collision_proximity_scale <= 0.0:
            raise ValueError(
                "self_collision_proximity_scale must be positive"
            )

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
        table = {"left": [], "right": []}

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
            entry = (
                target_body_id,
                sphere_model.geom_pos[geom_id].copy(),
                float(sphere_model.geom_size[geom_id, 0]),
            )
            detailed[side].append(entry)
            table_bodies = (
                LEFT_TABLE_COLLISION_BODIES
                if side == "left"
                else RIGHT_TABLE_COLLISION_BODIES
            )
            if body_name in table_bodies:
                table[side].append(entry)

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

        self._store_table_collision_spheres(table)
        self._store_self_collision_pairs()

    def _store_table_collision_spheres(self, table):
        """Store fitted spheres belonging only to non-grasping arm links."""
        for side, allowed_bodies in (
            ("left", LEFT_TABLE_COLLISION_BODIES),
            ("right", RIGHT_TABLE_COLLISION_BODIES),
        ):
            entries = table[side]
            if not entries:
                raise ValueError(
                    f"No fitted {side} arm-table collision spheres found"
                )
            body_ids = np.array([entry[0] for entry in entries], dtype=int)
            body_names = {self.model.body(body_id).name for body_id in body_ids}
            invalid_names = {
                name
                for name in body_names
                if name not in allowed_bodies
                or "finger" in name.lower()
                or name in {"hand_l", "hand_r"}
            }
            if invalid_names:
                raise ValueError(
                    "Invalid grasping bodies in arm-table sphere set: "
                    f"{sorted(invalid_names)}"
                )
            setattr(self, f"{side}_table_body_ids", body_ids)
            setattr(
                self,
                f"{side}_table_centers",
                np.array([entry[1] for entry in entries], dtype=float),
            )
            setattr(
                self,
                f"{side}_table_radii",
                np.array([entry[2] for entry in entries], dtype=float),
            )

    def _store_self_collision_pairs(self):
        """Cache valid full-model sphere pairs within each articulated arm."""
        for side in ("left", "right"):
            body_ids = getattr(self, f"{side}_detailed_body_ids")
            first, second = np.triu_indices(body_ids.size, k=1)
            first_bodies = body_ids[first]
            second_bodies = body_ids[second]
            distinct_bodies = first_bodies != second_bodies
            adjacent_bodies = (
                self.model.body_parentid[first_bodies] == second_bodies
            ) | (
                self.model.body_parentid[second_bodies] == first_bodies
            )
            body_names = np.array(
                [self.model.body(int(body_id)).name for body_id in body_ids]
            )
            gripper_bodies = np.array(
                [
                    "hand" in name.lower() or "finger" in name.lower()
                    for name in body_names
                ]
            )
            terminal_links = np.array(
                [name in {"link7_l", "link7_r"} for name in body_names]
            )
            internal_gripper_pair = (
                gripper_bodies[first] & gripper_bodies[second]
            )
            gripper_terminal_pair = (
                gripper_bodies[first] & terminal_links[second]
            ) | (
                terminal_links[first] & gripper_bodies[second]
            )
            valid_pairs = (
                distinct_bodies
                & ~adjacent_bodies
                & ~internal_gripper_pair
                & ~gripper_terminal_pair
            )
            pairs = np.column_stack(
                (first[valid_pairs], second[valid_pairs])
            )
            if self.enable_self_collision_penalty and not pairs.size:
                raise ValueError(
                    f"No non-adjacent {side} self-collision pairs found"
                )
            setattr(self, f"{side}_self_collision_pairs", pairs)

    def _resolve_table_collision_geom(self, requested_name):
        """Resolve the table top collision box by name or model geometry."""
        if requested_name is not None:
            geom_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                str(requested_name),
            )
            if geom_id < 0:
                raise ValueError(
                    f"Table collision geom {requested_name!r} was not found"
                )
            self._validate_table_box_geom(geom_id)
            return geom_id

        for geom_name in KNOWN_TABLE_TOP_GEOM_NAMES:
            geom_id = mujoco.mj_name2id(
                self.model,
                mujoco.mjtObj.mjOBJ_GEOM,
                geom_name,
            )
            if geom_id >= 0:
                self._validate_table_box_geom(geom_id)
                return geom_id

        table_root_id = mujoco.mj_name2id(
            self.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "vention_table",
        )
        if table_root_id < 0:
            raise ValueError(
                "Could not auto-detect the table top collision geom: "
                "body 'vention_table' is absent"
            )
        candidates = []
        for geom_id in range(self.model.ngeom):
            body_id = int(self.model.geom_bodyid[geom_id])
            if not self._is_descendant(self.model, body_id, table_root_id):
                continue
            if self.model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
                continue
            if np.allclose(
                self.model.geom_size[geom_id],
                TABLE_TOP_HALF_SIZE,
                rtol=0.0,
                atol=1e-4,
            ):
                candidates.append(geom_id)
        if len(candidates) != 1:
            raise ValueError(
                "Could not uniquely auto-detect the table top collision box "
                f"with half-size {TABLE_TOP_HALF_SIZE.tolist()}; found "
                f"{len(candidates)} candidates. Pass table_collision_geom_name."
            )
        return candidates[0]

    def _validate_table_box_geom(self, geom_id):
        if self.model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_BOX:
            raise ValueError("table_collision_geom_name must identify a box geom")

    @staticmethod
    def _sphere_world_positions(data, body_ids, local_centers):
        rotations = data.xmat[body_ids].reshape(-1, 3, 3)
        return data.xpos[body_ids] + np.einsum(
            "nij,nj->ni",
            rotations,
            local_centers,
        )

    def _make_direction_matrices(self, wrench_direction):
        """Return raw and dual-scaled diagonal wrench weighting matrices."""
        raw_weights = np.asarray(wrench_direction, dtype=float).copy()
        if raw_weights.shape != (6,):
            raise ValueError("desired_wrench_direction must contain six values")
        # Equation (15) defines F=diag(Fx,Fy,Fz,Mx,My,Mz).  Manipulability
        # ellipsoids are sign-symmetric, so load-axis weights are nonnegative.
        raw_weights = np.abs(raw_weights)
        if np.sum(raw_weights) <= 0.0:
            raise ValueError("desired_wrench_direction cannot be zero")
        # q_dot_scaled=S_l q_dot implies w_scaled=S_l^-T w.  Divide moment
        # entries exactly once by l while preserving the diagonal convention.
        scaled_weights = raw_weights.copy()
        scaled_weights[3:] /= self.characteristic_length
        return np.diag(raw_weights), np.diag(scaled_weights)

    def spatial_scaling_matrix(self):
        """Return ``S_l = diag(1, 1, 1, l, l, l)``."""
        return np.diag(
            [1.0, 1.0, 1.0]
            + [self.characteristic_length] * 3
        )

    def object_velocity_maps(self, data):
        """Return ``A_raw=A`` and ``A_scaled=S_l A_raw``."""
        raw_map = np.asarray(
            self.kinematics.paper_object_velocity_map(data),
            dtype=float,
        )
        scaled_map = self.spatial_scaling_matrix() @ raw_map
        return raw_map, scaled_map

    @staticmethod
    def _velocity_capabilities_from_maps(raw_map, scaled_map):
        return raw_map @ raw_map.T, scaled_map @ scaled_map.T

    def velocity_capability_matrices(self, data):
        """Return raw and scaled velocity capabilities ``A A.T``."""
        return self._velocity_capabilities_from_maps(
            *self.object_velocity_maps(data)
        )

    def _force_capabilities_from_velocity(self, raw_matrix, scaled_matrix):
        rcond = self.kinematics.pinv_rcond
        return (
            np.linalg.pinv(raw_matrix, rcond=rcond),
            np.linalg.pinv(scaled_matrix, rcond=rcond),
        )

    def force_capability_matrices(self, data):
        """Return ``Mf_raw=pinv(Mv_raw)`` and ``Mf_scaled=pinv(Mv_scaled)``."""
        return self._force_capabilities_from_velocity(
            *self.velocity_capability_matrices(data)
        )

    def spatial_capability_matrices(self, data):
        """Return ``A_raw, A_scaled, Mv_raw, Mv_scaled, Mf_raw, Mf_scaled``."""
        raw_map, scaled_map = self.object_velocity_maps(data)
        velocity_raw, velocity_scaled = (
            self._velocity_capabilities_from_maps(raw_map, scaled_map)
        )
        force_raw, force_scaled = self._force_capabilities_from_velocity(
            velocity_raw,
            velocity_scaled,
        )
        return (
            raw_map,
            scaled_map,
            velocity_raw,
            velocity_scaled,
            force_raw,
            force_scaled,
        )

    @staticmethod
    def velocity_map_diagnostics(velocity_map):
        """Return rank, extrema of singular values, and a safe condition number."""
        velocity_map = np.asarray(velocity_map, dtype=float)
        singular_values = np.linalg.svd(velocity_map, compute_uv=False)
        rank = int(np.linalg.matrix_rank(velocity_map))
        if singular_values.size == 0:
            return rank, 0.0, 0.0, float("inf")
        sigma_max = float(singular_values[0])
        sigma_min = float(singular_values[-1])
        tolerance = (
            np.finfo(float).eps
            * max(velocity_map.shape, default=0)
            * sigma_max
        )
        condition = (
            float("inf")
            if sigma_min <= tolerance
            else sigma_max / sigma_min
        )
        return rank, sigma_min, sigma_max, float(condition)

    def velocity_manipulability_raw(self, data):
        """Return the original diagnostic ``sqrt(det(Mv_raw))``."""
        raw_matrix, _ = self.velocity_capability_matrices(data)
        return self._sqrt_determinant(raw_matrix)

    def velocity_manipulability_scaled(self, data):
        """Return diagnostic ``sqrt(det(Mv_scaled))`` for CSV comparison."""
        _, scaled_matrix = self.velocity_capability_matrices(data)
        return self._sqrt_determinant(scaled_matrix)

    def velocity_manipulability(self, data):
        """Return raw velocity manipulability used by the optimizer."""
        return self.velocity_manipulability_raw(data)

    def force_manipulability_raw(self, data):
        """Return the original diagnostic ``sqrt(det(Mf_raw))``."""
        raw_matrix, _ = self.force_capability_matrices(data)
        return self._sqrt_determinant(raw_matrix)

    def force_manipulability_scaled(self, data):
        """Return diagnostic ``sqrt(det(Mf_scaled))`` for CSV comparison."""
        _, scaled_matrix = self.force_capability_matrices(data)
        return self._sqrt_determinant(scaled_matrix)

    def force_manipulability(self, data):
        """Return raw force manipulability used by the optimizer."""
        return self.force_manipulability_raw(data)

    @staticmethod
    def _normalized_frobenius_distance(capability, desired):
        capability_trace = float(np.trace(capability))
        desired_trace = float(np.trace(desired))
        if capability_trace <= 0.0 or desired_trace <= 0.0:
            return float("inf")
        difference = (
            capability / capability_trace
            - desired / desired_trace
        )
        return float(np.linalg.norm(difference, ord="fro"))

    def directional_force_cost_raw(self, data):
        """Return force-direction distance in original unscaled coordinates."""
        force_raw, _ = self.force_capability_matrices(data)
        return self._normalized_frobenius_distance(
            force_raw,
            self.desired_force_matrix_raw,
        )

    def directional_force_cost_scaled(self, data):
        """Return diagnostic force-direction distance in scaled coordinates."""
        _, force_scaled = self.force_capability_matrices(data)
        return self._normalized_frobenius_distance(
            force_scaled,
            self.desired_force_matrix_scaled,
        )

    def directional_force_cost(self, data):
        """Return raw directional-force distance minimized by the optimizer."""
        return self.directional_force_cost_raw(data)

    def directional_force_indirect_cost_raw(self, data):
        """Return raw velocity-capability distance maximized by the optimizer."""
        velocity_raw, _ = self.velocity_capability_matrices(data)
        return self._normalized_frobenius_distance(
            velocity_raw,
            self.desired_force_matrix_raw,
        )

    def directional_force_indirect_cost_scaled(self, data):
        """Return scaled velocity-capability directional distance."""
        _, velocity_scaled = self.velocity_capability_matrices(data)
        return self._normalized_frobenius_distance(
            velocity_scaled,
            self.desired_force_matrix_scaled,
        )

    def directional_force_indirect_cost(self, data):
        """Return raw indirect directional-force distance to maximize."""
        return self.directional_force_indirect_cost_raw(data)

    def paper_objective_values(self, data):
        """Return raw and scaled selected paper metrics before collision costs."""
        if self.objective is ManipulabilityObjective.VELOCITY:
            return (
                self.velocity_manipulability_raw(data),
                self.velocity_manipulability_scaled(data),
            )
        if self.objective is ManipulabilityObjective.FORCE:
            return (
                self.force_manipulability_raw(data),
                self.force_manipulability_scaled(data),
            )
        if (
            self.objective
            is ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT
        ):
            return (
                self.directional_force_indirect_cost_raw(data),
                self.directional_force_indirect_cost_scaled(data),
            )
        return (
            self.directional_force_cost_raw(data),
            self.directional_force_cost_scaled(data),
        )

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

    def table_collision_spheres(self, data):
        """Return arm-only table spheres as left/right xyz-radius arrays."""
        if not self.left_table_radii.size or not self.right_table_radii.size:
            return np.empty((0, 4)), np.empty((0, 4))
        left_positions = self._sphere_world_positions(
            data,
            self.left_table_body_ids,
            self.left_table_centers,
        )
        right_positions = self._sphere_world_positions(
            data,
            self.right_table_body_ids,
            self.right_table_centers,
        )
        return (
            np.column_stack((left_positions, self.left_table_radii)),
            np.column_stack((right_positions, self.right_table_radii)),
        )

    def _sphere_to_table_top_plane_clearances(
        self,
        data,
        sphere_positions,
        sphere_radii,
        table_geom_id,
    ):
        """Return sphere clearances from the oriented tabletop plane patch.

        The collision plane is the top face of the detected tabletop box. Its
        point and normal therefore follow the table's free-joint translation
        and rotation. The box's x/y half sizes bound the plane patch so arm
        spheres beside or below the table are not penalized as they would be
        by an infinite plane. Negative values mean sphere/plane intersection.
        """
        sphere_positions = np.asarray(sphere_positions, dtype=float)
        sphere_radii = np.asarray(sphere_radii, dtype=float)
        if sphere_positions.ndim != 2 or sphere_positions.shape[1] != 3:
            raise ValueError("sphere_positions must have shape (n, 3)")
        if sphere_radii.shape != (sphere_positions.shape[0],):
            raise ValueError("sphere_radii must have shape (n,)")
        box_center = data.geom_xpos[table_geom_id]
        box_rotation = data.geom_xmat[table_geom_id].reshape(3, 3)
        box_half_size = self.model.geom_size[table_geom_id]
        local_positions = (sphere_positions - box_center) @ box_rotation
        tangential_offset = np.maximum(
            np.abs(local_positions[:, :2]) - box_half_size[:2],
            0.0,
        )
        normal_offset = local_positions[:, 2] - box_half_size[2]
        center_distance = np.sqrt(
            np.sum(tangential_offset**2, axis=1) + normal_offset**2
        )
        return center_distance - sphere_radii

    def _arm_table_clearances(self, data):
        """Return all arm-sphere clearances from the tabletop plane patch."""
        if self.table_collision_geom_id < 0:
            try:
                self.table_collision_geom_id = (
                    self._resolve_table_collision_geom(
                        self.table_collision_geom_name
                    )
                )
            except ValueError:
                if self.enable_table_collision_penalty:
                    raise
                return np.empty(0)
        left_positions = self._sphere_world_positions(
            data,
            self.left_table_body_ids,
            self.left_table_centers,
        )
        right_positions = self._sphere_world_positions(
            data,
            self.right_table_body_ids,
            self.right_table_centers,
        )
        left_clearances = self._sphere_to_table_top_plane_clearances(
            data,
            left_positions,
            self.left_table_radii,
            self.table_collision_geom_id,
        )
        right_clearances = self._sphere_to_table_top_plane_clearances(
            data,
            right_positions,
            self.right_table_radii,
            self.table_collision_geom_id,
        )
        return np.concatenate((left_clearances, right_clearances))

    def _same_arm_clearances(self, data, side):
        """Return valid full-model sphere clearances on one arm."""
        body_ids = getattr(self, f"{side}_detailed_body_ids")
        centers = getattr(self, f"{side}_detailed_centers")
        radii = getattr(self, f"{side}_detailed_radii")
        pairs = getattr(self, f"{side}_self_collision_pairs")
        if not pairs.size:
            return np.empty(0)
        positions = self._sphere_world_positions(data, body_ids, centers)
        first, second = pairs.T
        center_distances = np.linalg.norm(
            positions[first] - positions[second],
            axis=1,
        )
        return center_distances - radii[first] - radii[second]

    def _self_collision_clearances(self, data):
        """Return all valid same-arm sphere clearances on both arms."""
        return np.concatenate(
            (
                self._same_arm_clearances(data, "left"),
                self._same_arm_clearances(data, "right"),
            )
        )

    def minimum_inter_arm_clearance(self, data):
        """Return the closest inter-arm sphere clearance in metres."""
        return float(np.min(self._inter_arm_clearances(data)))

    def minimum_arm_table_clearance(self, data):
        """Return the closest arm-sphere clearance to the tabletop plane."""
        clearances = self._arm_table_clearances(data)
        if not clearances.size:
            return float("inf")
        return float(np.min(clearances))

    def minimum_self_collision_clearance(self, data):
        """Return the closest valid same-arm sphere clearance."""
        clearances = self._self_collision_clearances(data)
        if not clearances.size:
            return float("inf")
        return float(np.min(clearances))

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

    def arm_table_collision_cost(self, data):
        """Return the smooth arm-sphere/table-plane proximity cost."""
        clearances = self._arm_table_clearances(data)
        if not clearances.size:
            return 0.0
        signed_violations = self.table_collision_safety_margin - clearances
        scale = self.table_collision_proximity_scale
        violations = scale * np.logaddexp(
            0.0,
            signed_violations / scale,
        )
        return float(np.sum(violations**2))

    def self_collision_cost(self, data):
        """Return smooth proximity cost for valid same-arm sphere pairs."""
        clearances = self._self_collision_clearances(data)
        if not clearances.size:
            return 0.0
        signed_violations = self.self_collision_safety_margin - clearances
        scale = self.self_collision_proximity_scale
        violations = scale * np.logaddexp(
            0.0,
            signed_violations / scale,
        )
        return float(np.sum(violations**2))

    def total_collision_cost(self, data):
        """Return the weighted sum of all enabled collision costs."""
        cost = 0.0
        if self.enable_collision_penalty:
            cost += self.collision_weight * self.inter_arm_collision_cost(data)
        if self.enable_table_collision_penalty:
            cost += (
                self.table_collision_weight
                * self.arm_table_collision_cost(data)
            )
        if self.enable_self_collision_penalty:
            cost += self.self_collision_weight * self.self_collision_cost(data)
        return float(cost)

    def _sqrt_determinant(self, matrix):
        sign, log_determinant = np.linalg.slogdet(matrix)
        if sign <= 0:
            return float(np.sqrt(self.determinant_floor))
        value = np.exp(0.5 * log_determinant)
        return float(max(value, np.sqrt(self.determinant_floor)))

    def value(self, data, objective=None):
        selected = ManipulabilityObjective(objective or self.objective)
        collision_cost = self.total_collision_cost(data)
        if selected is ManipulabilityObjective.VELOCITY:
            value = self.velocity_manipulability(data)
        elif selected is ManipulabilityObjective.FORCE:
            value = self.force_manipulability(data)
        elif selected is ManipulabilityObjective.DIRECTIONAL_FORCE_INDIRECT:
            value = self.directional_force_indirect_cost(data)
        else:
            # optimization_velocity() reverses this gradient to minimize the
            # directional distance. Adding collision cost here therefore
            # minimizes both directional error and collision violations.
            return self.directional_force_cost(data) + collision_cost

        return value - collision_cost

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
