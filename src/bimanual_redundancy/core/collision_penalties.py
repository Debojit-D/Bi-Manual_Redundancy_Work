"""Collision penalties and safety shaping for the null-space optimizer.

None of this module is paper mathematics: the manuscript's manipulability
metrics (Equations 12-17) do not account for collisions ("No additional
collision term was required in the planar setup because the motion was
constrained and the links were short. In the spatial simulations, a standard
potential-based collision penalty was added to the null-space objective").
This module implements exactly that added-on penalty so it can be inspected,
tested, and reasoned about separately from ``core.objectives``.

``CollisionPenaltiesMixin`` is mixed into ``ManipulabilityOptimizer`` rather
than composed, so every existing public attribute and method
(``left_table_body_ids``, ``inter_arm_collision_cost(data)``, ...) keeps
working exactly as before for external callers and tests; only the source
file changed.
"""

import mujoco
import numpy as np
from enum import Enum


class CollisionModelVersion(str, Enum):
    """Selectable inter-arm collision approximations."""

    VERSION1 = "version1"
    VERSION2 = "version2"


class CollisionPenaltiesMixin:
    """Collision-sphere loading, clearance queries, and soft penalty costs.

    Expects the concrete class (``ManipulabilityOptimizer``) to set
    ``self.model``, ``self.collision_version``, and the various
    ``enable_*``/``*_weight``/``*_safety_margin``/``*_proximity_scale``
    attributes in its own ``__init__`` before these methods are called.
    """

    @staticmethod
    def _is_descendant(model, body_id, root_id):
        while body_id > 0:
            if body_id == root_id:
                return True
            body_id = int(model.body_parentid[body_id])
        return False

    def _load_detailed_collision_spheres(self, model_path):
        """Load fitted body-local spheres and map them onto this MuJoCo model."""
        collision_spec = self.collision_spec
        if collision_spec is None:
            raise ValueError(
                "a collision-enabled optimizer requires system_spec.collision"
            )
        model_path = model_path or collision_spec.sphere_resource
        if model_path is None:
            raise ValueError(
                "collision_sphere_model_path is required for collision version2"
            )
        sphere_model = mujoco.MjModel.from_xml_path(str(model_path))
        left_root = sphere_model.body(collision_spec.left_root_body).id
        right_root = sphere_model.body(collision_spec.right_root_body).id
        # Fixed bases cannot move away from one another, so exclude them from
        # the optimization cost and retain all articulated arm/hand spheres.
        excluded_bodies = set(collision_spec.excluded_bodies)
        detailed = {"left": [], "right": []}
        table = {"left": [], "right": []}

        for geom_id in range(sphere_model.ngeom):
            geom_name = sphere_model.geom(geom_id).name
            if not geom_name.startswith(collision_spec.sphere_geom_prefix):
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
                collision_spec.left_table_bodies
                if side == "left"
                else collision_spec.right_table_bodies
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
            ("left", self.collision_spec.left_table_bodies),
            ("right", self.collision_spec.right_table_bodies),
        ):
            entries = table[side]
            if not entries:
                raise ValueError(
                    f"No fitted {side} arm-table collision spheres found"
                )
            body_ids = np.array([entry[0] for entry in entries], dtype=int)
            body_names = {self.model.body(body_id).name for body_id in body_ids}
            gripper_bodies = set(
                self.collision_spec.left_gripper_bodies
                + self.collision_spec.right_gripper_bodies
            )
            invalid_names = {
                name for name in body_names
                if name not in allowed_bodies or name in gripper_bodies
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
            configured_grippers = set(
                self.collision_spec.left_gripper_bodies
                if side == "left"
                else self.collision_spec.right_gripper_bodies
            )
            configured_terminals = set(
                self.collision_spec.left_terminal_bodies
                if side == "left"
                else self.collision_spec.right_terminal_bodies
            )
            gripper_bodies = np.array(
                [name in configured_grippers for name in body_names]
            )
            terminal_links = np.array(
                [name in configured_terminals for name in body_names]
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

        collision_spec = self.collision_spec
        if collision_spec is None:
            raise ValueError("table collision requires system_spec.collision")
        for geom_name in collision_spec.table_geom_names:
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
            collision_spec.table_root_body or "",
        )
        if table_root_id < 0:
            raise ValueError(
                "Could not auto-detect the table top collision geom: "
                f"body {collision_spec.table_root_body!r} is absent"
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
                collision_spec.table_top_half_size,
                rtol=0.0,
                atol=1e-4,
            ):
                candidates.append(geom_id)
        if len(candidates) != 1:
            raise ValueError(
                "Could not uniquely auto-detect the table top collision box "
                "with half-size "
                f"{list(collision_spec.table_top_half_size or ())}; found "
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

    def _inter_arm_clearances(self, data):
        """Return every left-right sphere surface clearance."""
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
        clearances = self._inter_arm_clearances(data)
        if not clearances.size:
            return float("inf")
        return float(np.min(clearances))

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
