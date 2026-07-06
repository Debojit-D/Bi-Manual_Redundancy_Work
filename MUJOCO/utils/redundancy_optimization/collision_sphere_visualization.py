"""MuJoCo viewer overlay for detailed inter-arm collision spheres."""

import mujoco
import numpy as np


def _draw_sphere_sets(viewer, sphere_sets, *, reset_scene):
    user_scene = viewer.user_scn
    if reset_scene:
        user_scene.ngeom = 0
    for spheres, color in sphere_sets:
        for x, y, z, radius in spheres:
            if user_scene.ngeom >= user_scene.maxgeom:
                return
            mujoco.mjv_initGeom(
                user_scene.geoms[user_scene.ngeom],
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=np.array([radius, 0.0, 0.0]),
                pos=np.array([x, y, z]),
                mat=np.eye(3).ravel(),
                rgba=color,
            )
            user_scene.ngeom += 1


def draw_detailed_collision_spheres(viewer, optimizer, data):
    """Draw all inter-arm fitted spheres in cyan and magenta."""
    left_spheres, right_spheres = optimizer.detailed_collision_spheres(data)
    _draw_sphere_sets(
        viewer,
        (
            (left_spheres, np.array([0.0, 1.0, 1.0, 0.35])),
            (right_spheres, np.array([1.0, 0.0, 1.0, 0.35])),
        ),
        reset_scene=True,
    )


def draw_table_collision_spheres(
    viewer,
    optimizer,
    data,
    *,
    reset_scene=True,
):
    """Draw the arm-only sphere set used against the table top box."""
    left_spheres, right_spheres = optimizer.table_collision_spheres(data)
    _draw_sphere_sets(
        viewer,
        (
            (left_spheres, np.array([1.0, 0.75, 0.0, 0.55])),
            (right_spheres, np.array([0.55, 1.0, 0.0, 0.55])),
        ),
        reset_scene=reset_scene,
    )
