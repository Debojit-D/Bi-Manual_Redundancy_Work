"""MuJoCo viewer overlay for detailed inter-arm collision spheres."""

import mujoco
import numpy as np


def draw_detailed_collision_spheres(viewer, optimizer, data):
    """Draw left spheres in cyan and right spheres in magenta."""
    left_spheres, right_spheres = optimizer.detailed_collision_spheres(data)
    user_scene = viewer.user_scn
    user_scene.ngeom = 0
    colors = (
        (left_spheres, np.array([0.0, 1.0, 1.0, 0.35])),
        (right_spheres, np.array([1.0, 0.0, 1.0, 0.35])),
    )
    for spheres, color in colors:
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
