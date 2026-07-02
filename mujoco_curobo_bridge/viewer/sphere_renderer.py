"""
sphere_renderer.py

Draw collision spheres inside MuJoCo's passive viewer.
"""

import mujoco
import numpy as np


class SphereRenderer:

    def __init__(self, viewer):

        self.viewer = viewer
        self.scene = viewer.user_scn

    def clear(self):
        """
        Remove all previously drawn spheres.
        """
        self.scene.ngeom = 0

    def draw_spheres(
        self,
        spheres,
        rgba=(0.0, 1.0, 1.0, 0.45),
        clear=True,
    ):
        if clear:
            self.clear()
        """
        Parameters
        ----------
        spheres : ndarray (N,4)

            columns:
                x
                y
                z
                radius
        """

        max_geom = self.scene.maxgeom

        for i, sphere in enumerate(spheres):

            if i >= max_geom:
                break

            x, y, z, radius = sphere

            geom = self.scene.geoms[self.scene.ngeom]

            mujoco.mjv_initGeom(
                geom,
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=np.array([radius, 0.0, 0.0]),
                pos=np.array([x, y, z]),
                mat=np.eye(3).flatten(),
                rgba=np.array(rgba),
            )

            self.scene.ngeom += 1

    def sync(self):
        """
        Push geometry to viewer.
        """
        self.viewer.sync()