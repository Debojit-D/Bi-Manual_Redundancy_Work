import mujoco

from world.obstacle import Obstacle


class WorldRenderer:

    def __init__(self, viewer):

        self.viewer = viewer

    def clear(self):

        self.viewer.user_scn.ngeom = 0

    def draw_obstacle(self, obstacle: Obstacle):

        if not obstacle.enabled:
            return

        scene = self.viewer.user_scn

        geom_id = scene.ngeom

        if geom_id >= scene.maxgeom:
            return

        geom = scene.geoms[geom_id]

        if obstacle.shape == "box":

            mujoco.mjv_initGeom(
                geom,
                type=mujoco.mjtGeom.mjGEOM_BOX,
                size=obstacle.size,
                pos=obstacle.position,
                mat=[1,0,0,
                     0,1,0,
                     0,0,1],
                rgba=obstacle.rgba,
            )

        elif obstacle.shape == "sphere":

            mujoco.mjv_initGeom(
                geom,
                type=mujoco.mjtGeom.mjGEOM_SPHERE,
                size=[obstacle.size[0],0,0],
                pos=obstacle.position,
                mat=[1,0,0,
                     0,1,0,
                     0,0,1],
                rgba=obstacle.rgba,
            )

        elif obstacle.shape == "cylinder":

            mujoco.mjv_initGeom(
                geom,
                type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                size=obstacle.size,
                pos=obstacle.position,
                mat=[1,0,0,
                     0,1,0,
                     0,0,1],
                rgba=obstacle.rgba,
            )

        scene.ngeom += 1

    def render(self, world, clear=True):

        if clear:
            self.clear()

        for obstacle in world.all():

            self.draw_obstacle(obstacle)