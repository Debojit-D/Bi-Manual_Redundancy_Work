"""
world_bridge.py

Converts world.world_manager.WorldManager obstacles into a
curobo.scene.Scene, so cuRobo's collision checker and
motion planner become aware of the same obstacles being rendered
in the MuJoCo viewer.

Unit conventions (this is where bugs like to hide):

- MuJoCo box/cylinder geom "size" is HALF-extents.
  cuRobo Cuboid "dims" is FULL extents, and Cylinder "height" is
  FULL height. Box/cylinder values are doubled on the way in.
- MuJoCo sphere "size[0]" and cuRobo Sphere "radius" are both a
  plain radius - no conversion needed.
- Obstacle carries no orientation, so every obstacle is given an
  identity quaternion [1, 0, 0, 0] (qw, qx, qy, qz) - cuRobo poses
  are [x, y, z, qw, qx, qy, qz].
"""

from curobo.scene import Scene, Cuboid, Sphere, Cylinder

from world.world_manager import WorldManager
from world.obstacle import Obstacle


IDENTITY_QUAT = [1.0, 0.0, 0.0, 0.0]


def _pose(obstacle: Obstacle):
    x, y, z = obstacle.position
    return [x, y, z, *IDENTITY_QUAT]


def _to_cuboid(obstacle: Obstacle) -> Cuboid:
    sx, sy, sz = obstacle.size
    return Cuboid(
        name=obstacle.name,
        pose=_pose(obstacle),
        dims=[sx * 2.0, sy * 2.0, sz * 2.0],
        color=list(obstacle.rgba),
    )


def _to_sphere(obstacle: Obstacle) -> Sphere:
    (radius,) = obstacle.size
    return Sphere(
        name=obstacle.name,
        pose=_pose(obstacle),
        radius=radius,
        color=list(obstacle.rgba),
    )


def _to_cylinder(obstacle: Obstacle) -> Cylinder:
    radius, half_height, _ = obstacle.size
    return Cylinder(
        name=obstacle.name,
        pose=_pose(obstacle),
        radius=radius,
        height=half_height * 2.0,
        color=list(obstacle.rgba),
    )


_CONVERTERS = {
    "box": _to_cuboid,
    "sphere": _to_sphere,
    "cylinder": _to_cylinder,
}


def to_world_config(world: WorldManager) -> Scene:
    """
    Build a cuRobo v0.8 Scene from every enabled obstacle in a
    WorldManager.

    "mesh" obstacles are skipped for now - WorldRenderer doesn't
    draw them either yet, and cuRobo's Mesh type needs a file_path,
    which Obstacle doesn't carry.
    """

    cuboids = []
    spheres = []
    cylinders = []

    skipped = []

    for obstacle in world.all():

        if not obstacle.enabled:
            continue

        converter = _CONVERTERS.get(obstacle.shape)

        if converter is None:
            skipped.append(obstacle.name)
            continue

        result = converter(obstacle)

        if obstacle.shape == "box":
            cuboids.append(result)
        elif obstacle.shape == "sphere":
            spheres.append(result)
        elif obstacle.shape == "cylinder":
            cylinders.append(result)

    if skipped:
        print(f"[world_bridge] Skipped unsupported obstacle shapes: {skipped}")

    scene = Scene(
        cuboid=cuboids,
        sphere=spheres,
        cylinder=cylinders,
    )

    # v0.8 collision kernels consume cuboids, meshes, and voxel grids. Convert
    # spheres and cylinders to in-memory meshes so they are actually checked.
    return Scene.create_collision_support_world(scene, process=False)
