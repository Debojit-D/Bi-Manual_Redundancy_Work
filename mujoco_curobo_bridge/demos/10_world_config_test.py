if __package__:
    from demos import _bootstrap
else:
    import _bootstrap

from world.world_manager import WorldManager
from world.obstacle import Obstacle
from bridge.world_bridge import to_world_config


world = WorldManager()

TABLE_TOP_Z = 0.375

# The table itself - it lives in scene.xml, not in WorldManager, so
# cuRobo has no knowledge of it unless we add it here explicitly.
# Table body pos=(0.60, 0, 0.35), tabletop half-size=(0.35, 0.45, 0.025).
world.add(
    Obstacle(
        name="table",
        shape="box",
        position=(0.60, 0.00, 0.35),
        size=(0.35, 0.45, 0.025),
        rgba=(0.63, 0.50, 0.35, 1.0),
    )
)

world.add(
    Obstacle(
        name="cube",
        shape="box",
        position=(0.45, 0.00, TABLE_TOP_Z + 0.05),
        size=(0.05, 0.05, 0.05),
        rgba=(1, 0, 0, 0.4),
    )
)

world.add(
    Obstacle(
        name="sphere",
        shape="sphere",
        position=(0.35, -0.25, TABLE_TOP_Z + 0.04),
        size=(0.04,),
        rgba=(0, 1, 0, 0.5),
    )
)

world.add(
    Obstacle(
        name="cylinder",
        shape="cylinder",
        position=(0.55, 0.25, TABLE_TOP_Z + 0.10),
        size=(0.03, 0.10, 0.0),
        rgba=(0, 0, 1, 0.5),
    )
)

world.print_summary()

world_cfg = to_world_config(world)

print()
print("=" * 70)
print("cuRobo v0.8 collision Scene")
print("=" * 70)
print(f"Cuboids   : {len(world_cfg.cuboid)}")
print(f"Meshes    : {len(world_cfg.mesh)}")

for c in world_cfg.cuboid:
    print(f"  cuboid   {c.name:10s} dims={c.dims}  pose={c.pose}")
for mesh in world_cfg.mesh:
    print(f"  mesh     {mesh.name:10s} pose={mesh.pose}")
