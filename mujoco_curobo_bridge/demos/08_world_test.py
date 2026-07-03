if __package__:
    from demos import _bootstrap
else:
    import _bootstrap

from world.world_manager import WorldManager
from world.obstacle import Obstacle


world = WorldManager()

world.add(
    Obstacle(
        name="cube",
        shape="box",
        position=(0.45, 0.0, 0.35),
        size=(0.05, 0.05, 0.05),
    )
)

world.add(
    Obstacle(
        name="sphere",
        shape="sphere",
        position=(0.25, 0.30, 0.45),
        size=(0.04,),
    )
)

world.print_summary()
