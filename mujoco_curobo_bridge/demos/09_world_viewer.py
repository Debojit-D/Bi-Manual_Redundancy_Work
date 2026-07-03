import time

if __package__:
    from demos import _bootstrap
else:
    import _bootstrap

from bridge.mujoco_loader import MujocoRobot

from viewer.world_renderer import WorldRenderer

from world.world_manager import WorldManager
from world.obstacle import Obstacle


robot = MujocoRobot()
robot.reset_home()

viewer = robot.launch()

renderer = WorldRenderer(viewer)

world = WorldManager()

# Table top surface height, from scene.xml:
#   table body pos.z (0.35) + table_top half-thickness (0.025)
TABLE_TOP_Z = 0.375

world.add(
    Obstacle(
        name="cube",
        shape="box",
        position=(0.45, 0.00, TABLE_TOP_Z + 0.05),
        size=(0.05,0.05,0.05),
        rgba=(1,0,0,0.4),
    )
)

world.add(
    Obstacle(
        name="sphere",
        shape="sphere",
        position=(0.35,-0.25, TABLE_TOP_Z + 0.04),
        size=(0.04,),
        rgba=(0,1,0,0.5),
    )
)

world.add(
    Obstacle(
        name="cylinder",
        shape="cylinder",
        position=(0.55,0.25, TABLE_TOP_Z + 0.10),
        size=(0.03,0.10,0.0),
        rgba=(0,0,1,0.5),
    )
)


while viewer.is_running():

    renderer.render(world)

    robot.step()

    viewer.sync()

    time.sleep(0.01)
