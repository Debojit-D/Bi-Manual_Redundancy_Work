"""
demos/12_planned_playback.py

Plans a collision-free trajectory, then opens the MuJoCo viewer and
steps the arm through it - while live-rendering both the world
obstacles AND cuRobo's collision spheres on the arm, synced to the
current joint configuration each frame.
"""

import time

if __package__:
    from demos import _bootstrap
else:
    import _bootstrap

from bridge.mujoco_loader import MujocoRobot
from bridge.curobo_loader import CuroboRobot
from bridge.state_sync import PandaState
from bridge.sphere_bridge import SphereBridge
from bridge.world_bridge import to_world_config
from bridge.motion_planner_bridge import MotionPlannerBridge

from viewer.world_renderer import WorldRenderer
from viewer.sphere_renderer import SphereRenderer

from world.world_manager import WorldManager
from world.obstacle import Obstacle


TABLE_TOP_Z = 0.375

world = WorldManager()

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

world_cfg = to_world_config(world)

print("Loading MuJoCo...")
mj = MujocoRobot()
mj.reset_home()
print("MuJoCo loaded.")

start_q = PandaState.get_arm_qpos(mj.get_qpos())

cr = CuroboRobot()
sphere_bridge = SphereBridge(cr)

home_state = cr.fk(start_q)
home_quat = cr.get_end_effector_pose(home_state).quaternion.squeeze().tolist()

motion_bridge = MotionPlannerBridge(world_cfg)

goal_xyz = (0.65, -0.15, TABLE_TOP_Z + 0.20)

print("Planning to goal pose:", goal_xyz)
result = motion_bridge.plan_to_pose(start_q, goal_xyz, goal_quat=tuple(home_quat))

print("Planning success:", motion_bridge.succeeded(result))

if not motion_bridge.succeeded(result):
    print("Planning failed - status:", motion_bridge.status(result))
    raise SystemExit(1)

positions = motion_bridge.get_arm_positions(result)

print(f"Playing back {len(positions)} trajectory steps...")

# Launch only after CUDA warmup and planning, so the viewer never appears
# frozen while cuRobo prepares its kernels.
viewer = mj.launch()
world_renderer = WorldRenderer(viewer)
sphere_renderer = SphereRenderer(viewer)

step = 0

while viewer.is_running():

    if step < len(positions):
        q = positions[step]

        full_q = mj.get_qpos()
        full_q[:7] = q
        mj.set_qpos(full_q)

        step += 1

    # Draw world obstacles first (this does the ngeom=0 reset)...
    world_renderer.render(world, clear=True)

    # ...then append collision spheres for the CURRENT arm pose,
    # without re-clearing (or the table/obstacles above would vanish).
    q_current = PandaState.get_arm_qpos(mj.get_qpos())
    spheres = sphere_bridge.get_spheres(q_current)
    sphere_renderer.draw_spheres(spheres, clear=False)

    # Playback is kinematic; do not integrate physics against actuator targets.
    mj.forward()

    viewer.sync()

    time.sleep(0.02)
    
viewer.close()
