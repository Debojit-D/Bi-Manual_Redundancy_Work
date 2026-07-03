"""
demos/13_multi_motion.py

Sequentially plans and plays back motions between several waypoints
above the table, so the arm keeps moving instead of stopping after
one reach. Cycles through the waypoint list indefinitely until you
close the viewer.
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

world.add(Obstacle(name="table", shape="box", position=(0.60, 0.00, 0.35),
                    size=(0.35, 0.45, 0.025), rgba=(0.63, 0.50, 0.35, 1.0)))
world.add(Obstacle(name="cube", shape="box", position=(0.45, 0.00, TABLE_TOP_Z + 0.05),
                    size=(0.05, 0.05, 0.05), rgba=(1, 0, 0, 0.4)))
world.add(Obstacle(name="sphere", shape="sphere", position=(0.35, -0.25, TABLE_TOP_Z + 0.04),
                    size=(0.04,), rgba=(0, 1, 0, 0.5)))
world.add(Obstacle(name="cylinder", shape="cylinder", position=(0.55, 0.25, TABLE_TOP_Z + 0.10),
                    size=(0.03, 0.10, 0.0), rgba=(0, 0, 1, 0.5)))

world_cfg = to_world_config(world)

print("Loading MuJoCo...")
mj = MujocoRobot()
mj.reset_home()
print("MuJoCo loaded.")

cr = CuroboRobot()
sphere_bridge = SphereBridge(cr)

start_q = PandaState.get_arm_qpos(mj.get_qpos())
home_state = cr.fk(start_q)
home_quat = cr.get_end_effector_pose(home_state).quaternion.squeeze().tolist()

motion_bridge = MotionPlannerBridge(world_cfg)

# Keep the GUI responsive by opening it after cuRobo's CUDA warmup.
viewer = mj.launch()
world_renderer = WorldRenderer(viewer)
sphere_renderer = SphereRenderer(viewer)

# Waypoints above the table, kept with generous clearance (>0.2m)
# from the cube/sphere/cylinder so IK isn't fighting collisions.
WAYPOINTS = [
    (0.65, -0.15, TABLE_TOP_Z + 0.20),
    (0.70,  0.20, TABLE_TOP_Z + 0.25),
    (0.55,  0.00, TABLE_TOP_Z + 0.30),
    (0.35, -0.30, TABLE_TOP_Z + 0.30),
]


def render_frame(current_q):
    world_renderer.render(world, clear=True)
    spheres = sphere_bridge.get_spheres(current_q)
    sphere_renderer.draw_spheres(spheres, clear=False)
    mj.forward()
    viewer.sync()


def play_trajectory(positions, dt=0.02):
    for q in positions:
        full_q = mj.get_qpos()
        full_q[:7] = q
        mj.set_qpos(full_q)
        render_frame(q)
        time.sleep(dt)


current_q = start_q
waypoint_index = 0

while viewer.is_running():

    goal_xyz = WAYPOINTS[waypoint_index % len(WAYPOINTS)]
    waypoint_index += 1

    print(f"Planning to waypoint {waypoint_index}: {goal_xyz}")

    result = motion_bridge.plan_to_pose(current_q, goal_xyz, goal_quat=tuple(home_quat))

    if not motion_bridge.succeeded(result):
        print("  -> planning failed:", motion_bridge.status(result), "- skipping this waypoint")
        continue

    positions = motion_bridge.get_arm_positions(result)

    print(f"  -> success, {len(positions)} steps")

    play_trajectory(positions)

    current_q = positions[-1]

    # brief pause at each waypoint so the motion is visually distinct
    for _ in range(25):
        if not viewer.is_running():
            break
        render_frame(current_q)
        time.sleep(0.02)

viewer.close()
