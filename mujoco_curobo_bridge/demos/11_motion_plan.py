"""
demos/11_motion_plan.py
"""

from bridge.mujoco_loader import MujocoRobot
from bridge.curobo_loader import CuroboRobot
from bridge.state_sync import PandaState
from bridge.world_bridge import to_world_config
from bridge.motion_gen_bridge import MotionGenBridge

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
    )
)

world_cfg = to_world_config(world)

print("Loading MuJoCo...")
mj = MujocoRobot()
mj.reset_home()
print("MuJoCo loaded.")

start_q = PandaState.get_arm_qpos(mj.get_qpos())
print("Start joint configuration:", start_q)

# Get a KNOWN-REACHABLE end-effector orientation by running FK on
# the current (valid) joint state - rather than guessing a quaternion.
cr = CuroboRobot()
home_state = cr.fk(start_q)

home_pos = home_state.ee_position.squeeze().tolist()
home_quat = home_state.ee_quaternion.squeeze().tolist()

print("Home EE position   :", home_pos)
print("Home EE orientation:", home_quat)

bridge = MotionGenBridge(world_cfg)

# Reuse the CURRENT orientation, only move position - isolates
# whether this was an orientation problem or something else.
goal_xyz = (0.5, 0.0, TABLE_TOP_Z + 0.15)

print("Planning to goal pose:", goal_xyz, "with home orientation:", home_quat)
result = bridge.plan_to_pose(start_q, goal_xyz, goal_quat=tuple(home_quat))

print("Planning success:", bool(result.success))
print("Status:", result.status)

if bool(result.success):
    traj = result.get_interpolated_plan()
    print("Trajectory length (steps):", len(traj.position))
else:
    print("Planning failed - check goal reachability and for collisions with the world.")