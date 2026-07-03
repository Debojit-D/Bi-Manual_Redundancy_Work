"""
demos/11_motion_plan.py
"""

if __package__:
    from demos import _bootstrap
else:
    import _bootstrap

from bridge.mujoco_loader import MujocoRobot
from bridge.curobo_loader import CuroboRobot
from bridge.state_sync import PandaState
from bridge.world_bridge import to_world_config
from bridge.motion_planner_bridge import MotionPlannerBridge

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
home_pose = cr.get_end_effector_pose(home_state)

home_pos = home_pose.position.squeeze().tolist()
home_quat = home_pose.quaternion.squeeze().tolist()

print("Home EE position   :", home_pos)
print("Home EE orientation:", home_quat)

bridge = MotionPlannerBridge(world_cfg)

# Reuse the CURRENT orientation, only move position - isolates
# whether this was an orientation problem or something else.
goal_xyz = (0.5, 0.0, TABLE_TOP_Z + 0.15)

print("Planning to goal pose:", goal_xyz, "with home orientation:", home_quat)
result = bridge.plan_to_pose(start_q, goal_xyz, goal_quat=tuple(home_quat))

print("Planning success:", bridge.succeeded(result))
print("Status:", bridge.status(result))

if bridge.succeeded(result):
    traj = result.get_interpolated_plan()
    print("Trajectory length (steps):", traj.position.shape[-2])
else:
    print("Planning failed - check goal reachability and for collisions with the world.")
