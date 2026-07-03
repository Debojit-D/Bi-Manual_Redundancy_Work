if __package__:
    from demos import _bootstrap
else:
    import _bootstrap

from bridge.mujoco_loader import MujocoRobot
from bridge.curobo_loader import CuroboRobot


print()

print("=" * 70)
print("Loading MuJoCo...")
print("=" * 70)

mj = MujocoRobot()
mj.reset_home()

print("✓ MuJoCo Loaded")

print()

print("=" * 70)
print("Loading cuRobo...")
print("=" * 70)

cr = CuroboRobot()

print("✓ cuRobo Loaded")

print()

print("=" * 70)
print("Current MuJoCo qpos")
print("=" * 70)

print(mj.get_qpos())

print()

print("=" * 70)
print("cuRobo Forward Kinematics")
print("=" * 70)

q = mj.get_qpos()[:7]
state = cr.fk(q)
ee_pose = cr.get_end_effector_pose(state)

print("EE position:", ee_pose.position)
print("EE quaternion (wxyz):", ee_pose.quaternion)
