from bridge.mujoco_loader import MujocoRobot
from bridge.curobo_loader import CuroboRobot


print()

print("=" * 70)
print("Loading MuJoCo...")
print("=" * 70)

mj = MujocoRobot()

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

print(state)