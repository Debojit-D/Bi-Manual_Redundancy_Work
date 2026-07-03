if __package__:
    from demos import _bootstrap
else:
    import _bootstrap

from bridge.mujoco_loader import MujocoRobot
from bridge.curobo_loader import CuroboRobot
from bridge.state_sync import PandaState

import torch


def main():

    print("=" * 70)
    print("Loading MuJoCo...")
    print("=" * 70)

    mj = MujocoRobot()
    mj.reset_home()

    print("✓ MuJoCo Loaded\n")

    print("=" * 70)
    print("Loading cuRobo...")
    print("=" * 70)

    cr = CuroboRobot()

    print("✓ cuRobo Loaded\n")

    # ------------------------------------------------------------------
    # Read current MuJoCo joint configuration
    # ------------------------------------------------------------------

    q = PandaState.get_arm_qpos(
        mj.get_qpos()
    )

    print("=" * 70)
    print("MuJoCo Joint Configuration")
    print("=" * 70)

    print(q)

    # ------------------------------------------------------------------
    # Forward Kinematics
    # ------------------------------------------------------------------

    state = cr.fk(q)

    print("\n")
    print("=" * 70)
    print("Returned State Type")
    print("=" * 70)

    print(type(state))

    print("\n")
    print("=" * 70)
    print("Available Attributes")
    print("=" * 70)

    attrs = [a for a in dir(state) if not a.startswith("_")]

    for attr in attrs:
        print(attr)

    print("\n")
    print("=" * 70)
    print("State Object")
    print("=" * 70)

    print(state)

    print("\n")
    print("=" * 70)
    print("Forward Kinematics Successful")
    print("=" * 70)


if __name__ == "__main__":
    main()
