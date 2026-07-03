import time

if __package__:
    from demos import _bootstrap
else:
    import _bootstrap

from bridge.mujoco_loader import MujocoRobot
from bridge.curobo_loader import CuroboRobot
from bridge.state_sync import PandaState
from bridge.sphere_bridge import SphereBridge
from viewer.sphere_renderer import SphereRenderer


def main():

    print("=" * 70)
    print("Launching MuJoCo...")
    print("=" * 70)

    mj = MujocoRobot()
    mj.reset_home()
    viewer = mj.launch()

    print("✓ Viewer Started")

    cr = CuroboRobot()
    bridge = SphereBridge(cr)

    renderer = SphereRenderer(viewer)

    print("✓ Sphere Renderer Ready")

    while viewer.is_running():

        # Read robot configuration
        q = PandaState.get_arm_qpos(
            mj.get_qpos()
        )

        # Compute collision spheres
        spheres = bridge.get_spheres(q)

        # Draw them
        renderer.draw_spheres(spheres)

        # Update viewer
        renderer.sync()

        time.sleep(0.01)


if __name__ == "__main__":
    main()
