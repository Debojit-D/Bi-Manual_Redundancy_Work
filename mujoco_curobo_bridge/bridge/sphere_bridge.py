"""
sphere_bridge.py

Converts cuRobo FK output into a simple list of collision spheres.

Output format:
[
    [x, y, z, radius],
    ...
]
"""

import numpy as np
import torch


class SphereBridge:
    def __init__(self, curobo_robot):
        """
        Parameters
        ----------
        curobo_robot : CuroboRobot
            Instance of bridge.curobo_loader.CuroboRobot
        """
        self.robot = curobo_robot

    def get_spheres(self, q):
        """
        Compute robot collision spheres.

        Parameters
        ----------
        q : numpy.ndarray
            7 joint values.

        Returns
        -------
        numpy.ndarray
            Shape = (N,4)

            columns:
                x
                y
                z
                radius
        """

        # Run Forward Kinematics
        state = self.robot.fk(q)

        spheres = state.robot_spheres

        if spheres is None:
            raise RuntimeError("cuRobo did not return robot collision spheres")

        if isinstance(spheres, torch.Tensor):
            spheres = spheres.detach().cpu().numpy()

        # v0.8 preserves batch and horizon dimensions: [B, H, N, 4].
        spheres = spheres.reshape(-1, spheres.shape[-2], 4)[0]

        # Remove disabled spheres
        #
        # cuRobo marks inactive spheres
        # with a negative radius.
        #
        spheres = spheres[spheres[:, 3] > 0.0]

        return spheres

    def sphere_count(self, q):
        """
        Returns number of active collision spheres.
        """
        return len(self.get_spheres(q))

    def print_summary(self, q):
        """
        Print all sphere positions.
        Useful for debugging.
        """

        spheres = self.get_spheres(q)

        print()
        print("=" * 70)
        print("Collision Spheres")
        print("=" * 70)

        print(f"Total Active Spheres : {len(spheres)}")
        print()

        for i, sphere in enumerate(spheres):

            x, y, z, r = sphere

            print(
                f"{i:02d} | "
                f"x={x: .4f}  "
                f"y={y: .4f}  "
                f"z={z: .4f}  "
                f"r={r: .4f}"
            )

        return spheres
