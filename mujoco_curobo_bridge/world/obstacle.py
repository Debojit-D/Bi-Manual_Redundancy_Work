"""
obstacle.py

Basic obstacle representation.

This class is intentionally independent of MuJoCo and cuRobo.
It simply stores obstacle information.

Later it can be converted into:

- MuJoCo Geoms
- cuRobo v0.8 Scene
- ROS Collision Objects
"""

from dataclasses import dataclass
from typing import Tuple


@dataclass
class Obstacle:

    name: str

    shape: str
    # "box"
    # "sphere"
    # "cylinder"
    # "mesh"

    position: Tuple[float, float, float]

    size: Tuple[float, ...]

    rgba: Tuple[float, float, float, float] = (
        1.0,
        0.0,
        0.0,
        0.35,
    )

    enabled: bool = True

    def move(self, x, y, z):

        self.position = (
            x,
            y,
            z,
        )

    def translate(self, dx, dy, dz):

        px, py, pz = self.position

        self.position = (
            px + dx,
            py + dy,
            pz + dz,
        )
