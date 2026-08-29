"""MuJoCo-specific scene construction, collision visualization, cameras,
and recording (the simulation backend for the mathematical ``core``).
"""

from .scene import DualFrankaMuJoCoScene
from .collisions import draw_detailed_collision_spheres, draw_table_collision_spheres

__all__ = [
    "DualFrankaMuJoCoScene",
    "draw_detailed_collision_spheres",
    "draw_table_collision_spheres",
]
