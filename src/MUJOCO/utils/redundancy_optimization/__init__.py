"""Deprecated shim — import from ``bimanual_redundancy.core`` and
``bimanual_redundancy.simulation`` instead.
"""

import warnings

warnings.warn(
    "MUJOCO.utils.redundancy_optimization is deprecated; import from "
    "bimanual_redundancy.core and bimanual_redundancy.simulation instead.",
    DeprecationWarning,
    stacklevel=2,
)

from bimanual_redundancy.core import (
    CapabilityMatrixKind,
    CollisionModelVersion,
    DirectionalDistanceCase,
    DirectionalDistanceOptimizationResult,
    DirectionalDistancePermutationOptimizer,
    DistanceDirection,
    Equation8Controller,
    Equation8Diagnostics,
    ManipulabilityObjective,
    ManipulabilityOptimizer,
    OptimizationResult,
    compute_nullspace_scale,
)
from bimanual_redundancy.simulation import (
    draw_detailed_collision_spheres,
    draw_table_collision_spheres,
)

__all__ = [
    "CapabilityMatrixKind",
    "CollisionModelVersion",
    "DirectionalDistanceCase",
    "DirectionalDistanceOptimizationResult",
    "DirectionalDistancePermutationOptimizer",
    "DistanceDirection",
    "Equation8Controller",
    "Equation8Diagnostics",
    "ManipulabilityObjective",
    "ManipulabilityOptimizer",
    "OptimizationResult",
    "compute_nullspace_scale",
    "draw_detailed_collision_spheres",
    "draw_table_collision_spheres",
]
