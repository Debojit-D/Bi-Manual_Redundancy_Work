"""Mathematical core: cooperative kinematics, the Equation (8) controller,
and task-specific manipulability objectives.
"""

from .cooperative_kinematics import CooperativeManipulationKinematics
from .controller import (
    Equation8Controller,
    Equation8Diagnostics,
    compute_nullspace_scale,
)
from .gradients import central_difference_gradient
from .objectives import (
    CollisionModelVersion,
    ManipulabilityObjective,
    ManipulabilityOptimizer,
    OptimizationResult,
)
from .directional_distance_optimization import (
    CapabilityMatrixKind,
    DirectionalDistanceCase,
    DirectionalDistanceOptimizationResult,
    DirectionalDistancePermutationOptimizer,
    DistanceDirection,
)

__all__ = [
    "CooperativeManipulationKinematics",
    "Equation8Controller",
    "Equation8Diagnostics",
    "compute_nullspace_scale",
    "central_difference_gradient",
    "CollisionModelVersion",
    "ManipulabilityObjective",
    "ManipulabilityOptimizer",
    "OptimizationResult",
    "CapabilityMatrixKind",
    "DirectionalDistanceCase",
    "DirectionalDistanceOptimizationResult",
    "DirectionalDistancePermutationOptimizer",
    "DistanceDirection",
]
