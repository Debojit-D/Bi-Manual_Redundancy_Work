"""Equation 8 controller and task-specific manipulability objectives."""

from .equation_8_controller import Equation8Controller, Equation8Diagnostics
from .manipulability_optimization import (
    ManipulabilityObjective,
    ManipulabilityOptimizer,
    OptimizationResult,
)

__all__ = [
    "Equation8Controller",
    "Equation8Diagnostics",
    "ManipulabilityObjective",
    "ManipulabilityOptimizer",
    "OptimizationResult",
]
