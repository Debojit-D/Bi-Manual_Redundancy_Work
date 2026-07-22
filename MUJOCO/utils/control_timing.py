"""Precise computation-only timing for Equation (8) controller updates."""

from dataclasses import dataclass
import time

import numpy as np

from MUJOCO.utils.redundancy_optimization import OptimizationResult


NANOSECONDS_PER_MILLISECOND = 1_000_000.0


@dataclass(frozen=True)
class ControlComputeTiming:
    """Optimizer, controller-update, and combined elapsed milliseconds."""

    optimizer_time_ms: float
    controller_update_time_ms: float
    control_compute_time_ms: float


def timed_equation_8_update(
    scene,
    equation_8,
    optimizer,
    phi,
    desired_position,
    desired_rotation,
    desired_twist,
    *,
    enable_redundancy_optimization=True,
):
    """Compute one Equation (8) command and measure only its CPU section.

    Timing begins immediately before the optional optimizer call and ends
    immediately after ``equation_8.update``. Command dispatch, MuJoCo stepping,
    rendering, recording, and rate limiting remain outside this function.
    """
    control_start_ns = time.perf_counter_ns()
    if enable_redundancy_optimization:
        optimization = optimizer.optimization_velocity(scene.data)
        optimizer_end_ns = time.perf_counter_ns()
        optimizer_time_ms = (
            optimizer_end_ns - control_start_ns
        ) / NANOSECONDS_PER_MILLISECOND
    else:
        zero_velocity = np.zeros(scene.arm_dofs.size)
        optimization = OptimizationResult(
            objective=optimizer.objective,
            value=optimizer.value(scene.data),
            gradient=zero_velocity.copy(),
            phi_dot_opt=zero_velocity,
        )
        optimizer_time_ms = 0.0

    controller_start_ns = time.perf_counter_ns()
    phi, diagnostics = equation_8.update(
        scene.data,
        phi,
        desired_position,
        desired_rotation,
        desired_twist,
        optimization.phi_dot_opt,
    )
    controller_end_ns = time.perf_counter_ns()
    controller_update_time_ms = (
        controller_end_ns - controller_start_ns
    ) / NANOSECONDS_PER_MILLISECOND
    control_compute_time_ms = (
        controller_end_ns - control_start_ns
    ) / NANOSECONDS_PER_MILLISECOND
    timing = ControlComputeTiming(
        optimizer_time_ms=optimizer_time_ms,
        controller_update_time_ms=controller_update_time_ms,
        control_compute_time_ms=control_compute_time_ms,
    )
    return phi, optimization, diagnostics, timing
