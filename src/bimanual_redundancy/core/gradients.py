"""Finite-difference gradient evaluation, shared by the paper objectives and
the experimental directional-distance permutation study.

This module contains no paper mathematics itself: it evaluates the joint-space
derivative of whatever scalar ``evaluate(data)`` a caller supplies, using
central differences. ``ManipulabilityOptimizer.gradient`` (Equations 13-17)
and ``DirectionalDistancePermutationOptimizer.gradient`` both call this
function so the perturb/restore mechanics exist in exactly one place.
"""

import mujoco
import numpy as np


def central_difference_gradient(model, data, qpos_indices, step, evaluate):
    """Return d(evaluate)/d(qpos) at ``qpos_indices`` via central differences.

    For each joint index, perturbs ``data.qpos`` by ``+-step``, re-runs
    forward kinematics, and evaluates ``evaluate(data)`` at each perturbation::

        gradient[i] = (evaluate(q + step*e_i) - evaluate(q - step*e_i)) / (2*step)

    ``data.qpos`` is restored to its original value (and forward kinematics
    re-run) before returning, including on error. This is the numerical
    realization of ``dW/dphi`` in Equation (4); it does not itself select
    which scalar ``W`` is differentiated.
    """
    gradient = np.zeros(len(qpos_indices))
    original_qpos = data.qpos.copy()

    try:
        for column, qpos_index in enumerate(qpos_indices):
            center = original_qpos[qpos_index]

            data.qpos[qpos_index] = center + step
            mujoco.mj_forward(model, data)
            value_plus = evaluate(data)

            data.qpos[qpos_index] = center - step
            mujoco.mj_forward(model, data)
            value_minus = evaluate(data)

            gradient[column] = (value_plus - value_minus) / (2.0 * step)
            data.qpos[qpos_index] = center
    finally:
        data.qpos[:] = original_qpos
        mujoco.mj_forward(model, data)

    return gradient
