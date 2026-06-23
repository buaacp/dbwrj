"""Shared rollout and manifold-error helpers for prediction validation."""

from typing import Any, Sequence

import numpy as np


def rollout_action_models(problem: Any, controls: Sequence[np.ndarray]) -> np.ndarray:
    """Roll out a Crocoddyl ShootingProblem with the supplied controls."""
    if len(controls) != problem.T:
        raise ValueError(f"Expected {problem.T} controls, got {len(controls)}")
    return np.asarray(problem.rollout([np.asarray(item, dtype=float) for item in controls]))


def manifold_errors(state: Any, reference: np.ndarray, candidate: np.ndarray) -> np.ndarray:
    """Return per-node StateMultibody tangent error norms."""
    reference = np.asarray(reference)
    candidate = np.asarray(candidate)
    if reference.shape != candidate.shape:
        raise ValueError(f"Trajectory shapes differ: {reference.shape} and {candidate.shape}")
    return np.asarray([
        np.linalg.norm(state.diff(reference[index], candidate[index]))
        for index in range(reference.shape[0])
    ])

