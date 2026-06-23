"""Discrete local LQR on the 24-dimensional UAM state tangent space."""

from dataclasses import dataclass
from typing import Any, Dict, Tuple

import numpy as np
from scipy import linalg

from .prediction_model import UAMPredictionModel


@dataclass
class LQRDesign:
    """DARE solution and open/closed local-system diagnostics."""

    A: np.ndarray
    B: np.ndarray
    K: np.ndarray
    P: np.ndarray
    open_eigenvalues: np.ndarray
    closed_eigenvalues: np.ndarray
    diagnostics: Dict[str, Any]


class LocalTrimLQR:
    """Local discrete-time LQR regulator around a static-trim equilibrium.

    Tangent state:
        dx = StateMultibody.diff(x_eq, x), dimension ndx=24.

    Physical control:
        u = [T1,T2,T3,T4,tau_arm_1,...,tau_arm_6], dimension nu=10.
    """

    def __init__(self, prediction_model: UAMPredictionModel,
                 rank_relative_tolerance: float = 1e-9,
                 eigenvalue_tolerance: float = 1e-8) -> None:
        self.prediction = prediction_model
        self.state = prediction_model.state
        self.rank_relative_tolerance = float(rank_relative_tolerance)
        self.eigenvalue_tolerance = float(eigenvalue_tolerance)

    def linearize_at_trim(self, x_eq: np.ndarray, u_eq: np.ndarray,
                          dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """Reuse Crocoddyl calcDiff through UAMPredictionModel.linearize()."""
        return self.prediction.linearize(x_eq, u_eq, dt)

    def solve_dlqr(self, A_d: np.ndarray, B_d: np.ndarray,
                   Q: np.ndarray, R: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Solve the infinite-horizon discrete algebraic Riccati equation."""
        A_d = np.asarray(A_d, dtype=float)
        B_d = np.asarray(B_d, dtype=float)
        Q = np.asarray(Q, dtype=float)
        R = np.asarray(R, dtype=float)
        if A_d.shape != (self.prediction.ndx, self.prediction.ndx):
            raise ValueError(f"A_d has wrong shape {A_d.shape}")
        if B_d.shape != (self.prediction.ndx, self.prediction.nu):
            raise ValueError(f"B_d has wrong shape {B_d.shape}")
        if np.min(np.linalg.eigvalsh(Q)) <= 0.0 or np.min(np.linalg.eigvalsh(R)) <= 0.0:
            raise ValueError("Q and R must be positive definite")
        P = linalg.solve_discrete_are(A_d, B_d, Q, R)
        gain = np.linalg.solve(R + B_d.T @ P @ B_d, B_d.T @ P @ A_d)
        eigenvalues = np.linalg.eigvals(A_d - B_d @ gain)
        return gain, P, eigenvalues

    def design(self, x_eq: np.ndarray, u_eq: np.ndarray, dt: float,
               Q: np.ndarray, R: np.ndarray) -> LQRDesign:
        """Linearize, solve DARE, and attach controllability diagnostics."""
        A_d, B_d = self.linearize_at_trim(x_eq, u_eq, dt)
        gain, riccati, closed_eigenvalues = self.solve_dlqr(A_d, B_d, Q, R)
        diagnostics = self.analyze_controllability_or_stabilizability(A_d, B_d)
        return LQRDesign(
            A=A_d, B=B_d, K=gain, P=riccati,
            open_eigenvalues=np.linalg.eigvals(A_d),
            closed_eigenvalues=closed_eigenvalues,
            diagnostics=diagnostics)

    def control(self, x: np.ndarray, x_eq: np.ndarray, u_eq: np.ndarray,
                K: np.ndarray, u_lb: np.ndarray,
                u_ub: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute clipped manifold LQR control and per-channel saturation flags."""
        dx = np.asarray(self.state.diff(x_eq, x), dtype=float)
        raw = np.asarray(u_eq, dtype=float) - np.asarray(K, dtype=float) @ dx
        command = np.clip(raw, u_lb, u_ub)
        saturated = np.abs(raw - command) > 1e-12
        return command, dx, saturated

    def rollout_closed_loop(self, x0: np.ndarray, x_eq: np.ndarray,
                            u_eq: np.ndarray, K: np.ndarray,
                            duration_s: float, dt: float) -> Dict[str, np.ndarray]:
        """Roll out nonlinear f_pred under clipped local LQR feedback."""
        lower, upper = self.prediction.get_control_bounds()
        steps = int(round(float(duration_s) / float(dt)))
        x = np.asarray(x0, dtype=float).copy()
        states = [x.copy()]
        controls = []
        tangent_errors = [np.asarray(self.state.diff(x_eq, x), dtype=float)]
        saturation = []
        raw_controls = []
        for _ in range(steps):
            command, dx, saturated = self.control(
                x, x_eq, u_eq, K, lower, upper)
            raw_controls.append(np.asarray(u_eq) - K @ dx)
            controls.append(command)
            saturation.append(saturated)
            x = self.prediction.step(x, command, dt)
            states.append(x.copy())
            if not np.all(np.isfinite(x)):
                tangent_errors.append(np.full(self.prediction.ndx, np.nan))
                break
            tangent_errors.append(np.asarray(self.state.diff(x_eq, x), dtype=float))
        return {
            "states": np.asarray(states), "controls": np.asarray(controls),
            "raw_controls": np.asarray(raw_controls),
            "saturation": np.asarray(saturation, dtype=bool),
            "tangent_errors": np.asarray(tangent_errors),
        }

    def rollout_open_loop(self, x0: np.ndarray, x_eq: np.ndarray,
                          u_eq: np.ndarray, duration_s: float,
                          dt: float) -> Dict[str, np.ndarray]:
        """Roll out nonlinear f_pred with fixed static-trim control."""
        steps = int(round(float(duration_s) / float(dt)))
        x = np.asarray(x0, dtype=float).copy()
        states = [x.copy()]
        tangent_errors = [np.asarray(self.state.diff(x_eq, x), dtype=float)]
        controls = []
        for _ in range(steps):
            controls.append(np.asarray(u_eq, dtype=float).copy())
            x = self.prediction.step(x, u_eq, dt)
            states.append(x.copy())
            if not np.all(np.isfinite(x)):
                tangent_errors.append(np.full(self.prediction.ndx, np.nan))
                break
            tangent_errors.append(np.asarray(self.state.diff(x_eq, x), dtype=float))
        controls_array = np.asarray(controls)
        return {
            "states": np.asarray(states), "controls": controls_array,
            "raw_controls": controls_array.copy(),
            "saturation": np.zeros_like(controls_array, dtype=bool),
            "tangent_errors": np.asarray(tangent_errors),
        }

    def analyze_controllability_or_stabilizability(self, A_d: np.ndarray,
                                                    B_d: np.ndarray) -> Dict[str, Any]:
        """Report scaled controllability rank and PBH tests for unstable modes."""
        n = A_d.shape[0]
        blocks = []
        propagated = np.asarray(B_d, dtype=float)
        for _ in range(n):
            blocks.append(propagated)
            propagated = A_d @ propagated
        controllability = np.hstack(blocks)
        singular_values = np.linalg.svd(controllability, compute_uv=False)
        threshold = self.rank_relative_tolerance * singular_values[0]
        rank = int(np.sum(singular_values > threshold))
        eigenvalues = np.linalg.eigvals(A_d)
        unstable = eigenvalues[np.abs(eigenvalues) >= 1.0 - self.eigenvalue_tolerance]
        uncontrollable_unstable = []
        pbh_ranks = []
        for eigenvalue in unstable:
            pbh = np.hstack((eigenvalue * np.eye(n) - A_d, B_d)).astype(complex)
            values = np.linalg.svd(pbh, compute_uv=False)
            pbh_threshold = self.rank_relative_tolerance * values[0]
            pbh_rank = int(np.sum(values > pbh_threshold))
            pbh_ranks.append(pbh_rank)
            if pbh_rank < n:
                uncontrollable_unstable.append(eigenvalue)
        return {
            "controllability_rank": rank,
            "state_tangent_dimension": n,
            "controllability_singular_values": singular_values.tolist(),
            "rank_threshold": float(threshold),
            "pbh_tested_modes": len(unstable),
            "pbh_minimum_rank": min(pbh_ranks) if pbh_ranks else n,
            "uncontrollable_unstable_modes": [
                {"real": float(value.real), "imag": float(value.imag),
                 "magnitude": float(abs(value))}
                for value in uncontrollable_unstable],
            "stabilizable": not uncontrollable_unstable,
        }
