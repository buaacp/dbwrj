"""Constrained configuration-dependent static trim for the UAM model."""

from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np
import pinocchio as pin

from .actuation import UamActuation
from .model_loader import MODULE_ROOT, UamModel, load_yaml


STRICT_FEASIBLE = "STRICT_FEASIBLE"
APPROXIMATE_ONLY = "APPROXIMATE_ONLY"
ROTOR_SATURATION = "ROTOR_SATURATION"
JOINT_TORQUE_SATURATION = "JOINT_TORQUE_SATURATION"
UNACTUATED_WRENCH = "UNACTUATED_WRENCH"
NUMERICAL_FAILURE = "NUMERICAL_FAILURE"


@dataclass
class TrimResult:
    """Static-trim solution and independent residual/acceleration evidence."""

    success: bool
    strict_feasible: bool
    q: np.ndarray
    u_eq: np.ndarray
    tau_required: np.ndarray
    tau_generated: np.ndarray
    generalized_force_residual: np.ndarray
    base_force_residual: np.ndarray
    base_moment_residual: np.ndarray
    joint_torque_residual: np.ndarray
    aba_acceleration: np.ndarray
    rotor_margins: np.ndarray
    joint_torque_margins: np.ndarray
    status: str
    diagnostics: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        """Convert arrays recursively into YAML/JSON-safe lists."""
        return {
            "success": self.success,
            "strict_feasible": self.strict_feasible,
            "q": self.q.tolist(),
            "u_eq": self.u_eq.tolist(),
            "tau_required": self.tau_required.tolist(),
            "tau_generated": self.tau_generated.tolist(),
            "generalized_force_residual": self.generalized_force_residual.tolist(),
            "base_force_residual": self.base_force_residual.tolist(),
            "base_moment_residual": self.base_moment_residual.tolist(),
            "joint_torque_residual": self.joint_torque_residual.tolist(),
            "aba_acceleration": self.aba_acceleration.tolist(),
            "rotor_margins": self.rotor_margins.tolist(),
            "joint_torque_margins": self.joint_torque_margins.tolist(),
            "status": self.status,
            "diagnostics": _plain(self.diagnostics),
        }


def _plain(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {key: _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    return value


class StaticTrimSolver:
    """Compute constrained static equilibrium inputs for the floating-base UAM."""

    def __init__(self, robot: UamModel, actuation: UamActuation,
                 config_path: Optional[Path] = None, prediction_model: Any = None) -> None:
        self.robot = robot
        self.actuation = actuation
        self.config = load_yaml(config_path or MODULE_ROOT / "config" / "static_trim_scenarios.yaml")
        self.solver_config = self.config["solver"]
        self.prediction_model = prediction_model
        self.B = np.asarray(actuation.mapping, dtype=float).copy()
        self.lower, self.upper = actuation.control_bounds()
        self.rank = int(np.linalg.matrix_rank(self.B))
        if self.B.shape != (robot.model.nv, actuation.nu):
            raise ValueError(f"Unexpected trim mapping shape {self.B.shape}")

    def compute_required_generalized_force(self, q: np.ndarray) -> np.ndarray:
        """Compute h(q)=RNEA(q,0,0) entirely with Pinocchio."""
        q = self._configuration(q)
        zero = np.zeros(self.robot.model.nv)
        return np.asarray(pin.rnea(
            self.robot.model, self.robot.model.createData(), q, zero, zero)).copy()

    def solve_trim(self, q: np.ndarray, *, strict: bool = True) -> TrimResult:
        """Solve strict bounded equality trim, then an explicitly approximate fallback."""
        q = self._configuration(q)
        required = self.compute_required_generalized_force(q)
        bias = self.actuation.equal_hover_control(float(np.linalg.norm(self.robot.model.gravity.linear)))
        strict_candidate, strict_diagnostics = self._solve_strict_qp(required, bias)
        tolerance = float(self.solver_config["strict_absolute_tolerance"])
        bound_tolerance = float(self.solver_config["bound_tolerance"])
        strict_residual = self.B @ strict_candidate - required
        within_bounds = bool(np.all(strict_candidate >= self.lower - bound_tolerance)
                             and np.all(strict_candidate <= self.upper + bound_tolerance))
        exact = bool(np.linalg.norm(strict_residual, ord=np.inf) <= tolerance)
        strict_feasible = exact and within_bounds and strict_diagnostics["solver_success"]

        if strict_feasible:
            control = strict_candidate
            status = STRICT_FEASIBLE
            success = True
            approximate_diagnostics: Dict[str, Any] = {}
        else:
            control, approximate_diagnostics = self._solve_approximate(required, bias)
            status = self._classify_infeasibility(
                required, strict_candidate, exact, within_bounds,
                strict_diagnostics, approximate_diagnostics)
            success = bool(approximate_diagnostics.get("solver_success", False))
            if strict and not success:
                status = NUMERICAL_FAILURE

        generated = self.B @ control
        residual = generated - required
        acceleration = self.validate_with_aba(q, control)
        margins = self.compute_margins(control)
        diagnostics = {
            "mapping_shape": list(self.B.shape), "mapping_rank": self.rank,
            "strict_solver": strict_diagnostics,
            "approximate_solver": approximate_diagnostics,
            "strict_residual_inf": float(np.linalg.norm(strict_residual, ord=np.inf)),
            "strict_candidate_within_bounds": within_bounds,
            "unbounded_equality_candidate_within_bounds": bool(
                strict_diagnostics.get("unbounded_candidate_within_bounds", False)),
            "required_generalized_force_norm": float(np.linalg.norm(required)),
            "generated_generalized_force_norm": float(np.linalg.norm(generated)),
        }
        return TrimResult(
            success=success, strict_feasible=strict_feasible, q=q.copy(), u_eq=control.copy(),
            tau_required=required, tau_generated=generated,
            generalized_force_residual=residual,
            base_force_residual=residual[:3].copy(),
            base_moment_residual=residual[3:6].copy(),
            joint_torque_residual=residual[6:].copy(),
            aba_acceleration=acceleration,
            rotor_margins=margins["rotor"],
            joint_torque_margins=margins["joint"],
            status=status, diagnostics=diagnostics)

    def validate_with_aba(self, q: np.ndarray, u_eq: np.ndarray) -> np.ndarray:
        """Independently evaluate ABA(q,0,Bu) for a proposed trim input."""
        q = self._configuration(q)
        control = np.asarray(u_eq, dtype=float).reshape(self.actuation.nu)
        tau = self.actuation.physical_control_to_generalized_torque(control)
        return np.asarray(pin.aba(
            self.robot.model, self.robot.model.createData(), q,
            np.zeros(self.robot.model.nv), tau)).copy()

    def rollout_validation(self, q: np.ndarray, u_eq: np.ndarray,
                           duration_s: float, dt: float) -> Dict[str, Any]:
        """Roll out fixed trim through the canonical Crocoddyl prediction model."""
        if self.prediction_model is None:
            from .prediction_model import UAMPredictionModel
            self.prediction_model = UAMPredictionModel(self.robot, self.actuation)
        q = self._configuration(q)
        x0 = np.concatenate((q, np.zeros(self.robot.model.nv)))
        steps = int(round(float(duration_s) / float(dt)))
        controls = np.tile(np.asarray(u_eq, dtype=float), (steps, 1))
        states = self.prediction_model.rollout(x0, controls, float(dt))
        initial_rotation = pin.Quaternion(q[3:7]).matrix()
        arm_indices = [joint.idx_q for joint in self.robot.arm_joints]
        position_errors = []
        rotation_errors = []
        joint_errors = []
        velocity_errors = []
        for state in states:
            current_q = state[:self.robot.model.nq]
            current_rotation = pin.Quaternion(current_q[3:7]).matrix()
            position_errors.append(float(np.linalg.norm(current_q[:3] - q[:3])))
            rotation_errors.append(float(np.linalg.norm(pin.log3(initial_rotation.T @ current_rotation))))
            joint_errors.append(float(np.linalg.norm(current_q[arm_indices] - q[arm_indices])))
            velocity_errors.append(float(np.linalg.norm(state[self.robot.model.nq:])))
        return {
            "duration_s": float(duration_s), "dt_s": float(dt),
            "position_error": position_errors, "rotation_error": rotation_errors,
            "joint_position_error": joint_errors, "velocity_norm": velocity_errors,
            "max_position_error": max(position_errors),
            "max_rotation_error": max(rotation_errors),
            "max_joint_position_error": max(joint_errors),
            "max_velocity_norm": max(velocity_errors),
            "finite": bool(np.all(np.isfinite(states))),
        }

    def compute_margins(self, u_eq: np.ndarray) -> Dict[str, np.ndarray]:
        """Return lower/upper rotor margins and nearest joint torque margins."""
        control = np.asarray(u_eq, dtype=float).reshape(self.actuation.nu)
        rotor = control[:self.actuation.n_rotors]
        rotor_margins = np.column_stack((
            rotor - self.lower[:self.actuation.n_rotors],
            self.upper[:self.actuation.n_rotors] - rotor))
        joint = control[self.actuation.n_rotors:]
        joint_lower = self.lower[self.actuation.n_rotors:]
        joint_upper = self.upper[self.actuation.n_rotors:]
        joint_margins = np.minimum(joint - joint_lower, joint_upper - joint)
        return {"rotor": rotor_margins, "joint": joint_margins}

    def _solve_strict_qp(self, required: np.ndarray, bias: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any]]:
        from scipy import linalg, optimize
        rotor_weight = float(self.solver_config["bias_weights"]["rotor"])
        joint_weight = float(self.solver_config["bias_weights"]["joint"])
        weights = np.concatenate((
            np.full(self.actuation.n_rotors, rotor_weight),
            np.full(self.robot.n_arm, joint_weight)))
        inverse_sqrt = 1.0 / np.sqrt(weights)
        weighted_map = self.B * inverse_sqrt[np.newaxis, :]
        correction = inverse_sqrt * (np.linalg.pinv(weighted_map) @ (required - self.B @ bias))
        analytic = bias + correction

        _, _, pivots = linalg.qr(self.B.T, pivoting=True, mode="economic")
        independent_rows = np.asarray(pivots[:self.rank], dtype=int)
        equality_map = self.B[independent_rows]
        equality_target = required[independent_rows]
        initial = np.clip(analytic, self.lower, self.upper)

        def objective(control: np.ndarray) -> float:
            error = control - bias
            return 0.5 * float(np.dot(weights * error, error))

        def gradient(control: np.ndarray) -> np.ndarray:
            return weights * (control - bias)

        result = optimize.minimize(
            objective, initial, jac=gradient, method="SLSQP",
            bounds=optimize.Bounds(self.lower, self.upper),
            constraints=[optimize.LinearConstraint(equality_map, equality_target, equality_target)],
            options={"ftol": 1e-12, "maxiter": int(self.solver_config["qp_max_iterations"]),
                     "disp": False})
        candidate = np.asarray(result.x, dtype=float) if np.all(np.isfinite(result.x)) else analytic
        residual_inf = float(np.linalg.norm(self.B @ candidate - required, ord=np.inf))
        solver_success = bool(result.success and residual_inf <= float(
            self.solver_config["strict_absolute_tolerance"]))
        if not solver_success:
            analytic_residual = float(np.linalg.norm(self.B @ analytic - required, ord=np.inf))
            analytic_bounds = bool(np.all(analytic >= self.lower) and np.all(analytic <= self.upper))
            if analytic_residual <= float(self.solver_config["strict_absolute_tolerance"]) and analytic_bounds:
                candidate = analytic
                residual_inf = analytic_residual
                solver_success = True
        return candidate, {
            "method": "scipy.optimize.SLSQP_reduced_independent_equalities",
            "solver_success": solver_success, "scipy_success": bool(result.success),
            "message": str(result.message), "iterations": int(result.nit),
            "independent_equality_rows": independent_rows.tolist(),
            "residual_inf": residual_inf,
            "unbounded_candidate": analytic.tolist(),
            "unbounded_candidate_within_bounds": bool(
                np.all(analytic >= self.lower) and np.all(analytic <= self.upper)),
        }

    def _solve_approximate(self, required: np.ndarray, bias: np.ndarray) -> Tuple[np.ndarray, Dict[str, Any]]:
        from scipy import optimize
        gravity_force = self.robot.total_mass * float(np.linalg.norm(self.robot.model.gravity.linear))
        rotor_positions = np.asarray([item["position"] for item in self.actuation.rotors], dtype=float)
        characteristic_length = max(float(np.linalg.norm(item[:2])) for item in rotor_positions)
        joint_scale = np.maximum(np.abs(self.lower[self.actuation.n_rotors:]),
                                 np.abs(self.upper[self.actuation.n_rotors:]))
        weights = self.solver_config["approximate_residual_weights"]
        scales = np.concatenate((
            np.full(3, float(weights["force"]) / gravity_force),
            np.full(3, float(weights["moment"]) / (gravity_force * characteristic_length)),
            float(weights["joint"]) / joint_scale))
        regularization = float(self.solver_config["approximate_bias_regularization"])
        augmented_map = np.vstack((scales[:, None] * self.B,
                                   np.sqrt(regularization) * np.eye(self.actuation.nu)))
        augmented_target = np.concatenate((scales * required,
                                           np.sqrt(regularization) * bias))
        result = optimize.lsq_linear(
            augmented_map, augmented_target, bounds=(self.lower, self.upper),
            method="trf", tol=1e-12, max_iter=int(self.solver_config["qp_max_iterations"]))
        return np.asarray(result.x, dtype=float), {
            "method": "scipy.optimize.lsq_linear_bounded_weighted_residual",
            "solver_success": bool(result.success), "message": str(result.message),
            "iterations": int(result.nit), "cost": float(result.cost),
        }

    def _classify_infeasibility(self, required: np.ndarray, strict_candidate: np.ndarray,
                                exact: bool, within_bounds: bool,
                                strict_diagnostics: Dict[str, Any],
                                approximate_diagnostics: Dict[str, Any]) -> str:
        if not strict_diagnostics.get("solver_success") and not approximate_diagnostics.get("solver_success"):
            return NUMERICAL_FAILURE
        uncontrollable_rows = np.linalg.norm(self.B, axis=1) < 1e-12
        if np.any(np.abs(required[uncontrollable_rows]) > float(
                self.solver_config["strict_absolute_tolerance"])):
            return UNACTUATED_WRENCH
        unbounded = np.asarray(
            strict_diagnostics.get("unbounded_candidate", strict_candidate), dtype=float)
        if not bool(strict_diagnostics.get("unbounded_candidate_within_bounds", within_bounds)):
            rotor = unbounded[:self.actuation.n_rotors]
            if np.any(rotor < self.lower[:self.actuation.n_rotors]) or np.any(
                    rotor > self.upper[:self.actuation.n_rotors]):
                return ROTOR_SATURATION
            return JOINT_TORQUE_SATURATION
        if self.rank < self.actuation.nu or not exact:
            return APPROXIMATE_ONLY
        return APPROXIMATE_ONLY

    def _configuration(self, q: np.ndarray) -> np.ndarray:
        value = np.asarray(q, dtype=float).reshape(-1)
        if value.size != self.robot.model.nq or not np.all(np.isfinite(value)):
            raise ValueError(f"q must contain {self.robot.model.nq} finite values")
        quaternion_norm = float(np.linalg.norm(value[3:7]))
        if abs(quaternion_norm - 1.0) > 1e-8:
            raise ValueError(f"Free-flyer quaternion norm must be one, got {quaternion_norm}")
        return value.copy()
