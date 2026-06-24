"""Strong-soft terminal full-body rest costs and metrics."""

from typing import Any, Dict, Tuple

import numpy as np

from .model_loader import UamModel


REQUIRED_KEYS = (
    "enabled",
    "base_linear_velocity_weight",
    "base_angular_velocity_weight",
    "arm_joint_velocity_weight",
    "base_linear_velocity_tolerance_mps",
    "base_angular_velocity_tolerance_radps",
    "arm_joint_velocity_tolerance_radps",
    "barrier_weight",
    "window_steps",
    "pass_base_linear_velocity_norm_mps",
    "pass_base_angular_velocity_norm_radps",
    "pass_arm_joint_velocity_inf_radps",
)


def terminal_rest_config(config: Dict[str, Any]) -> Dict[str, Any]:
    """Return validated terminal-rest settings from a scenario mapping."""
    if "terminal_rest" not in config:
        raise KeyError("Missing required terminal_rest configuration")
    rest = dict(config["terminal_rest"])
    missing = [key for key in REQUIRED_KEYS if key not in rest]
    if missing:
        raise KeyError(f"terminal_rest is missing required keys: {missing}")
    rest["enabled"] = bool(rest["enabled"])
    rest["window_steps"] = int(rest["window_steps"])
    for key in REQUIRED_KEYS:
        if key not in ("enabled", "window_steps"):
            rest[key] = float(rest[key])
    if rest["window_steps"] < 0:
        raise ValueError("terminal_rest.window_steps must be nonnegative")
    return rest


def velocity_weight_vector(robot: UamModel, rest: Dict[str, Any]) -> np.ndarray:
    """Build StateMultibody tangent weights for v=[vB,wB,dqa] at dx[nv:]."""
    nv = int(robot.model.nv)
    weights = np.zeros(robot.state.ndx)
    velocity = np.zeros(nv)
    velocity[:3] = float(rest["base_linear_velocity_weight"])
    velocity[3:6] = float(rest["base_angular_velocity_weight"])
    velocity[6:] = float(rest["arm_joint_velocity_weight"])
    weights[nv:] = velocity
    return weights


def velocity_barrier_bounds(robot: UamModel, rest: Dict[str, Any]) -> Tuple[np.ndarray, np.ndarray]:
    """Build quadratic-barrier bounds for terminal full-body velocity residuals."""
    nv = int(robot.model.nv)
    lower = np.full(robot.state.ndx, -1.0e9)
    upper = np.full(robot.state.ndx, 1.0e9)
    tolerances = np.zeros(nv)
    tolerances[:3] = float(rest["base_linear_velocity_tolerance_mps"])
    tolerances[3:6] = float(rest["base_angular_velocity_tolerance_radps"])
    tolerances[6:] = float(rest["arm_joint_velocity_tolerance_radps"])
    lower[nv:] = -tolerances
    upper[nv:] = tolerances
    return lower, upper


def add_terminal_rest_costs(costs: Any, robot: UamModel, nu: int, xref: np.ndarray,
                            rest: Dict[str, Any], terminal: bool,
                            window_scale: float = 1.0) -> None:
    """Add strong-soft full-body rest costs using StateMultibody residuals."""
    if not rest.get("enabled", False):
        return
    import crocoddyl
    state = robot.state
    x_rest = np.asarray(xref, dtype=float).copy()
    x_rest[robot.model.nq:] = 0.0
    residual = crocoddyl.ResidualModelState(state, x_rest, nu)
    weights = velocity_weight_vector(robot, rest) * float(window_scale)
    costs.addCost(
        "terminal_full_body_rest" if terminal else "running_full_body_rest_window",
        crocoddyl.CostModelResidual(
            state, crocoddyl.ActivationModelWeightedQuad(weights), residual),
        1.0)
    lower, upper = velocity_barrier_bounds(robot, rest)
    costs.addCost(
        "terminal_full_body_rest_barrier" if terminal else "running_full_body_rest_barrier_window",
        crocoddyl.CostModelResidual(
            state,
            crocoddyl.ActivationModelQuadraticBarrier(crocoddyl.ActivationBounds(lower, upper)),
            residual),
        float(rest["barrier_weight"]) * float(window_scale))


def metrics_from_state(robot: UamModel, state: np.ndarray, rest: Dict[str, Any]) -> Dict[str, Any]:
    """Compute terminal full-body rest metrics from x=[q,v]."""
    v = np.asarray(state, dtype=float)[robot.model.nq:]
    base_linear = float(np.linalg.norm(v[:3]))
    base_angular = float(np.linalg.norm(v[3:6]))
    arm_inf = float(np.max(np.abs(v[6:]))) if v.size > 6 else 0.0
    full = float(np.linalg.norm(v))
    passed = (
        base_linear <= float(rest["pass_base_linear_velocity_norm_mps"])
        and base_angular <= float(rest["pass_base_angular_velocity_norm_radps"])
        and arm_inf <= float(rest["pass_arm_joint_velocity_inf_radps"]))
    return {
        "terminal_base_linear_velocity_norm_mps": base_linear,
        "terminal_base_angular_velocity_norm_radps": base_angular,
        "terminal_max_arm_joint_velocity_radps": arm_inf,
        "terminal_full_body_velocity_norm": full,
        "terminal_rest_pass": bool(passed),
        "terminal_rest_mode": "strong-soft terminal rest condition",
        "terminal_rest_window_steps": int(rest["window_steps"]),
    }
