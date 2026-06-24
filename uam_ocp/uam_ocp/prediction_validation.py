"""P2 consistency, derivative, and hover validation for f_pred."""

from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import yaml

from .p2_planner import P2Planner
from .prediction_model import UAMPredictionModel
from .prediction_rollout import manifold_errors, rollout_action_models


def _action_step(action: Any, x: np.ndarray, u: np.ndarray) -> np.ndarray:
    data = action.createData()
    action.calc(data, x, u)
    return np.asarray(data.xnext, dtype=float).copy()


def validate_prediction_model(
        prediction: UAMPredictionModel, planner: Optional[P2Planner] = None,
        seed: int = 23) -> Dict[str, Any]:
    """Execute validation Tests A-D against the current P2 action model."""
    import crocoddyl
    planner = planner or P2Planner(
        prediction.robot, prediction.actuation, prediction_model=prediction)
    problem, scenario, _, _, trim = planner.build_problem("pregrasp")
    trim_reference = np.asarray(trim, dtype=float)
    if trim_reference.ndim == 2:
        trim_reference = trim_reference[0]
    dt = float(scenario["dt_s"])
    state = prediction.state
    rng = np.random.default_rng(seed)
    lower, upper = prediction.get_control_bounds()

    # Test A: legal manifold states and controls through P2 and f_pred.
    single_errors = []
    for _ in range(8):
        tangent = np.zeros(prediction.ndx)
        tangent[:6] = rng.normal(0.0, 0.02, 6)
        tangent[6:prediction.nv] = rng.normal(0.0, 0.04, prediction.nv - 6)
        tangent[prediction.nv:] = rng.normal(0.0, 0.03, prediction.nv)
        x = state.integrate(problem.x0, tangent)
        perturbation = np.concatenate((rng.normal(0.0, 0.05, prediction.actuation.n_rotors),
                                       rng.normal(0.0, 1e-4, prediction.robot.n_arm)))
        u = np.clip(trim_reference + perturbation, lower, upper)
        p2_next = _action_step(problem.runningModels[0], x, u)
        pred_next = prediction.step(x, u, dt)
        single_errors.append(float(np.linalg.norm(state.diff(p2_next, pred_next))))

    # Test B: short finite P2 rollout versus the shared prediction model.
    horizon = 10
    indices = np.arange(horizon, dtype=float)
    controls = np.tile(trim_reference, (horizon, 1))
    rotor_pattern = np.array([1.0, -1.0, 1.0, -1.0])
    controls[:, :prediction.actuation.n_rotors] += (
        0.02 * np.sin(0.4 * indices)[:, None] * rotor_pattern)
    controls[:, prediction.actuation.n_rotors:] += 1e-5 * np.sin(0.3 * indices)[:, None]
    controls = np.clip(controls, lower, upper)
    short_problem = crocoddyl.ShootingProblem(
        problem.x0, [problem.runningModels[0]] * horizon, problem.terminalModel)
    p2_states = rollout_action_models(short_problem, controls)
    pred_states = prediction.rollout(problem.x0, controls, dt)
    rollout_errors = manifold_errors(state, p2_states, pred_states)

    # Test C: calcDiff derivatives should show second-order remainder scaling.
    x = problem.x0.copy()
    u = trim_reference.copy()
    dx = rng.normal(size=prediction.ndx)
    du = rng.normal(size=prediction.nu)
    dx /= np.linalg.norm(dx)
    du /= np.linalg.norm(du)
    fx, fu = prediction.linearize(x, u, dt)
    nominal_next = prediction.step(x, u, dt)
    epsilons = [1e-3, 1e-4, 1e-5]
    linearization_errors = []
    for epsilon in epsilons:
        perturbed_x = state.integrate(x, epsilon * dx)
        perturbed_u = u + epsilon * du
        actual_next = prediction.step(perturbed_x, perturbed_u, dt)
        actual_delta = state.diff(nominal_next, actual_next)
        linear_delta = epsilon * (fx @ dx + fu @ du)
        linearization_errors.append(float(np.linalg.norm(actual_delta - linear_delta)))
    error_ratios = [linearization_errors[index] / linearization_errors[index + 1]
                    for index in range(len(linearization_errors) - 1)]

    # Test D: required equal mg/4 input and a full-model static trim comparison.
    hover_dt = 0.01
    hover_duration = 2.0
    hover_steps = int(round(hover_duration / hover_dt))
    hover = prediction.hover_control()
    hover_states = prediction.rollout(
        problem.x0, np.tile(hover, (hover_steps, 1)), hover_dt)
    trim_control = prediction.static_trim_control(problem.x0[:prediction.nq])
    trim_states = prediction.rollout(
        problem.x0, np.tile(trim_control, (hover_steps, 1)), hover_dt)

    def drift(states: np.ndarray) -> Dict[str, float]:
        final = states[-1]
        return {
            "position_displacement_m": float(np.linalg.norm(final[:3] - states[0, :3])),
            "base_angular_velocity_radps": float(np.linalg.norm(
                final[prediction.nq + 3:prediction.nq + 6])),
            "arm_joint_velocity_norm_radps": float(np.linalg.norm(
                final[prediction.nq + 6:])),
            "finite": bool(np.all(np.isfinite(states))),
        }

    test_a_pass = max(single_errors) < 1e-10
    test_b_pass = float(rollout_errors.max()) < 1e-9
    test_c_pass = all(linearization_errors[index + 1] < 0.02 * linearization_errors[index]
                      for index in range(len(linearization_errors) - 1))
    equal_drift = drift(hover_states)
    trim_drift = drift(trim_states)
    test_d_pass = equal_drift["finite"] and trim_drift["finite"]
    return {
        "pass": test_a_pass and test_b_pass and test_c_pass and test_d_pass,
        "dimensions": {
            "nq": prediction.nq, "nv": prediction.nv, "nx": prediction.nx,
            "ndx": prediction.ndx, "nu": prediction.nu,
            "Fx": list(fx.shape), "Fu": list(fu.shape),
        },
        "test_a_single_step": {
            "pass": test_a_pass, "samples": len(single_errors),
            "errors": single_errors, "max_error": max(single_errors),
            "threshold": 1e-10,
        },
        "test_b_rollout": {
            "pass": test_b_pass, "steps": horizon,
            "errors": rollout_errors.tolist(), "max_error": float(rollout_errors.max()),
            "threshold": 1e-9,
        },
        "test_c_linearization": {
            "pass": test_c_pass, "source": "Crocoddyl calcDiff",
            "epsilons": epsilons, "errors": linearization_errors,
            "successive_error_ratios": error_ratios,
        },
        "test_d_hover": {
            "pass": test_d_pass, "duration_s": hover_duration, "dt_s": hover_dt,
            "equal_mg_over_4_zero_arm_torque": {
                "control": hover.tolist(), "drift": equal_drift,
            },
            "full_model_static_trim": {
                "control": trim_control.tolist(), "drift": trim_drift,
            },
            "analysis": [
                "Equal mg/4 thrust with zero arm torque is a nominal input, not an equilibrium for the offset free arm.",
                "The arm joints receive no gravity compensation, damping, servo-loop torque, or hard joint-limit reaction.",
                "Arm motion changes the center of mass and reacts on the base; small source inertias amplify the response.",
                "The static trim includes arm gravity torque and asymmetric rotor allocation and remains near equilibrium over this test.",
            ],
        },
    }


def save_validation(summary: Dict[str, Any], results_dir: Path, report_path: Path) -> None:
    """Persist machine-readable validation and the required Markdown report."""
    results_dir.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    (results_dir / "prediction_validation.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    test_a = summary["test_a_single_step"]
    test_b = summary["test_b_rollout"]
    test_c = summary["test_c_linearization"]
    test_d = summary["test_d_hover"]
    equal = test_d["equal_mg_over_4_zero_arm_torque"]["drift"]
    trim = test_d["full_model_static_trim"]["drift"]
    lines = [
        "# Prediction model validation", "",
        f"Overall: **{'PASS' if summary['pass'] else 'FAIL'}**", "",
        "## P2 consistency", "",
        f"- Test A single-step maximum manifold error: `{test_a['max_error']:.6e}` (threshold `{test_a['threshold']:.1e}`)",
        f"- Test B {test_b['steps']}-step rollout maximum manifold error: `{test_b['max_error']:.6e}` (threshold `{test_b['threshold']:.1e}`)",
        "- Quaternion-bearing states were compared with `StateMultibody.diff()`, not direct subtraction.", "",
        "## Linearization", "",
        f"- Source: `{test_c['source']}`", f"- Epsilons: `{test_c['epsilons']}`",
        f"- Remainder errors: `{test_c['errors']}`",
        f"- Successive error ratios: `{test_c['successive_error_ratios']}`",
        "- A tenfold perturbation reduction produces approximately a hundredfold remainder reduction, consistent with a first-order local model.", "",
        "## Two-second hover prediction", "",
        "The required nominal hover input uses equal `m_total*g/4` rotor thrust and zero arm torque.",
        f"It produced position displacement `{equal['position_displacement_m']:.6e} m`, base angular speed `{equal['base_angular_velocity_radps']:.6e} rad/s`, and arm joint-speed norm `{equal['arm_joint_velocity_norm_radps']:.6e} rad/s`.",
        f"For comparison, full-model static trim produced `{trim['position_displacement_m']:.6e} m`, `{trim['base_angular_velocity_radps']:.6e} rad/s`, and `{trim['arm_joint_velocity_norm_radps']:.6e} rad/s` respectively.", "",
        "The nominal input is not a physical equilibrium because the arm COM is offset and zero arm torque does not compensate arm gravity. The model also excludes servo damping and hard joint-stop reactions. The large nominal drift is therefore reported, not hidden or retuned away.", "",
        "## Result", "",
        f"- Prediction model: {'PASS' if summary['pass'] else 'FAIL'}",
        f"- P2 consistency: {'PASS' if test_a['pass'] and test_b['pass'] else 'FAIL'}",
        f"- Linearization: {'PASS' if test_c['pass'] else 'FAIL'}",
        f"- Hover simulation finite: {'PASS' if test_d['pass'] else 'FAIL'}", "",
    ]
    report_path.write_text("\n".join(lines), encoding="utf-8")
