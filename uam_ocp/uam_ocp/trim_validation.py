"""Scenario loading, metrics, artifacts, and reports for static trim."""

import csv
from copy import deepcopy
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pinocchio as pin
import yaml

from .actuation import UamActuation
from .model_loader import MODULE_ROOT, PROJECT_ROOT, UamModel, load_yaml
from .static_trim import StaticTrimSolver


def load_trim_configurations(robot: UamModel, config_path: Optional[Path] = None) -> Tuple[Dict[str, Any], List[Dict[str, Any]]]:
    """Load explicit and P2 trajectory configurations without altering values."""
    config = load_yaml(config_path or MODULE_ROOT / "config" / "static_trim_scenarios.yaml")
    base = config["base"]
    configurations: List[Dict[str, Any]] = []
    for name, item in config["scenarios"].items():
        q = robot.neutral_configuration(item["joints"])
        q[:3] = np.asarray(base["position"], dtype=float)
        q[3:7] = np.asarray(base["quaternion_xyzw"], dtype=float)
        configurations.append({"name": name, "source": item["source"], "q": q})

    trajectory_config = config["trajectory_scenarios"]
    source = PROJECT_ROOT / trajectory_config["source_npz"]
    if not source.exists():
        raise FileNotFoundError(f"P2 trajectory required for trim validation: {source}")
    states = np.load(str(source))["states"]
    def horizontal_configuration(state: np.ndarray) -> np.ndarray:
        q = robot.neutral_configuration()
        q[:3] = np.asarray(base["position"], dtype=float)
        q[3:7] = np.asarray(base["quaternion_xyzw"], dtype=float)
        for joint in robot.arm_joints:
            q[joint.idx_q] = state[joint.idx_q]
        return q
    if bool(trajectory_config.get("include_terminal", True)):
        configurations.append({
            "name": "p2_terminal", "source": str(source) + ":terminal",
            "q": horizontal_configuration(states[-1]),
        })
    count = int(trajectory_config.get("sampled_nodes", 10))
    indices = np.linspace(0, states.shape[0] - 1, count, dtype=int)
    for sample_index, state_index in enumerate(indices):
        configurations.append({
            "name": f"p2_node_{sample_index:02d}",
            "source": f"{source}:state[{state_index}]",
            "q": horizontal_configuration(states[state_index]),
        })
    return config, configurations


def configuration_limit_violations(robot: UamModel, q: np.ndarray) -> List[str]:
    """Return named independent joints outside their exact URDF limits."""
    violations = []
    for joint in robot.arm_joints:
        value = float(q[joint.idx_q])
        lower = float(robot.model.lowerPositionLimit[joint.idx_q])
        upper = float(robot.model.upperPositionLimit[joint.idx_q])
        if value < lower - 1e-12 or value > upper + 1e-12:
            violations.append(f"{joint.name}={value} outside [{lower}, {upper}]")
    return violations


def validate_trim_scenarios(robot: UamModel, actuation: UamActuation,
                            solver: StaticTrimSolver,
                            config_path: Optional[Path] = None) -> Dict[str, Any]:
    """Run strict trim, ABA, normalized residual, margin, and rollout checks."""
    config, configurations = load_trim_configurations(robot, config_path)
    validation = config["validation"]
    gravity_force = robot.total_mass * float(np.linalg.norm(robot.model.gravity.linear))
    rotor_positions = np.asarray([item["position"] for item in actuation.rotors], dtype=float)
    characteristic_length = max(float(np.linalg.norm(item[:2])) for item in rotor_positions)
    joint_scale = float(np.max(np.maximum(
        np.abs(solver.lower[actuation.n_rotors:]),
        np.abs(solver.upper[actuation.n_rotors:]))))
    entries = []
    for item in configurations:
        q = np.asarray(item["q"], dtype=float)
        violations = configuration_limit_violations(robot, q)
        if violations:
            entries.append({
                "name": item["name"], "source": item["source"], "skipped": True,
                "status": "INVALID_CONFIGURATION", "violations": violations,
            })
            continue
        result = solver.solve_trim(q)
        force_norm = float(np.linalg.norm(result.base_force_residual))
        moment_norm = float(np.linalg.norm(result.base_moment_residual))
        joint_norm = float(np.linalg.norm(result.joint_torque_residual))
        aba_linear = float(np.linalg.norm(result.aba_acceleration[:3]))
        aba_angular = float(np.linalg.norm(result.aba_acceleration[3:6]))
        aba_joint = float(np.linalg.norm(result.aba_acceleration[6:]))
        rollout = solver.rollout_validation(
            q, result.u_eq, float(validation["duration_s"]), float(validation["dt_s"]))
        normalized = {
            "force": force_norm / gravity_force,
            "moment": moment_norm / (gravity_force * characteristic_length),
            "joint": joint_norm / joint_scale,
        }
        strict_checks = {
            "force_residual": normalized["force"] < float(validation["normalized_force_tolerance"]),
            "moment_residual": normalized["moment"] < float(validation["normalized_moment_tolerance"]),
            "joint_residual": normalized["joint"] < float(validation["normalized_joint_tolerance"]),
            "aba_linear": aba_linear < float(validation["acceleration_tolerance"]),
            "aba_angular": aba_angular < float(validation["acceleration_tolerance"]),
            "aba_joint": aba_joint < float(validation["acceleration_tolerance"]),
            "rollout_finite": rollout["finite"],
        }
        entries.append({
            "name": item["name"], "source": item["source"], "skipped": False,
            "status": result.status, "strict_feasible": result.strict_feasible,
            "success": result.success,
            "result": result.to_dict(),
            "residual_norms": {"force": force_norm, "moment": moment_norm, "joint": joint_norm,
                               "generalized_inf": float(np.linalg.norm(result.generalized_force_residual, ord=np.inf))},
            "normalized_residuals": normalized,
            "aba_norms": {"linear": aba_linear, "angular": aba_angular, "joint": aba_joint},
            "rollout": rollout,
            "strict_checks": strict_checks,
            "validation_pass": bool(result.strict_feasible and all(strict_checks.values())),
        })
    evaluated = [item for item in entries if not item.get("skipped", False)]
    strict = [item for item in evaluated if item["strict_feasible"]]
    summary = {
        "pass": bool(evaluated and len(strict) == len(evaluated)
                     and all(item["validation_pass"] for item in evaluated)),
        "total_configurations": len(entries), "evaluated_configurations": len(evaluated),
        "skipped_configurations": len(entries) - len(evaluated),
        "strict_feasible_configurations": len(strict),
        "maximum_generalized_force_residual": max(
            item["residual_norms"]["generalized_inf"] for item in evaluated),
        "maximum_aba_linear_acceleration": max(item["aba_norms"]["linear"] for item in evaluated),
        "maximum_aba_angular_acceleration": max(item["aba_norms"]["angular"] for item in evaluated),
        "maximum_aba_joint_acceleration": max(item["aba_norms"]["joint"] for item in evaluated),
        "maximum_rollout_position_drift": max(item["rollout"]["max_position_error"] for item in evaluated),
        "minimum_rotor_margin": min(
            float(np.min(np.asarray(item["result"]["rotor_margins"]))) for item in strict),
        "minimum_joint_torque_margin": min(
            float(np.min(np.asarray(item["result"]["joint_torque_margins"]))) for item in strict),
        "entries": entries,
    }
    return summary


def validate_p2_static_trim_mode(robot: UamModel, actuation: UamActuation,
                                 prediction_model: Any) -> Dict[str, Any]:
    """Solve P2 with explicit per-reference-node strict static trim enabled."""
    from .p2_planner import P2Planner
    planner = P2Planner(robot, actuation, prediction_model=prediction_model)
    planner.scenarios["pregrasp"] = deepcopy(planner.scenarios["pregrasp"])
    planner.scenarios["pregrasp"]["control_reference_mode"] = "static_trim"
    solution = planner.solve("pregrasp")
    strict = [result.strict_feasible for result in planner.last_trim_results]
    passed = bool(solution.converged and len(strict) == len(solution.controls)
                  and all(strict) and solution.rollout_error < 1e-9)
    return {
        "pass": passed, "solver_converged": solution.converged,
        "iterations": solution.iterations, "nodes": len(solution.controls),
        "strict_trim_nodes": int(sum(strict)),
        "rollout_state_error": solution.rollout_error,
        "control_reference_shape": list(planner.last_control_references.shape),
    }


def save_trim_validation(summary: Dict[str, Any], output: Path, report_path: Path) -> None:
    """Write CSV/YAML, margin/drift plots, and the required validation report."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    report_path.parent.mkdir(parents=True, exist_ok=True)
    (output / "trim_results.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    evaluated = [item for item in summary["entries"] if not item.get("skipped", False)]
    rows = []
    for item in summary["entries"]:
        row = {"name": item["name"], "source": item["source"], "status": item["status"],
               "skipped": item.get("skipped", False)}
        if not item.get("skipped", False):
            row.update({
                "strict_feasible": item["strict_feasible"],
                "generalized_residual_inf": item["residual_norms"]["generalized_inf"],
                "aba_linear": item["aba_norms"]["linear"],
                "aba_angular": item["aba_norms"]["angular"],
                "aba_joint": item["aba_norms"]["joint"],
                "rollout_position_drift": item["rollout"]["max_position_error"],
                "rollout_rotation_drift": item["rollout"]["max_rotation_error"],
                "rollout_joint_drift": item["rollout"]["max_joint_position_error"],
                "minimum_rotor_margin": float(np.min(np.asarray(item["result"]["rotor_margins"]))),
                "minimum_joint_margin": float(np.min(np.asarray(item["result"]["joint_torque_margins"]))),
            })
        rows.append(row)
    fields = sorted({key for row in rows for key in row})
    with (output / "trim_summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)

    names = [item["name"] for item in evaluated]
    rotor_lower = [float(np.min(np.asarray(item["result"]["rotor_margins"])[:, 0])) for item in evaluated]
    rotor_upper = [float(np.min(np.asarray(item["result"]["rotor_margins"])[:, 1])) for item in evaluated]
    figure, axis = plt.subplots(figsize=(max(10, len(names) * 0.65), 5))
    x = np.arange(len(names)); width = 0.4
    axis.bar(x - width / 2, rotor_lower, width, label="minimum lower margin")
    axis.bar(x + width / 2, rotor_upper, width, label="minimum upper margin")
    axis.set(xticks=x, xticklabels=names, ylabel="rotor thrust margin [N]")
    axis.tick_params(axis="x", rotation=60); axis.legend(); figure.tight_layout()
    figure.savefig(output / "rotor_margins.png", dpi=160); plt.close(figure)

    joint_margins = np.asarray([item["result"]["joint_torque_margins"] for item in evaluated])
    figure, axis = plt.subplots(figsize=(max(10, len(names) * 0.65), 5))
    image = axis.imshow(joint_margins.T, aspect="auto", cmap="viridis")
    axis.set(xticks=np.arange(len(names)), xticklabels=names,
             yticks=np.arange(joint_margins.shape[1]),
             yticklabels=[f"joint {index + 1}" for index in range(joint_margins.shape[1])])
    axis.tick_params(axis="x", rotation=60)
    figure.colorbar(image, ax=axis, label="nearest torque margin [N m]")
    figure.tight_layout(); figure.savefig(output / "joint_torque_margins.png", dpi=160); plt.close(figure)

    figure, axis = plt.subplots(figsize=(max(10, len(names) * 0.65), 5))
    position = [item["rollout"]["max_position_error"] for item in evaluated]
    rotation = [item["rollout"]["max_rotation_error"] for item in evaluated]
    joints = [item["rollout"]["max_joint_position_error"] for item in evaluated]
    axis.semilogy(x, position, "o-", label="base position [m]")
    axis.semilogy(x, rotation, "s-", label="base rotation [rad]")
    axis.semilogy(x, joints, "^-", label="joint position norm [rad]")
    axis.set(xticks=x, xticklabels=names, ylabel="maximum 2 s rollout drift")
    axis.tick_params(axis="x", rotation=60); axis.legend(); axis.grid(True)
    figure.tight_layout(); figure.savefig(output / "rollout_drift.png", dpi=160); plt.close(figure)

    lines = [
        "# Static trim validation", "",
        f"Overall: **{'PASS' if summary['pass'] else 'FAIL'}**", "",
        f"- Strict feasible: `{summary['strict_feasible_configurations']} / {summary['total_configurations']}`",
        f"- Skipped invalid configurations: `{summary['skipped_configurations']}`",
        f"- Maximum generalized-force infinity residual: `{summary['maximum_generalized_force_residual']:.6e}`",
        f"- Maximum ABA base linear acceleration: `{summary['maximum_aba_linear_acceleration']:.6e} m/s^2`",
        f"- Maximum ABA base angular acceleration: `{summary['maximum_aba_angular_acceleration']:.6e} rad/s^2`",
        f"- Maximum ABA joint acceleration norm: `{summary['maximum_aba_joint_acceleration']:.6e} rad/s^2`",
        f"- Maximum two-second rollout position drift: `{summary['maximum_rollout_position_drift']:.6e} m`",
        f"- Minimum rotor thrust margin: `{summary['minimum_rotor_margin']:.6e} N`",
        f"- Minimum joint torque margin: `{summary['minimum_joint_torque_margin']:.6e} N m`", "",
        "All strict solutions were independently checked with Pinocchio ABA and rolled out through the canonical Crocoddyl prediction model. A strict result means the full 12-dimensional equality and bounds passed; approximate solutions would not be counted as strict.", "",
        "Static equality is not a stability guarantee. The longest rollout drift occurs at the undamped fully-extended equilibrium: numerical perturbations excite arm motion even though its initial RNEA residual and ABA acceleration pass strict tolerances.", "",
    ]
    p2 = summary.get("p2_static_trim", {})
    if p2:
        lines.extend([
            "## P2 static-trim reference mode", "",
            f"- Status: **{'PASS' if p2['pass'] else 'FAIL'}**",
            f"- Strict trim references: `{p2['strict_trim_nodes']} / {p2['nodes']}`",
            f"- BoxFDDP converged: `{p2['solver_converged']}` in `{p2['iterations']}` iterations",
            f"- Dynamics rollout state error: `{p2['rollout_state_error']:.6e}`", "",
        ])
    report_path.write_text("\n".join(lines), encoding="utf-8")
