"""Scenario selection and nonlinear disturbance recovery validation."""

import csv
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import yaml

from .actuation import UamActuation
from .local_lqr import LQRDesign, LocalTrimLQR
from .model_loader import MODULE_ROOT, PROJECT_ROOT, UamModel, load_yaml
from .prediction_model import UAMPredictionModel
from .static_trim import StaticTrimSolver


def build_lqr_weights(config: Dict[str, Any], robot: UamModel,
                      actuation: UamActuation) -> Tuple[np.ndarray, np.ndarray]:
    """Build positive-definite tangent-state and physical-control weights."""
    weights = config["lqr_weights"]
    q_diagonal = np.concatenate((
        np.full(3, float(weights["q_position"])),
        np.full(3, float(weights["q_rotation"])),
        np.full(robot.n_arm, float(weights["q_arm_position"])),
        np.full(3, float(weights["q_linear_velocity"])),
        np.full(3, float(weights["q_angular_velocity"])),
        np.full(robot.n_arm, float(weights["q_arm_velocity"])),
    ))
    r_diagonal = np.concatenate((
        np.full(actuation.n_rotors, float(weights["r_rotor_thrust"])),
        np.full(robot.n_arm, float(weights["r_arm_torque"])),
    ))
    return np.diag(q_diagonal), np.diag(r_diagonal)


def select_stability_configurations(robot: UamModel, actuation: UamActuation,
                                    config: Dict[str, Any]) -> Tuple[List[Dict[str, Any]], Dict[str, Any]]:
    """Select required and automatically detected minimum-margin trim points."""
    source = PROJECT_ROOT / config["source_static_trim_results"]
    trim_summary = load_yaml(source)
    entries = [item for item in trim_summary["entries"] if item.get("strict_feasible", False)]
    by_name = {item["name"]: item for item in entries}
    if not entries:
        raise RuntimeError("No strict static-trim entries available")

    def rotor_minimum(item: Dict[str, Any]) -> float:
        return float(np.min(np.asarray(item["result"]["rotor_margins"], dtype=float)))

    def joint_minimum(item: Dict[str, Any]) -> float:
        return float(np.min(np.asarray(item["result"]["joint_torque_margins"], dtype=float)))

    worst_rotor = min(entries, key=rotor_minimum)
    worst_joint = min(entries, key=joint_minimum)
    rotor_margins = np.asarray(worst_rotor["result"]["rotor_margins"], dtype=float)
    rotor_index, rotor_side_index = np.unravel_index(np.argmin(rotor_margins), rotor_margins.shape)
    joint_margins = np.asarray(worst_joint["result"]["joint_torque_margins"], dtype=float)
    joint_index = int(np.argmin(joint_margins))

    names = list(config["selection"]["required"])
    if bool(config["selection"].get("include_lowest_rotor_margin", True)):
        names.append(worst_rotor["name"])
    if bool(config["selection"].get("include_lowest_joint_torque_margin", True)):
        names.append(worst_joint["name"])
    unique_names = list(dict.fromkeys(names))
    missing = [name for name in unique_names if name not in by_name]
    if missing:
        raise KeyError(f"Selected trim configurations are absent: {missing}")
    selected = [by_name[name] for name in unique_names]
    selection = {
        "source": str(source), "selected_names": unique_names,
        "lowest_rotor_margin": {
            "configuration": worst_rotor["name"], "rotor_index": int(rotor_index),
            "rotor_id": int(actuation.rotors[rotor_index]["id"]),
            "rotor_name": actuation.rotors[rotor_index]["name"],
            "bound_side": "lower" if rotor_side_index == 0 else "upper",
            "margin_N": float(rotor_margins[rotor_index, rotor_side_index]),
        },
        "lowest_joint_torque_margin": {
            "configuration": worst_joint["name"], "joint_index_zero_based": joint_index,
            "joint_number_one_based": joint_index + 1,
            "joint_name": actuation.joint_names[joint_index],
            "margin_Nm": float(joint_margins[joint_index]),
        },
    }
    return selected, selection


def build_disturbances(robot: UamModel, x_eq: np.ndarray,
                       config: Dict[str, Any]) -> Dict[str, np.ndarray]:
    """Build safe manifold tangent perturbations without crossing joint limits."""
    disturbances = config["disturbances"]
    pose = np.zeros(robot.state.ndx)
    pose[:3] = np.asarray(disturbances["body_pose"]["base_position_m"], dtype=float)
    pose[3:6] = np.deg2rad(disturbances["body_pose"]["base_rotation_deg"])

    arm = np.zeros(robot.state.ndx)
    request = np.deg2rad(float(disturbances["arm_configuration"]["requested_joint_delta_deg"]))
    signs = np.asarray(disturbances["arm_configuration"]["alternating_signs"], dtype=float)
    safety = float(disturbances["arm_configuration"]["limit_safety_fraction"])
    q_eq = x_eq[:robot.model.nq]
    for offset, joint in enumerate(robot.arm_joints):
        sign = float(signs[offset])
        current = float(q_eq[joint.idx_q])
        lower = float(robot.model.lowerPositionLimit[joint.idx_q])
        upper = float(robot.model.upperPositionLimit[joint.idx_q])
        available = upper - current if sign > 0.0 else current - lower
        magnitude = min(request, max(0.0, safety * available))
        arm[joint.idx_v] = sign * magnitude

    velocity = np.zeros(robot.state.ndx)
    velocity[robot.model.nv:robot.model.nv + 3] = np.asarray(
        disturbances["velocity"]["base_linear_mps"], dtype=float)
    velocity[robot.model.nv + 3:robot.model.nv + 6] = np.asarray(
        disturbances["velocity"]["base_angular_radps"], dtype=float)
    velocity_signs = np.asarray(disturbances["velocity"]["alternating_signs"], dtype=float)
    velocity[robot.model.nv + 6:] = np.deg2rad(
        float(disturbances["velocity"]["joint_velocity_degps"])) * velocity_signs
    combined = float(disturbances["combined"]["scale"]) * (pose + arm + velocity)
    return {
        "body_pose": pose, "arm_configuration": arm,
        "velocity": velocity, "combined": combined,
    }


def error_curves(rollout: Dict[str, np.ndarray], robot: UamModel) -> Dict[str, np.ndarray]:
    """Extract manifold-consistent grouped error curves."""
    tangent = np.asarray(rollout["tangent_errors"], dtype=float)
    return {
        "position": np.linalg.norm(tangent[:, :3], axis=1),
        "rotation": np.linalg.norm(tangent[:, 3:6], axis=1),
        "arm_position": np.linalg.norm(tangent[:, 6:6 + robot.n_arm], axis=1),
        "velocity": np.linalg.norm(tangent[:, robot.model.nv:], axis=1),
    }


def recovery_metrics(curves: Dict[str, np.ndarray], rollout: Dict[str, np.ndarray],
                     u_eq: np.ndarray, config: Dict[str, Any],
                     actuation: UamActuation) -> Dict[str, Any]:
    """Compute error extrema, sustained recovery, saturation, and actuator use."""
    thresholds = config["recovery_thresholds"]
    limits = {
        "position": float(thresholds["position_m"]),
        "rotation": np.deg2rad(float(thresholds["rotation_deg"])),
        "arm_position": np.deg2rad(float(thresholds["arm_position_deg"])),
        "velocity": float(thresholds["velocity_norm"]),
    }
    length = min(len(value) for value in curves.values())
    inside = np.ones(length, dtype=bool)
    for name, values in curves.items():
        inside &= np.isfinite(values[:length]) & (values[:length] < limits[name])
    recovery_index = None
    for index in range(length):
        if np.all(inside[index:]):
            recovery_index = index
            break
    dt = float(config["rollout"]["dt_s"])
    recovery_time = None if recovery_index is None else recovery_index * dt
    controls = np.asarray(rollout["controls"], dtype=float)
    saturation = np.asarray(rollout["saturation"], dtype=bool)
    saturated_steps = np.any(saturation, axis=1) if saturation.size else np.zeros(0, dtype=bool)
    saturation_ratio = float(np.mean(saturated_steps)) if saturated_steps.size else 0.0
    lower, upper = actuation.control_bounds()
    rotor = controls[:, :actuation.n_rotors] if controls.size else np.empty((0, actuation.n_rotors))
    joint = controls[:, actuation.n_rotors:] if controls.size else np.empty((0, actuation.robot.n_arm))
    rotor_occupancy = rotor / upper[:actuation.n_rotors] if rotor.size else rotor
    joint_denominator = np.maximum(
        np.abs(lower[actuation.n_rotors:]), np.abs(upper[actuation.n_rotors:]))
    joint_occupancy = np.abs(joint) / joint_denominator if joint.size else joint
    finite = bool(all(np.all(np.isfinite(values)) for values in curves.values())
                  and np.all(np.isfinite(controls)))
    recovered = bool(recovery_time is not None and finite and saturation_ratio <= float(
        config["rollout"]["maximum_saturation_ratio"]))
    if recovered:
        failure_reason = None
    elif not finite:
        failure_reason = "NONFINITE_ROLLOUT"
    elif saturation_ratio > float(config["rollout"]["maximum_saturation_ratio"]):
        failure_reason = "FREQUENT_CONTROL_SATURATION"
    else:
        failure_reason = "ERROR_THRESHOLDS_NOT_REACHED"
    grouped = {}
    for name, values in curves.items():
        grouped[name] = {
            "initial": float(values[0]), "peak": float(np.nanmax(values)),
            "terminal": float(values[-1]), "threshold": limits[name],
        }
    return {
        "status": "RECOVERED" if recovered else "NOT_RECOVERED",
        "recovered": recovered, "finite": finite,
        "failure_reason": failure_reason,
        "recovery_time_s": recovery_time,
        "errors": grouped,
        "saturated_steps": int(np.count_nonzero(saturated_steps)),
        "saturation_ratio": saturation_ratio,
        "saturated_channels": int(np.count_nonzero(saturation)),
        "rotor_minimum_N": np.min(rotor, axis=0).tolist() if rotor.size else [],
        "rotor_maximum_N": np.max(rotor, axis=0).tolist() if rotor.size else [],
        "joint_maximum_absolute_torque_Nm": np.max(np.abs(joint), axis=0).tolist() if joint.size else [],
        "maximum_rotor_occupancy": float(np.max(rotor_occupancy)) if rotor_occupancy.size else 0.0,
        "maximum_joint_torque_occupancy": float(np.max(joint_occupancy)) if joint_occupancy.size else 0.0,
        "maximum_control_deviation": float(np.max(np.linalg.norm(controls - u_eq, axis=1))) if controls.size else 0.0,
    }


def validate_local_trim_stability(robot: UamModel, actuation: UamActuation,
                                  prediction: UAMPredictionModel,
                                  config_path: Optional[Path] = None) -> Dict[str, Any]:
    """Run open/closed nonlinear recovery tests at automatically selected trims."""
    config = load_yaml(config_path or MODULE_ROOT / "config" / "local_stability_scenarios.yaml")
    selected, selection = select_stability_configurations(robot, actuation, config)
    Q, R = build_lqr_weights(config, robot, actuation)
    trim_solver = StaticTrimSolver(robot, actuation, prediction_model=prediction)
    linear = config["linearization"]
    regulator = LocalTrimLQR(
        prediction, rank_relative_tolerance=float(linear["rank_relative_tolerance"]),
        eigenvalue_tolerance=float(linear["eigenvalue_tolerance"]))
    dt = float(linear["dt_s"])
    duration = float(config["rollout"]["duration_s"])
    cases = []
    scenario_summaries = []
    designs: Dict[str, LQRDesign] = {}
    for entry in selected:
        q = np.asarray(entry["result"]["q"], dtype=float)
        trim = trim_solver.solve_trim(q)
        if not trim.strict_feasible:
            raise RuntimeError(f"Selected configuration {entry['name']} lost strict trim: {trim.status}")
        aba_max = float(np.max(np.abs(trim.aba_acceleration)))
        if aba_max >= 1e-7:
            raise RuntimeError(f"Selected configuration {entry['name']} ABA residual {aba_max:.3e}")
        x_eq = np.concatenate((q, np.zeros(robot.model.nv)))
        design = regulator.design(x_eq, trim.u_eq, dt, Q, R)
        designs[entry["name"]] = design
        open_magnitudes = np.abs(design.open_eigenvalues)
        closed_magnitudes = np.abs(design.closed_eigenvalues)
        eigen_tolerance = float(linear["eigenvalue_tolerance"])
        scenario_summary = {
            "name": entry["name"], "q": q.tolist(), "u_eq": trim.u_eq.tolist(),
            "aba_acceleration_max": aba_max,
            "open_spectral_radius": float(np.max(open_magnitudes)),
            "closed_spectral_radius": float(np.max(closed_magnitudes)),
            "open_unstable_modes": int(np.count_nonzero(open_magnitudes > 1.0 + eigen_tolerance)),
            "closed_unstable_modes": int(np.count_nonzero(closed_magnitudes > 1.0 + eigen_tolerance)),
            "controllability": design.diagnostics,
            "open_eigenvalues": [[float(value.real), float(value.imag)] for value in design.open_eigenvalues],
            "closed_eigenvalues": [[float(value.real), float(value.imag)] for value in design.closed_eigenvalues],
        }
        disturbances = build_disturbances(robot, x_eq, config)
        scenario_cases = []
        for case_name, tangent in disturbances.items():
            x0 = robot.state.integrate(x_eq, tangent)
            open_rollout = regulator.rollout_open_loop(
                x0, x_eq, trim.u_eq, duration, dt)
            closed_rollout = regulator.rollout_closed_loop(
                x0, x_eq, trim.u_eq, design.K, duration, dt)
            open_curves = error_curves(open_rollout, robot)
            closed_curves = error_curves(closed_rollout, robot)
            open_metrics = recovery_metrics(
                open_curves, open_rollout, trim.u_eq, config, actuation)
            closed_metrics = recovery_metrics(
                closed_curves, closed_rollout, trim.u_eq, config, actuation)
            case = {
                "scenario": entry["name"], "case": case_name,
                "open": open_metrics, "closed": closed_metrics,
                "open_curves": {key: value.tolist() for key, value in open_curves.items()},
                "closed_curves": {key: value.tolist() for key, value in closed_curves.items()},
                "initial_tangent": tangent.tolist(),
                "open_rollout": open_rollout, "closed_rollout": closed_rollout,
            }
            cases.append(case); scenario_cases.append(case)
        scenario_summary["cases"] = [{
            "case": item["case"], "open": item["open"], "closed": item["closed"]
        } for item in scenario_cases]
        scenario_summary["all_closed_recovered"] = all(
            item["closed"]["recovered"] for item in scenario_cases)
        scenario_summaries.append(scenario_summary)

    maximum_closed_radius = max(item["closed_spectral_radius"] for item in scenario_summaries)
    recovered_times = [item["closed"]["recovery_time_s"] for item in cases
                       if item["closed"]["recovery_time_s"] is not None]
    summary = {
        "pass": bool(all(item["all_closed_recovered"] for item in scenario_summaries)
                     and maximum_closed_radius < 1.0
                     and all(item["controllability"]["stabilizable"] for item in scenario_summaries)),
        "config": config, "selection": selection,
        "scenarios": scenario_summaries,
        "maximum_closed_spectral_radius": maximum_closed_radius,
        "maximum_recovery_time_s": max(recovered_times) if recovered_times else None,
        "maximum_rotor_occupancy": max(item["closed"]["maximum_rotor_occupancy"] for item in cases),
        "maximum_joint_torque_occupancy": max(item["closed"]["maximum_joint_torque_occupancy"] for item in cases),
        "maximum_saturation_ratio": max(item["closed"]["saturation_ratio"] for item in cases),
        "case_count": len(cases),
    }
    summary["_runtime_cases"] = cases
    summary["_runtime_designs"] = designs
    return summary


def _serializable(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {key: _serializable(item) for key, item in value.items()
                if not key.startswith("_runtime")}
    if isinstance(value, (list, tuple)):
        return [_serializable(item) for item in value]
    return value


def save_local_stability_results(summary: Dict[str, Any], output: Path,
                                 report_path: Path) -> None:
    """Save trajectories, metrics, eigenvalue/recovery plots, and validation report."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    plots_dir = output / "plots"
    plots_dir.mkdir(parents=True, exist_ok=True)
    cases = summary["_runtime_cases"]
    designs = summary["_runtime_designs"]
    clean_summary = _serializable(summary)
    (output / "summary.yaml").write_text(
        yaml.safe_dump(clean_summary, sort_keys=False), encoding="utf-8")

    rows = []
    scenario_by_name = {item["name"]: item for item in summary["scenarios"]}
    for item in cases:
        scenario = scenario_by_name[item["scenario"]]
        row = {
            "scenario": item["scenario"], "case": item["case"],
            "open_spectral_radius": scenario["open_spectral_radius"],
            "closed_spectral_radius": scenario["closed_spectral_radius"],
            "controllability_rank": scenario["controllability"]["controllability_rank"],
            "stabilizable": scenario["controllability"]["stabilizable"],
            "open_recovered": item["open"]["recovered"],
            "closed_recovered": item["closed"]["recovered"],
            "closed_recovery_time_s": item["closed"]["recovery_time_s"],
            "closed_saturation_ratio": item["closed"]["saturation_ratio"],
            "closed_max_rotor_occupancy": item["closed"]["maximum_rotor_occupancy"],
            "closed_max_joint_occupancy": item["closed"]["maximum_joint_torque_occupancy"],
        }
        for group in ("position", "rotation", "arm_position", "velocity"):
            row[f"open_{group}_terminal"] = item["open"]["errors"][group]["terminal"]
            row[f"closed_{group}_terminal"] = item["closed"]["errors"][group]["terminal"]
            row[f"closed_{group}_peak"] = item["closed"]["errors"][group]["peak"]
        rows.append(row)
    fields = list(rows[0])
    with (output / "summary.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerows(rows)

    dt = float(summary["config"]["rollout"]["dt_s"])
    for item in cases:
        name = f"scenario_{item['scenario']}_{item['case']}"
        open_rollout = item["open_rollout"]
        closed_rollout = item["closed_rollout"]
        np.savez_compressed(
            output / f"{name}.npz",
            states_open=open_rollout["states"], controls_open=open_rollout["controls"],
            states_closed=closed_rollout["states"], controls_closed=closed_rollout["controls"],
            raw_controls_closed=closed_rollout["raw_controls"],
            saturation_closed=closed_rollout["saturation"],
            tangent_open=open_rollout["tangent_errors"],
            tangent_closed=closed_rollout["tangent_errors"],
            open_spectral_radius=scenario_by_name[item["scenario"]]["open_spectral_radius"],
            closed_spectral_radius=scenario_by_name[item["scenario"]]["closed_spectral_radius"],
            recovery_time_closed=np.nan if item["closed"]["recovery_time_s"] is None
            else item["closed"]["recovery_time_s"])
        _plot_case(item, dt, plots_dir / f"{name}.png")

    unit_circle = np.exp(1j * np.linspace(0.0, 2.0 * np.pi, 400))
    for closed, filename, title in (
            (False, "eigenvalues_open_loop.png", "Open-loop eigenvalues"),
            (True, "eigenvalues_closed_loop.png", "Closed-loop eigenvalues")):
        figure, axis = plt.subplots(figsize=(7, 7))
        axis.plot(unit_circle.real, unit_circle.imag, "k--", linewidth=1, label="unit circle")
        for name, design in designs.items():
            values = design.closed_eigenvalues if closed else design.open_eigenvalues
            axis.scatter(values.real, values.imag, s=25, label=name)
        axis.set(xlabel="real", ylabel="imaginary", title=title, aspect="equal")
        axis.grid(True); axis.legend(fontsize=8); figure.tight_layout()
        figure.savefig(output / filename, dpi=160); plt.close(figure)

    combined = {(item["scenario"], item["case"]): item for item in cases}
    if ("fully_extended", "combined") in combined:
        _plot_case(combined[("fully_extended", "combined")], dt,
                   output / "fully_extended_open_vs_closed.png")
    worst_joint = summary["selection"]["lowest_joint_torque_margin"]["configuration"]
    if (worst_joint, "combined") in combined:
        _plot_case(combined[(worst_joint, "combined")], dt,
                   output / "worst_joint_margin_open_vs_closed.png")

    labels = [f"{item['scenario']}:{item['case']}" for item in cases]
    ratios = [item["closed"]["saturation_ratio"] for item in cases]
    figure, axis = plt.subplots(figsize=(max(12, len(labels) * 0.55), 5))
    axis.bar(np.arange(len(labels)), ratios)
    axis.axhline(float(summary["config"]["rollout"]["maximum_saturation_ratio"]),
                 color="r", linestyle="--", label="failure threshold")
    axis.set(xticks=np.arange(len(labels)), xticklabels=labels,
             ylabel="closed-loop saturated-step ratio")
    axis.tick_params(axis="x", rotation=70); axis.legend(); figure.tight_layout()
    figure.savefig(output / "control_saturation.png", dpi=160); plt.close(figure)

    report_path.parent.mkdir(parents=True, exist_ok=True)
    selection = summary["selection"]
    lines = [
        "# Local trim stability validation", "",
        f"Overall: **{'PASS' if summary['pass'] else 'FAIL'}**", "",
        f"Selected configurations: `{selection['selected_names']}`", "",
        "## Automatic margin selection", "",
        f"- Lowest rotor margin: `{selection['lowest_rotor_margin']['margin_N']:.6e} N` at `{selection['lowest_rotor_margin']['configuration']}`, rotor `{selection['lowest_rotor_margin']['rotor_id']}` (`{selection['lowest_rotor_margin']['rotor_name']}`), `{selection['lowest_rotor_margin']['bound_side']}` side.",
        f"- Lowest joint torque margin: `{selection['lowest_joint_torque_margin']['margin_Nm']:.6e} N m` at `{selection['lowest_joint_torque_margin']['configuration']}`, joint {selection['lowest_joint_torque_margin']['joint_number_one_based']} `{selection['lowest_joint_torque_margin']['joint_name']}`.", "",
        "## Spectral and nonlinear recovery results", "",
        "| Configuration | rho open | rho closed | Open unstable | Closed unstable | Controllability rank | Stabilizable | Closed cases recovered |",
        "|---|---:|---:|---:|---:|---:|---|---:|",
    ]
    for scenario in summary["scenarios"]:
        recovered = sum(item["closed"]["recovered"] for item in scenario["cases"])
        lines.append(
            f"| {scenario['name']} | {scenario['open_spectral_radius']:.6f} | "
            f"{scenario['closed_spectral_radius']:.6f} | {scenario['open_unstable_modes']} | "
            f"{scenario['closed_unstable_modes']} | "
            f"{scenario['controllability']['controllability_rank']}/24 | "
            f"{scenario['controllability']['stabilizable']} | {recovered}/4 |")
    lines.extend(["", "## Aggregate", "",
                  f"- Maximum closed-loop spectral radius: `{summary['maximum_closed_spectral_radius']:.6e}`",
                  f"- Maximum sustained recovery time: `{summary['maximum_recovery_time_s']:.6e} s`",
                  f"- Maximum rotor thrust occupancy: `{summary['maximum_rotor_occupancy']:.6e}`",
                  f"- Maximum joint torque occupancy: `{summary['maximum_joint_torque_occupancy']:.6e}`",
                  f"- Maximum saturated-step ratio: `{summary['maximum_saturation_ratio']:.6e}`", "",
                  "Open-loop recovery is expected to fail for position offsets and unstable arm modes because a fixed equilibrium input contains no restoring feedback. Closed-loop PASS means recovery only for the tested nominal-model local perturbations.", ""])
    report_path.write_text("\n".join(lines), encoding="utf-8")


def _plot_case(item: Dict[str, Any], dt: float, path: Path) -> None:
    import matplotlib.pyplot as plt
    figure, axes = plt.subplots(2, 2, figsize=(11, 8))
    labels = {
        "position": "base position error [m]",
        "rotation": "base rotation error [rad]",
        "arm_position": "arm position error [rad]",
        "velocity": "velocity tangent norm",
    }
    for axis, group in zip(axes.ravel(), labels):
        open_values = np.asarray(item["open_curves"][group], dtype=float)
        closed_values = np.asarray(item["closed_curves"][group], dtype=float)
        threshold = float(item["closed"]["errors"][group]["threshold"])
        open_normalized = np.clip(open_values / threshold, 1e-8, 1e6)
        closed_normalized = np.clip(closed_values / threshold, 1e-8, 1e6)
        axis.semilogy(np.arange(len(open_values)) * dt, open_normalized, label="open loop")
        axis.semilogy(np.arange(len(closed_values)) * dt, closed_normalized, label="LQR closed loop")
        axis.axhline(1.0, color="k", linestyle="--", linewidth=1,
                     label="recovery threshold")
        axis.set(xlabel="time [s]", ylabel=labels[group] + " / threshold")
        axis.grid(True); axis.legend(fontsize=8)
    figure.suptitle(f"{item['scenario']} / {item['case']}")
    figure.tight_layout(); figure.savefig(path, dpi=160); plt.close(figure)
