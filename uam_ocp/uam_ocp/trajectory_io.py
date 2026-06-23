"""Trajectory metrics and deterministic CSV/NPZ/YAML export."""

import csv
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import pinocchio as pin
import yaml

from .actuation import UamActuation
from .model_loader import UamModel
from .p2_planner import P2Solution


def trajectory_samples(robot: UamModel, solution: P2Solution) -> List[Dict[str, Any]]:
    """Evaluate base and end-effector kinematics at every state."""
    samples: List[Dict[str, Any]] = []
    dt = float(solution.scenario["dt_s"])
    for index, state in enumerate(solution.states):
        q = state[:robot.model.nq]
        v = state[robot.model.nq:]
        data = robot.model.createData()
        pin.forwardKinematics(robot.model, data, q, v)
        pin.updateFramePlacements(robot.model, data)
        ee = data.oMf[robot.end_effector_frame_id]
        ee_velocity = pin.getFrameVelocity(
            robot.model, data, robot.end_effector_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        base_rotation = pin.Quaternion(q[3:7]).matrix()
        body_linear = v[:3]
        body_angular = v[3:6]
        samples.append({
            "time": index * dt, "q": q.copy(), "v": v.copy(),
            "base_position": q[:3].copy(), "base_rpy": pin.rpy.matrixToRpy(base_rotation),
            "base_linear_body": body_linear.copy(), "base_angular_body": body_angular.copy(),
            "base_linear_world": base_rotation @ body_linear,
            "base_angular_world": base_rotation @ body_angular,
            "ee_position": ee.translation.copy(), "ee_rotation": ee.rotation.copy(),
            "ee_linear_world": ee_velocity.linear.copy(),
            "ee_angular_world": ee_velocity.angular.copy(),
            "ee_position_error": float(np.linalg.norm(ee.translation - solution.target_pose.translation)),
            "ee_rotation_error": float(np.linalg.norm(
                pin.log3(solution.target_pose.rotation.T @ ee.rotation))),
        })
    return samples


def terminal_metrics(robot: UamModel, actuation: UamActuation,
                     solution: P2Solution, samples: List[Dict[str, Any]]) -> Dict[str, Any]:
    """Evaluate P2 acceptance metrics and configured thresholds."""
    final = samples[-1]
    lower, upper = actuation.control_bounds()
    minimum_margin = np.min(solution.controls - lower, axis=0)
    maximum_margin = np.min(upper - solution.controls, axis=0)
    tolerance = solution.scenario["tolerances"]
    metrics = {
        "solver_converged": solution.converged,
        "iterations": solution.iterations,
        "final_cost": solution.costs[-1] if solution.costs else None,
        "rollout_state_error": solution.rollout_error,
        "terminal_ee_position_error_m": final["ee_position_error"],
        "terminal_ee_rotation_error_rad": final["ee_rotation_error"],
        "terminal_ee_linear_velocity_mps": float(np.linalg.norm(final["ee_linear_world"])),
        "terminal_ee_angular_velocity_radps": float(np.linalg.norm(final["ee_angular_world"])),
        "terminal_base_rpy_rad": final["base_rpy"].tolist(),
        "control_minimum": solution.controls.min(axis=0).tolist(),
        "control_maximum": solution.controls.max(axis=0).tolist(),
        "minimum_lower_bound_margin": minimum_margin.tolist(),
        "minimum_upper_bound_margin": maximum_margin.tolist(),
        "control_bounds_satisfied": bool(
            np.all(solution.controls >= lower - 1e-10)
            and np.all(solution.controls <= upper + 1e-10)),
    }
    checks = {
        "solver": solution.converged,
        "rollout": solution.rollout_error <= float(tolerance["rollout_state"]),
        "control_bounds": metrics["control_bounds_satisfied"],
        "ee_position": final["ee_position_error"] <= float(tolerance["ee_position_m"]),
        "ee_rotation": final["ee_rotation_error"] <= float(tolerance["ee_rotation_rad"]),
        "ee_linear_velocity": metrics["terminal_ee_linear_velocity_mps"] <= float(tolerance["ee_linear_velocity_mps"]),
        "ee_angular_velocity": metrics["terminal_ee_angular_velocity_radps"] <= float(tolerance["ee_angular_velocity_radps"]),
        "base_attitude": bool(np.max(np.abs(final["base_rpy"][:2])) < np.deg2rad(30.0)),
    }
    metrics["checks"] = checks
    metrics["pass"] = all(checks.values())
    return metrics


def _write_csv(path: Path, fieldnames: List[str], rows: List[Dict[str, Any]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def save_trajectory(robot: UamModel, actuation: UamActuation,
                    solution: P2Solution, output: Path) -> Dict[str, Any]:
    """Save all required P2 numeric artifacts and terminal report."""
    output.mkdir(parents=True, exist_ok=True)
    samples = trajectory_samples(robot, solution)
    np.savez_compressed(
        output / "trajectory.npz", states=solution.states, controls=solution.controls,
        solver_states=solution.solver_states, costs=np.asarray(solution.costs),
        target_translation=solution.target_pose.translation,
        target_rotation=solution.target_pose.rotation)

    q_names = ["base_x", "base_y", "base_z", "base_qx", "base_qy", "base_qz", "base_qw"]
    q_names.extend(joint.name for joint in robot.arm_joints)
    v_names = ["base_vx_body", "base_vy_body", "base_vz_body",
               "base_wx_body", "base_wy_body", "base_wz_body"]
    v_names.extend(joint.name + "_velocity" for joint in robot.arm_joints)
    state_rows = []
    for sample in samples:
        row = {"time_s": sample["time"]}
        row.update(zip(q_names, sample["q"]))
        row.update(zip(v_names, sample["v"]))
        state_rows.append(row)
    _write_csv(output / "states.csv", ["time_s"] + q_names + v_names, state_rows)

    control_names = [f"rotor_{item['id']}_thrust_N" for item in actuation.rotors]
    control_names.extend(name + "_torque_Nm" for name in actuation.joint_names)
    control_rows = []
    for index, control in enumerate(solution.controls):
        row = {"time_s": index * float(solution.scenario["dt_s"])}
        row.update(zip(control_names, control))
        control_rows.append(row)
    _write_csv(output / "controls.csv", ["time_s"] + control_names, control_rows)

    ee_fields = ["time_s", "x", "y", "z", "qx", "qy", "qz", "qw",
                 "vx_world", "vy_world", "vz_world", "wx_world", "wy_world", "wz_world",
                 "position_error_m", "rotation_error_rad"]
    ee_rows = []
    for sample in samples:
        quaternion = pin.Quaternion(sample["ee_rotation"]).coeffs()
        values = np.concatenate((sample["ee_position"], quaternion,
                                 sample["ee_linear_world"], sample["ee_angular_world"]))
        row = dict(zip(ee_fields[1:14], values))
        row.update(time_s=sample["time"], position_error_m=sample["ee_position_error"],
                   rotation_error_rad=sample["ee_rotation_error"])
        ee_rows.append(row)
    _write_csv(output / "ee_pose.csv", ee_fields, ee_rows)

    metrics = terminal_metrics(robot, actuation, solution, samples)
    summary = {
        "status": "PASS" if metrics["pass"] else "FAIL",
        "architecture": "StateMultibody -> verified ActuationModelFloatingBaseThrusters -> DifferentialActionModelFreeFwdDynamics -> IntegratedActionModelEuler -> ShootingProblem -> SolverBoxFDDP",
        "scenario": solution.scenario,
        "dimensions": {"nq": robot.model.nq, "nv": robot.model.nv,
                       "nx": robot.state.nx, "ndx": robot.state.ndx,
                       "nu": actuation.nu, "nodes": len(solution.controls)},
        "metrics": metrics,
        "delta_u": solution.scenario["delta_u"],
        "constraint_note": "Control bounds are hard BoxFDDP bounds; joint position, joint velocity, and base tilt use quadratic-barrier soft costs and are not strict guarantees.",
    }
    (output / "optimization_summary.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    report = [
        "# P2 terminal error report", "",
        f"- Status: {'PASS' if metrics['pass'] else 'FAIL'}",
        f"- Solver converged: {metrics['solver_converged']}",
        f"- Iterations: {metrics['iterations']}",
        f"- Rollout state error: {metrics['rollout_state_error']:.6e}",
        f"- EE position error: {metrics['terminal_ee_position_error_m']:.6e} m",
        f"- EE rotation error: {metrics['terminal_ee_rotation_error_rad']:.6e} rad",
        f"- EE linear speed: {metrics['terminal_ee_linear_velocity_mps']:.6e} m/s",
        f"- EE angular speed: {metrics['terminal_ee_angular_velocity_radps']:.6e} rad/s",
        f"- Control bounds satisfied: {metrics['control_bounds_satisfied']}",
        "- Joint position, joint velocity, and base tilt constraints: quadratic-barrier soft costs only.",
        "- Delta-u term: P2.1 pending; no claim of implementation.", "",
    ]
    (output / "terminal_error_report.md").write_text("\n".join(report), encoding="utf-8")
    return metrics
