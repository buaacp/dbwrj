#!/usr/bin/env python3
"""Run P2.7 whole-body pregrasp optimization over multiple offline scenarios."""

import argparse
import csv
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from uam_ocp.actuation import UamActuation
from uam_ocp.bulb_pregrasp import IKSeedUnreachable
from uam_ocp.bulb_pregrasp_planner import BulbPregraspPlanner
from uam_ocp.bulb_pregrasp_results import evaluate_solution, save_comparison, save_strategy
from uam_ocp.model_loader import load_uam_model
from uam_ocp.prediction_model import UAMPredictionModel


DEFAULT_SCENARIOS = ["nominal_pregrasp", "lateral_offset_pregrasp", "vertical_offset_pregrasp"]
STRATEGIES = ["arm_dominant", "uav_dominant", "whole_body"]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenarios", default="all",
                        help="'all' or comma-separated scenario names")
    args = parser.parse_args()
    scenarios = DEFAULT_SCENARIOS if args.scenarios == "all" else [
        item.strip() for item in args.scenarios.split(",") if item.strip()]
    output = ROOT / "results" / "p2_batch"
    output.mkdir(parents=True, exist_ok=True)
    robot = load_uam_model()
    actuation = UamActuation(robot)
    prediction = UAMPredictionModel(robot, actuation)
    rows = []
    for scenario_name in scenarios:
        scenario_dir = output / scenario_name
        scenario_dir.mkdir(parents=True, exist_ok=True)
        try:
            planner = BulbPregraspPlanner(robot, actuation, prediction, scenario_name)
        except IKSeedUnreachable as exc:
            rows.append({"scenario": scenario_name, "strategy": "", "status": "IK_SEED_UNREACHABLE",
                         "failure_reason": str(exc)})
            continue
        except Exception as exc:
            rows.append({"scenario": scenario_name, "strategy": "", "status": "FAIL",
                         "failure_reason": repr(exc)})
            continue
        solutions = []
        evaluations = {}
        for strategy in STRATEGIES:
            try:
                solution = planner.solve_strategy(strategy)
                metrics, arrays = evaluate_solution(robot, actuation, planner, solution)
                strategy_dir = scenario_dir / strategy
                save_strategy(robot, actuation, solution, metrics, arrays, strategy_dir)
                solutions.append(solution)
                evaluations[strategy] = (metrics, arrays)
                rows.append(_summary_row(scenario_name, solution.report_name, metrics, "PASS" if metrics["pass"] else "FAIL", ""))
            except Exception as exc:
                rows.append({"scenario": scenario_name, "strategy": strategy, "status": "FAIL",
                             "failure_reason": repr(exc)})
        if solutions:
            save_comparison(solutions, evaluations, scenario_dir)
    _write_summary(output, rows)
    passed = rows and all(row.get("status") == "PASS" for row in rows)
    print("P2 batch:", "PASS" if passed else "FAIL")
    print("results:", output)
    return 0 if passed else 1


def _summary_row(scenario, strategy, metrics, status, failure_reason):
    return {
        "scenario": scenario,
        "strategy": strategy,
        "status": status,
        "solver_converged": metrics.get("fddp_converged"),
        "iterations": metrics.get("iterations"),
        "final_cost": metrics.get("final_cost"),
        "rollout_error": metrics.get("rollout_error"),
        "terminal_position_error_m": metrics.get("terminal_position_error_m"),
        "terminal_orientation_error_rad": metrics.get("terminal_orientation_error_rad"),
        "terminal_base_linear_velocity_norm_mps": metrics.get("terminal_base_linear_velocity_norm_mps"),
        "terminal_base_angular_velocity_norm_radps": metrics.get("terminal_base_angular_velocity_norm_radps"),
        "terminal_max_arm_joint_velocity_radps": metrics.get("terminal_max_arm_joint_velocity_radps"),
        "terminal_ee_linear_velocity_mps": metrics.get("terminal_ee_linear_velocity_mps"),
        "terminal_ee_angular_velocity_radps": metrics.get("terminal_ee_angular_velocity_radps"),
        "terminal_rest_pass": metrics.get("terminal_rest_pass"),
        "control_bounds_satisfied": metrics.get("control_bounds_satisfied"),
        "minimum_rotor_margin_N": metrics.get("minimum_rotor_margin_N"),
        "minimum_joint_torque_margin_Nm": metrics.get("minimum_joint_torque_margin_Nm"),
        "failure_reason": failure_reason,
    }


def _write_summary(output: Path, rows):
    fields = [
        "scenario", "strategy", "status", "solver_converged", "iterations",
        "final_cost", "rollout_error", "terminal_position_error_m",
        "terminal_orientation_error_rad", "terminal_base_linear_velocity_norm_mps",
        "terminal_base_angular_velocity_norm_radps", "terminal_max_arm_joint_velocity_radps",
        "terminal_ee_linear_velocity_mps", "terminal_ee_angular_velocity_radps",
        "terminal_rest_pass", "control_bounds_satisfied", "minimum_rotor_margin_N",
        "minimum_joint_torque_margin_Nm", "failure_reason"]
    with (output / "scenario_comparison.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({field: row.get(field) for field in fields})
    (output / "scenario_comparison.yaml").write_text(
        yaml.safe_dump({"scenarios": rows}, sort_keys=False), encoding="utf-8")
    _write_summary_plots(output, rows)
    lines = ["# P2 batch feasibility report", ""]
    for row in rows:
        lines.append(f"- {row.get('scenario')} / {row.get('strategy')}: {row.get('status')} {row.get('failure_reason','')}")
    (output / "scenario_feasibility_report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_summary_plots(output: Path, rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    valid = [row for row in rows if row.get("strategy")]
    if not valid:
        return
    labels = [f"{row.get('scenario')}\n{row.get('strategy')}" for row in valid]
    x = range(len(valid))
    fig, ax = plt.subplots(figsize=(max(8, len(valid) * 1.2), 4))
    ax.bar(x, [float(row.get("terminal_position_error_m") or 0.0) for row in valid], label="position [m]")
    ax.bar(x, [float(row.get("terminal_orientation_error_rad") or 0.0) for row in valid], alpha=0.6, label="orientation [rad]")
    ax.set_xticks(list(x), labels, rotation=45, ha="right"); ax.legend(); fig.tight_layout()
    fig.savefig(output / "scenario_terminal_task_error_comparison.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(max(8, len(valid) * 1.2), 4))
    ax.plot(list(x), [float(row.get("terminal_base_linear_velocity_norm_mps") or 0.0) for row in valid], marker="o", label="base linear")
    ax.plot(list(x), [float(row.get("terminal_base_angular_velocity_norm_radps") or 0.0) for row in valid], marker="o", label="base angular")
    ax.plot(list(x), [float(row.get("terminal_max_arm_joint_velocity_radps") or 0.0) for row in valid], marker="o", label="arm max")
    ax.set_xticks(list(x), labels, rotation=45, ha="right"); ax.legend(); fig.tight_layout()
    fig.savefig(output / "scenario_terminal_stationarity_comparison.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(max(8, len(valid) * 1.2), 4))
    ax.semilogy(list(x), [max(float(row.get("final_cost") or 1.0), 1e-12) for row in valid], marker="o")
    ax.set_xticks(list(x), labels, rotation=45, ha="right"); ax.set_ylabel("final cost")
    fig.tight_layout(); fig.savefig(output / "scenario_cost_convergence_comparison.png", dpi=160); plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
