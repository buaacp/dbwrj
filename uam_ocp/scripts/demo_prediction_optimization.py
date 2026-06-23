#!/usr/bin/env python3
"""Prove that BoxFDDP directly uses the canonical UAM prediction model."""

import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from uam_ocp.actuation import UamActuation
from uam_ocp.model_loader import load_uam_model
from uam_ocp.p2_planner import P2Planner
from uam_ocp.prediction_model import UAMPredictionModel
from uam_ocp.trajectory_io import save_trajectory
from uam_ocp.visualization import save_plots


def main() -> int:
    robot = load_uam_model()
    actuation = UamActuation(robot)
    prediction = UAMPredictionModel(robot, actuation)
    planner = P2Planner(robot, actuation, prediction_model=prediction)
    problem, scenario, _, _, trim = planner.build_problem("pregrasp")

    # Demonstrate the exact f_pred transition used by the running action model.
    p2_data = problem.runningModels[0].createData()
    problem.runningModels[0].calc(p2_data, problem.x0, trim)
    prediction_next = prediction.step(problem.x0, trim, float(scenario["dt_s"]))
    one_step_error = float(np.linalg.norm(
        robot.state.diff(np.asarray(p2_data.xnext), prediction_next)))

    solution = planner.solve("pregrasp")
    output = ROOT / "results" / "prediction_optimization"
    metrics = save_trajectory(robot, actuation, solution, output)
    save_plots(robot, actuation, solution, output)
    metadata = {
        "status": "PASS" if solution.converged and metrics["pass"] and one_step_error < 1e-10 else "FAIL",
        "prediction_equation": "x[k+1] = f_pred(x[k], u[k], dt)",
        "prediction_class": "uam_ocp.prediction_model.UAMPredictionModel",
        "action_model": "IntegratedActionModelEuler(DifferentialActionModelFreeFwdDynamics)",
        "solver": "SolverBoxFDDP",
        "dimensions": {"nq": prediction.nq, "nv": prediction.nv,
                       "nx": prediction.nx, "ndx": prediction.ndx, "nu": prediction.nu},
        "p2_prediction_single_step_error": one_step_error,
        "solver_converged": solution.converged,
        "solver_iterations": solution.iterations,
        "terminal_metrics": metrics,
    }
    (output / "prediction_optimization_summary.yaml").write_text(
        yaml.safe_dump(metadata, sort_keys=False), encoding="utf-8")
    print(f"f_pred/P2 one-step error: {one_step_error:.3e}")
    print(f"BoxFDDP converged={solution.converged}, iterations={solution.iterations}")
    print(f"output: {output}")
    return 0 if metadata["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())

