#!/usr/bin/env python3
"""Run P2.6 configuration-dependent static-trim validation."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))

from uam_ocp.actuation import UamActuation
from uam_ocp.model_loader import load_uam_model
from uam_ocp.prediction_model import UAMPredictionModel
from uam_ocp.static_trim import StaticTrimSolver
from uam_ocp.trim_validation import (
    save_trim_validation, validate_p2_static_trim_mode, validate_trim_scenarios)


def main() -> int:
    robot = load_uam_model()
    actuation = UamActuation(robot)
    prediction = UAMPredictionModel(robot, actuation)
    solver = StaticTrimSolver(robot, actuation, prediction_model=prediction)
    summary = validate_trim_scenarios(robot, actuation, solver)
    summary["p2_static_trim"] = validate_p2_static_trim_mode(
        robot, actuation, prediction)
    summary["pass"] = bool(summary["pass"] and summary["p2_static_trim"]["pass"])
    save_trim_validation(
        summary, ROOT / "results" / "static_trim",
        PROJECT_ROOT / "docs" / "uam_ocp" / "STATIC_TRIM_VALIDATION.md")
    print(f"Static trim: {'PASS' if summary['pass'] else 'FAIL'}")
    print(f"strict feasible: {summary['strict_feasible_configurations']} / {summary['total_configurations']}")
    print(f"max generalized residual: {summary['maximum_generalized_force_residual']:.3e}")
    print(f"max ABA norms: linear={summary['maximum_aba_linear_acceleration']:.3e}, "
          f"angular={summary['maximum_aba_angular_acceleration']:.3e}, "
          f"joint={summary['maximum_aba_joint_acceleration']:.3e}")
    print(f"max rollout position drift: {summary['maximum_rollout_position_drift']:.3e}")
    print(f"P2 static_trim: {'PASS' if summary['p2_static_trim']['pass'] else 'FAIL'}")
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())
