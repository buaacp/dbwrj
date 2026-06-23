#!/usr/bin/env python3
"""Run all f_pred validation tests and generate the required report."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))

from uam_ocp.actuation import UamActuation
from uam_ocp.model_loader import load_uam_model
from uam_ocp.p2_planner import P2Planner
from uam_ocp.prediction_model import UAMPredictionModel
from uam_ocp.prediction_validation import save_validation, validate_prediction_model


def main() -> int:
    robot = load_uam_model()
    actuation = UamActuation(robot)
    prediction = UAMPredictionModel(robot, actuation)
    planner = P2Planner(robot, actuation, prediction_model=prediction)
    summary = validate_prediction_model(prediction, planner)
    save_validation(
        summary, ROOT / "results" / "prediction_model",
        PROJECT_ROOT / "docs" / "uam_ocp" / "PREDICTION_MODEL_VALIDATION.md")
    print(f"Prediction model: {'PASS' if summary['pass'] else 'FAIL'}")
    print(f"single-step error: {summary['test_a_single_step']['max_error']:.3e}")
    print(f"rollout error: {summary['test_b_rollout']['max_error']:.3e}")
    print(f"linearization errors: {summary['test_c_linearization']['errors']}")
    print(f"hover drift: {summary['test_d_hover']['equal_mg_over_4_zero_arm_torque']['drift']}")
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())

