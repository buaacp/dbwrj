#!/usr/bin/env python3
"""Run P2.6b local LQR stability and nonlinear recovery validation."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))

from uam_ocp.actuation import UamActuation
from uam_ocp.local_stability import (
    save_local_stability_results, validate_local_trim_stability)
from uam_ocp.model_loader import load_uam_model
from uam_ocp.prediction_model import UAMPredictionModel


def main() -> int:
    robot = load_uam_model()
    actuation = UamActuation(robot)
    prediction = UAMPredictionModel(robot, actuation)
    summary = validate_local_trim_stability(robot, actuation, prediction)
    save_local_stability_results(
        summary, ROOT / "results" / "local_trim_stability",
        PROJECT_ROOT / "docs" / "uam_ocp" / "LOCAL_TRIM_STABILITY_VALIDATION.md")
    selection = summary["selection"]
    print(f"Local trim stability: {'PASS' if summary['pass'] else 'FAIL'}")
    print(f"configurations: {selection['selected_names']}")
    print(f"worst joint margin: {selection['lowest_joint_torque_margin']}")
    print(f"worst rotor margin: {selection['lowest_rotor_margin']}")
    print(f"maximum closed spectral radius: {summary['maximum_closed_spectral_radius']:.6f}")
    print(f"maximum recovery time: {summary['maximum_recovery_time_s']}")
    print(f"maximum saturation ratio: {summary['maximum_saturation_ratio']:.6f}")
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())

