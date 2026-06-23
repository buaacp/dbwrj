#!/usr/bin/env python3
"""Run and persist the P0 floating-base model validation."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from uam_ocp.model_loader import load_uam_model
from uam_ocp.model_validation import save_summary, validate_model


def main() -> int:
    robot = load_uam_model()
    summary = validate_model(robot)
    output = ROOT / "results" / "p0"
    save_summary(summary, output)
    print(f"P0 model: nq={summary['nq']} nv={summary['nv']} n_arm={summary['n_arm']}")
    print(f"total mass: {summary['total_mass_kg']:.6f} kg")
    print(f"EE neutral: {summary['neutral']['ee_position_world']}")
    print(f"checks: {summary['checks']}")
    print("P0:", "PASS" if summary["pass"] else "FAIL")
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())

