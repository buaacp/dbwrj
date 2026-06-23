#!/usr/bin/env python3
"""Solve and export the configured P2 pre-grasp trajectory."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from uam_ocp.actuation import UamActuation
from uam_ocp.model_loader import load_uam_model
from uam_ocp.p2_planner import P2Planner
from uam_ocp.trajectory_io import save_trajectory
from uam_ocp.visualization import save_plots


def main() -> int:
    robot = load_uam_model()
    actuation = UamActuation(robot)
    solution = P2Planner(robot, actuation).solve("pregrasp")
    output = ROOT / "results" / "p2_pregrasp"
    metrics = save_trajectory(robot, actuation, solution, output)
    save_plots(robot, actuation, solution, output)
    print(f"BoxFDDP converged={solution.converged} iterations={solution.iterations}")
    print(f"rollout error={solution.rollout_error:.3e}")
    print(f"terminal EE position error={metrics['terminal_ee_position_error_m']:.3e} m")
    print(f"terminal EE rotation error={metrics['terminal_ee_rotation_error_rad']:.3e} rad")
    print(f"results: {output}")
    print("P2:", "PASS" if metrics["pass"] else "FAIL")
    return 0 if metrics["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())

