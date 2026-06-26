#!/usr/bin/env python3
import os
import sys

import numpy as np
import yaml

from nmpc_tracking.acados_controller import AcadosNmpcController
from nmpc_tracking.acados_model import (
    command_bounds,
    command_rate_bounds,
    controller_dimensions,
    hover_command,
)
from nmpc_tracking.robot_layout import load_robot_layout


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CONFIG = os.path.join(ROOT, "src", "nmpc_tracking", "config", "dual_mpc_pregrasp.yaml")
RESULTS = os.path.join(ROOT, "results", "nmpc_smoke")


def load_config():
    with open(CONFIG, "r") as f:
        return yaml.safe_load(f)


def hover_state(config, layout, z_height=1.0):
    dims = controller_dimensions(config, layout)
    z = np.zeros(dims["state_dim"])
    z[2] = float(z_height)
    z[dims["idx_command"]] = hover_command(config, layout)
    return z


def constant_reference(config, z_ref):
    N = int(config["controller"]["horizon_steps"])
    return np.tile(np.asarray(z_ref, dtype=float).reshape(1, -1), (N + 1, 1))


def as_plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, (np.floating, np.integer)):
        return value.item()
    if isinstance(value, dict):
        return {k: as_plain(v) for k, v in value.items()}
    if isinstance(value, list):
        return [as_plain(v) for v in value]
    return value


def main():
    config = load_config()
    layout = load_robot_layout(config)
    os.makedirs(RESULTS, exist_ok=True)
    controller = AcadosNmpcController(config, build_dir=os.path.join(ROOT, "build", "nmpc_acados"))
    controller.build()

    z_ref = hover_state(config, layout, z_height=1.0)
    z0 = z_ref.copy()
    controller.set_reference(constant_reference(config, z_ref))
    controller.set_initial_state(z0)
    result = controller.solve()

    lower_u, upper_u = command_bounds(config, layout)
    lower_du, upper_du = command_rate_bounds(config, layout)
    first_command = result["first_command"]
    first_rate = result["predicted_command_rates"][0]
    command_bounds_ok = bool(np.all(first_command >= lower_u - 1e-8) and
                             np.all(first_command <= upper_u + 1e-8))
    rate_bounds_ok = bool(np.all(first_rate >= lower_du - 1e-8) and
                          np.all(first_rate <= upper_du + 1e-8))
    hover = hover_command(config, layout)[0]
    report = {
        "solver_status": int(result["status"]),
        "solve_time_s": float(result["solve_time_s"]),
        "hover_thrust_reference_n": float(hover),
        "first_optimal_command": first_command,
        "first_optimal_command_rate": first_rate,
        "command_bounds_ok": command_bounds_ok,
        "command_rate_bounds_ok": rate_bounds_ok,
        "command_lower": lower_u,
        "command_upper": upper_u,
        "command_rate_lower": lower_du,
        "command_rate_upper": upper_du,
        "cost": float(result["cost"]),
        "build_count": int(controller.build_count),
        "model_mass_kg": float(layout.total_mass_kg),
        "effective_mass_kg": float(layout.effective_mass_kg),
        "mass_override_kg": layout.mass_override_kg,
    }
    with open(os.path.join(RESULTS, "one_step_summary.yaml"), "w") as f:
        yaml.safe_dump(as_plain(report), f, sort_keys=False)

    print("solver status:", result["status"])
    print("solve time [s]: %.6f" % result["solve_time_s"])
    print("hover thrust reference [N]: %.6f" % hover)
    print("first optimal command:", first_command)
    print("first optimal command-rate:", first_rate)
    print("command bounds ok:", command_bounds_ok)
    print("command-rate bounds ok:", rate_bounds_ok)

    if result["status"] != 0:
        raise SystemExit("solver failed with status %d" % result["status"])
    if abs(first_command[0] - hover) > 0.5:
        raise SystemExit("hover thrust check failed")
    if np.linalg.norm(first_command[1:4]) > 1e-2:
        raise SystemExit("body-rate command is not near zero")
    if np.linalg.norm(first_command[4:]) > 1e-2:
        raise SystemExit("arm velocity command is not near zero")
    if not command_bounds_ok or not rate_bounds_ok:
        raise SystemExit("bound check failed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
