#!/usr/bin/env python3
import csv
import os
import sys

import numpy as np
import yaml

from nmpc_tracking.acados_controller import AcadosNmpcController
from nmpc_tracking.acados_model import (
    build_interface_dynamics,
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


def reference_state(config, layout):
    dims = controller_dimensions(config, layout)
    z = np.zeros(dims["state_dim"])
    z[2] = 1.0
    z[dims["idx_command"]] = hover_command(config, layout)
    return z


def errored_initial_state(config, layout, z_ref):
    dims = controller_dimensions(config, layout)
    z = z_ref.copy()
    z[0:3] += np.array([0.03, -0.02, 0.02])
    z[6] += np.deg2rad(5.0)
    z[dims["idx_joint_position"]] += 0.05
    return z


def constant_reference(config, z_ref):
    N = int(config["controller"]["horizon_steps"])
    return np.tile(z_ref.reshape(1, -1), (N + 1, 1))


def main():
    config = load_config()
    layout = load_robot_layout(config)
    os.makedirs(RESULTS, exist_ok=True)
    dims = controller_dimensions(config, layout)
    dt = float(config["controller"]["dt"])
    z_ref = reference_state(config, layout)
    z = errored_initial_state(config, layout, z_ref)
    ref_window = constant_reference(config, z_ref)

    controller = AcadosNmpcController(config, build_dir=os.path.join(ROOT, "build", "nmpc_acados"))
    controller.build()
    dynamics = build_interface_dynamics(config, layout)["f_expl"]
    lower_u, upper_u = command_bounds(config, layout)
    lower_du, upper_du = command_rate_bounds(config, layout)
    previous = None
    rows = []
    command_history = []
    command_rate_history = []

    for k in range(100):
        controller.set_reference(ref_window)
        controller.set_initial_state(z)
        controller.warm_start(previous)
        result = controller.solve()
        previous = result
        nu0 = result["predicted_command_rates"][0]
        cmd0 = result["first_command"]
        command_history.append(cmd0.copy())
        command_rate_history.append(nu0.copy())
        pos_error = float(np.linalg.norm(z[0:3] - z_ref[0:3]))
        att_error = float(np.linalg.norm(z[6:9] - z_ref[6:9]))
        joint_error = float(np.linalg.norm(z[dims["idx_joint_position"]] -
                                          z_ref[dims["idx_joint_position"]]))
        rows.append({
            "cycle": k,
            "status": int(result["status"]),
            "solve_time_s": float(result["solve_time_s"]),
            "position_error_m": pos_error,
            "attitude_error_rad": att_error,
            "joint_error_rad": joint_error,
            "cost": float(result["cost"]),
            "max_abs_command": float(np.max(np.abs(cmd0))),
            "max_abs_command_rate": float(np.max(np.abs(nu0))),
            "T_cmd": float(cmd0[0]),
            "p_cmd": float(cmd0[1]),
            "q_cmd": float(cmd0[2]),
            "r_cmd": float(cmd0[3]),
        })
        for index, value in enumerate(cmd0):
            rows[-1]["cmd_%02d" % index] = float(value)
        for index, value in enumerate(nu0):
            rows[-1]["cmd_rate_%02d" % index] = float(value)
        zdot = np.asarray(dynamics(z, nu0)).reshape(-1)
        z = z + dt * zdot

    times = np.asarray([r["solve_time_s"] for r in rows])
    status_ok = sum(1 for r in rows if r["status"] == 0)
    commands = np.asarray(command_history)
    command_rates = np.asarray(command_rate_history)
    command_ok = bool(np.all(commands >= lower_u - 1e-8) and
                      np.all(commands <= upper_u + 1e-8))
    rate_ok = bool(np.all(command_rates >= lower_du - 1e-8) and
                   np.all(command_rates <= upper_du + 1e-8))
    initial_errors = {
        "position_m": rows[0]["position_error_m"],
        "attitude_rad": rows[0]["attitude_error_rad"],
        "joint_rad": rows[0]["joint_error_rad"],
    }
    final_errors = {
        "position_m": rows[-1]["position_error_m"],
        "attitude_rad": rows[-1]["attitude_error_rad"],
        "joint_rad": rows[-1]["joint_error_rad"],
    }
    sigma = config["controller"]["sigma"]
    initial_norm = float(np.linalg.norm([
        initial_errors["position_m"] / float(sigma["position_m"]),
        initial_errors["attitude_rad"] / float(sigma["attitude_rad"]),
        initial_errors["joint_rad"] / float(sigma["joint_position_rad"]),
    ]))
    final_norm = float(np.linalg.norm([
        final_errors["position_m"] / float(sigma["position_m"]),
        final_errors["attitude_rad"] / float(sigma["attitude_rad"]),
        final_errors["joint_rad"] / float(sigma["joint_position_rad"]),
    ]))
    summary = {
        "cycles": len(rows),
        "status_zero_count": status_ok,
        "solver_build_count": controller.build_count,
        "mean_solve_time_s": float(np.mean(times)),
        "max_solve_time_s": float(np.max(times)),
        "p95_solve_time_s": float(np.percentile(times, 95.0)),
        "command_bounds_ok": command_ok,
        "command_rate_bounds_ok": rate_ok,
        "max_command": np.max(commands, axis=0),
        "min_command": np.min(commands, axis=0),
        "max_command_rate": np.max(command_rates, axis=0),
        "min_command_rate": np.min(command_rates, axis=0),
        "initial_errors": initial_errors,
        "final_errors": final_errors,
        "initial_normalized_error": initial_norm,
        "final_normalized_error": final_norm,
        "error_decreased": bool(final_norm < initial_norm),
    }

    csv_path = os.path.join(RESULTS, "warm_start_replay.csv")
    with open(csv_path, "w") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)
    np.savez_compressed(
        os.path.join(RESULTS, "warm_start_replay.npz"),
        rows=np.asarray(rows, dtype=object),
        solve_time_s=times,
        applied_commands=commands,
        applied_command_rates=command_rates,
    )
    with open(os.path.join(RESULTS, "warm_start_replay_summary.yaml"), "w") as f:
        yaml.safe_dump(as_plain(summary), f, sort_keys=False)

    save_plots(rows, RESULTS)

    print("status zero count:", status_ok, "/ 100")
    print("solver build count:", controller.build_count)
    print("solve time mean/max/p95 [s]: %.6f %.6f %.6f" %
          (summary["mean_solve_time_s"], summary["max_solve_time_s"], summary["p95_solve_time_s"]))
    print("command bounds ok:", command_ok)
    print("command-rate bounds ok:", rate_ok)
    print("initial errors:", initial_errors)
    print("final errors:", final_errors)
    print("normalized error initial/final: %.6f %.6f" % (initial_norm, final_norm))
    if status_ok != 100:
        raise SystemExit("not all solves succeeded")
    if controller.build_count != 1:
        raise SystemExit("solver was not built exactly once")
    if not command_ok or not rate_ok:
        raise SystemExit("bound check failed")
    if not summary["error_decreased"]:
        raise SystemExit("tracking errors did not decrease")
    return 0


def save_plots(rows, output_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.asarray([r["cycle"] for r in rows])
    pos = np.asarray([r["position_error_m"] for r in rows])
    att = np.asarray([r["attitude_error_rad"] for r in rows])
    joint = np.asarray([r["joint_error_rad"] for r in rows])
    solve = np.asarray([r["solve_time_s"] for r in rows])
    thrust = np.asarray([r["T_cmd"] for r in rows])
    rates = np.asarray([[r["p_cmd"], r["q_cmd"], r["r_cmd"]] for r in rows])

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, pos, label="position")
    ax.plot(t, att, label="attitude")
    ax.plot(t, joint, label="joint")
    ax.set_xlabel("cycle")
    ax.set_ylabel("error")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "tracking_error.png"), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, thrust, label="T")
    ax.plot(t, rates[:, 0], label="p")
    ax.plot(t, rates[:, 1], label="q")
    ax.plot(t, rates[:, 2], label="r")
    ax.set_xlabel("cycle")
    ax.set_ylabel("command")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "command_trajectory.png"), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t, solve)
    ax.set_xlabel("cycle")
    ax.set_ylabel("solve time [s]")
    ax.grid(True)
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "solve_time.png"), dpi=150)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(t[1:], np.diff(thrust), label="dT per cycle")
    ax.set_xlabel("cycle")
    ax.set_ylabel("command increment")
    ax.grid(True)
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(output_dir, "command_rate_trajectory.png"), dpi=150)
    plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
