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
)
from nmpc_tracking.robot_layout import load_robot_layout
from nmpc_tracking.trajectory_reference import TrajectoryReference


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
CONFIG = os.path.join(ROOT, "src", "nmpc_tracking", "config", "dual_mpc_pregrasp.yaml")
RESULTS = os.path.join(ROOT, "results", "nmpc_real_replay")


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


def z_from_ref(ref, config, layout):
    dims = controller_dimensions(config, layout)
    z = np.zeros(dims["state_dim"])
    z[dims["idx_position"]] = ref.position_w
    z[dims["idx_velocity"]] = ref.velocity_w
    z[dims["idx_attitude"]] = ref.euler_rpy
    z[dims["idx_body_rate"]] = ref.body_rate
    z[dims["idx_joint_position"]] = ref.joint_position
    z[dims["idx_joint_velocity"]] = ref.joint_velocity
    z[dims["idx_command"]] = ref.command
    return z


def reference_window(traj, t, config, layout):
    dims = controller_dimensions(config, layout)
    N = int(config["controller"]["horizon_steps"])
    dt = float(config["controller"]["dt"])
    return np.asarray([z_from_ref(traj.sample(t + k * dt), config, layout)
                       for k in range(N + 1)]).reshape(N + 1, dims["state_dim"])


def component_errors(z, zref, dims, joint_names=None):
    """返回整体误差和每个 planar4 关节的独立位置/速度误差。"""
    errors = {
        "e_p": float(np.linalg.norm(z[dims["idx_position"]] - zref[dims["idx_position"]])),
        "e_v": float(np.linalg.norm(z[dims["idx_velocity"]] - zref[dims["idx_velocity"]])),
        "e_R": float(np.linalg.norm(z[dims["idx_attitude"]] - zref[dims["idx_attitude"]])),
        "e_omega": float(np.linalg.norm(z[dims["idx_body_rate"]] - zref[dims["idx_body_rate"]])),
        "e_q": float(np.linalg.norm(z[dims["idx_joint_position"]] - zref[dims["idx_joint_position"]])),
        "e_dq": float(np.linalg.norm(z[dims["idx_joint_velocity"]] - zref[dims["idx_joint_velocity"]])),
    }
    joint_position_error = z[dims["idx_joint_position"]] - zref[dims["idx_joint_position"]]
    joint_velocity_error = z[dims["idx_joint_velocity"]] - zref[dims["idx_joint_velocity"]]
    for i, name in enumerate(joint_names or []):
        key = name.replace("_joint", "")
        errors["e_q_" + key] = float(joint_position_error[i])
        errors["abs_e_q_" + key] = float(abs(joint_position_error[i]))
        errors["e_dq_" + key] = float(joint_velocity_error[i])
        errors["abs_e_dq_" + key] = float(abs(joint_velocity_error[i]))
    return errors


def cost_components(z, zref, nu, config, dims):
    sigma = config["controller"]["sigma"]
    comp = component_errors(z, zref, dims)
    du = z[dims["idx_command"]] - zref[dims["idx_command"]]
    return {
        "J_p": comp["e_p"] ** 2 / float(sigma["position_m"]) ** 2,
        "J_v": comp["e_v"] ** 2 / float(sigma["velocity_mps"]) ** 2,
        "J_R": comp["e_R"] ** 2 / float(sigma["attitude_rad"]) ** 2,
        "J_omega": comp["e_omega"] ** 2 / float(sigma["body_rate_radps"]) ** 2,
        "J_q": comp["e_q"] ** 2 / float(sigma["joint_position_rad"]) ** 2,
        "J_dq": comp["e_dq"] ** 2 / float(sigma["joint_velocity_radps"]) ** 2,
        "J_u": float(np.sum(du ** 2) / float(sigma["command"]) ** 2),
        "J_nu": float(np.sum(nu ** 2) / float(sigma["command_rate"]) ** 2),
    }


def verify_node_interpolation(traj):
    max_pos = 0.0
    max_q = 0.0
    for i, t in enumerate(traj.times):
        ref = traj.sample(t)
        state = traj.snapshot.states[i]
        max_pos = max(max_pos, float(np.max(np.abs(ref.position_w - state[:3]))))
        max_q = max(max_q, float(np.max(np.abs(ref.joint_position - state[7:len(traj.snapshot.q_names)]))))
    return {"max_position_node_error": max_pos, "max_joint_node_error": max_q}


def perturbed_initial_state(z, dims):
    # 为了检验闭环收敛，replay 从固定扰动开始，而不是从零误差初值开始。
    z0 = np.asarray(z, dtype=float).copy()
    z0[dims["idx_position"]] += np.array([0.03, -0.02, 0.02])
    z0[dims["idx_attitude"].start] += np.deg2rad(5.0)
    z0[dims["idx_joint_position"]] += 0.05
    return z0


def run_replay(config, output_dir=RESULTS, build_dir=None, save_artifacts=True):
    """执行完整预抓取轨迹 + 终端保持的无 ROS 闭环 replay。"""
    layout = load_robot_layout(config)
    traj = TrajectoryReference.from_npz(
        config["trajectory"]["offline_npz"],
        hold_after_s=float(config["trajectory"].get("hold_after_s", 5.0)))
    os.makedirs(output_dir, exist_ok=True)
    dims = controller_dimensions(config, layout)
    dt = float(config["controller"]["dt"])
    total_time = traj.snapshot.duration + float(config["trajectory"].get("hold_after_s", 5.0))
    steps = int(round(total_time / dt))

    controller = AcadosNmpcController(
        config,
        build_dir=build_dir or os.path.join(ROOT, "build", "nmpc_acados", "real_replay"))
    controller.build()
    dynamics = build_interface_dynamics(config, layout)["f_expl"]
    lower_u, upper_u = command_bounds(config, layout)
    lower_du, upper_du = command_rate_bounds(config, layout)

    z = perturbed_initial_state(z_from_ref(traj.sample(0.0), config, layout), dims)
    previous = None
    rows = []
    commands = []
    rates = []
    status = []
    states = []

    for k in range(steps):
        t = k * dt
        # 每个控制周期重新从 TrajectoryReference 取 NMPC horizon 内的参考窗口。
        zref = reference_window(traj, t, config, layout)
        controller.set_reference(zref)
        controller.set_initial_state(z)
        controller.warm_start(previous)
        result = controller.solve()
        previous = result
        nu0 = result["predicted_command_rates"][0]
        cmd0 = result["first_command"]
        err = component_errors(z, zref[0], dims, layout.arm_joint_names)
        costs = cost_components(z, zref[0], nu0, config, dims)
        row = {
            "cycle": k,
            "time_s": t,
            "status": int(result["status"]),
            "solve_time_s": float(result["solve_time_s"]),
            "cost": float(result["cost"]),
        }
        row.update(err)
        row.update(costs)
        for i, value in enumerate(cmd0):
            row["cmd_%02d" % i] = float(value)
        for i, value in enumerate(nu0):
            row["cmd_rate_%02d" % i] = float(value)
        rows.append(row)
        commands.append(cmd0.copy())
        rates.append(nu0.copy())
        status.append(int(result["status"]))
        states.append(z.copy())
        # 被控对象也用同一个接口级模型推进；这仍是无 ROS 数值 replay。
        z = z + dt * np.asarray(dynamics(z, nu0)).reshape(-1)

    commands = np.asarray(commands)
    rates = np.asarray(rates)
    states = np.asarray(states)
    times = np.asarray([r["solve_time_s"] for r in rows])
    status = np.asarray(status)
    ref_joints = traj.snapshot.joint_position_reference
    joint_ref_ranges = np.ptp(ref_joints, axis=0)
    joint_cmd_ranges = np.ptp(commands[:, 4:], axis=0)
    command_ok = bool(np.all(commands >= lower_u - 1e-8) and np.all(commands <= upper_u + 1e-8))
    rate_ok = bool(np.all(rates >= lower_du - 1e-8) and np.all(rates <= upper_du + 1e-8))
    node_check = verify_node_interpolation(traj)
    position_final_less_initial = bool(rows[-1]["e_p"] < rows[0]["e_p"])
    summary = {
        "cycles": int(steps),
        "status_zero_count": int(np.sum(status == 0)),
        "solver_build_count": int(controller.build_count),
        "mean_solve_time_s": float(np.mean(times)),
        "max_solve_time_s": float(np.max(times)),
        "p95_solve_time_s": float(np.percentile(times, 95.0)),
        "command_bounds_ok": command_ok,
        "command_rate_bounds_ok": rate_ok,
        "joint_reference_ranges": joint_ref_ranges,
        "joint_command_ranges": joint_cmd_ranges,
        "joint_references_all_different": bool(np.all(joint_ref_ranges > 1e-9)),
        "joint_commands_all_different": bool(np.all(joint_cmd_ranges > 1e-9)),
        "initial_errors": {k: rows[0][k] for k in ["e_p", "e_v", "e_R", "e_omega", "e_q", "e_dq"]},
        "maximum_errors": {k: float(max(r[k] for r in rows)) for k in ["e_p", "e_v", "e_R", "e_omega", "e_q", "e_dq"]},
        "final_errors": {k: rows[-1][k] for k in ["e_p", "e_v", "e_R", "e_omega", "e_q", "e_dq"]},
        "initial_joint_abs_errors": {
            name: abs(rows[0]["e_q_" + name.replace("_joint", "")])
            for name in layout.arm_joint_names
        },
        "final_joint_abs_errors": {
            name: abs(rows[-1]["e_q_" + name.replace("_joint", "")])
            for name in layout.arm_joint_names
        },
        "maximum_joint_abs_errors": {
            name: max(r["abs_e_q_" + name.replace("_joint", "")] for r in rows)
            for name in layout.arm_joint_names
        },
        "final_position_error_less_than_initial": position_final_less_initial,
        "node_interpolation": node_check,
    }
    if save_artifacts:
        with open(os.path.join(output_dir, "summary.yaml"), "w") as f:
            yaml.safe_dump(as_plain(summary), f, sort_keys=False)
        with open(os.path.join(output_dir, "replay.csv"), "w") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)
        np.savez_compressed(os.path.join(output_dir, "replay.npz"),
                            rows=np.asarray(rows, dtype=object),
                            states=states, commands=commands, command_rates=rates)
        save_plots(rows, commands, rates, states, traj, config, layout, output_dir)

    print("status zero count:", summary["status_zero_count"], "/", steps)
    print("solver build count:", controller.build_count)
    print("solve time mean/max/p95 [s]: %.6f %.6f %.6f" %
          (summary["mean_solve_time_s"], summary["max_solve_time_s"], summary["p95_solve_time_s"]))
    print("command bounds ok:", command_ok)
    print("command-rate bounds ok:", rate_ok)
    print("initial/final position error:", summary["initial_errors"]["e_p"], summary["final_errors"]["e_p"])
    print("final position error < initial:", position_final_less_initial)
    if summary["status_zero_count"] != steps:
        raise SystemExit("not all solves succeeded")
    if controller.build_count != 1:
        raise SystemExit("solver was not built exactly once")
    if not command_ok or not rate_ok:
        raise SystemExit("bound check failed")
    return summary, rows


def main():
    config = load_config()
    summary, _ = run_replay(config, RESULTS)
    if not summary["final_position_error_less_than_initial"]:
        raise SystemExit("terminal position error is not smaller than initial position error")
    return 0


def save_plots(rows, commands, rates, states, traj, config, layout, output_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    t = np.asarray([r["time_s"] for r in rows])
    keys = ["e_p", "e_v", "e_R", "e_omega", "e_q", "e_dq"]
    fig, ax = plt.subplots(figsize=(9, 5))
    for key in keys:
        ax.plot(t, [r[key] for r in rows], label=key)
    ax.grid(True); ax.legend(); ax.set_xlabel("time [s]"); ax.set_ylabel("error")
    fig.tight_layout(); fig.savefig(os.path.join(output_dir, "tracking_error_components.png"), dpi=150); plt.close(fig)

    sigma = config["controller"]["sigma"]
    norm = np.sqrt(
        (np.asarray([r["e_p"] for r in rows]) / float(sigma["position_m"])) ** 2 +
        (np.asarray([r["e_R"] for r in rows]) / float(sigma["attitude_rad"])) ** 2 +
        (np.asarray([r["e_q"] for r in rows]) / float(sigma["joint_position_rad"])) ** 2)
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, norm); ax.grid(True); ax.set_xlabel("time [s]"); ax.set_ylabel("normalized error")
    fig.tight_layout(); fig.savefig(os.path.join(output_dir, "normalized_tracking_error.png"), dpi=150); plt.close(fig)

    dims = controller_dimensions(config, layout)
    ref_pos = np.asarray([z_from_ref(traj.sample(time), config, layout)[dims["idx_position"]] for time in t])
    fig, ax = plt.subplots(figsize=(9, 5))
    for i, name in enumerate(["x", "y", "z"]):
        ax.plot(t, states[:, i], label=name)
        ax.plot(t, ref_pos[:, i], "--", label=name + "_ref")
    ax.grid(True); ax.legend(ncol=2); ax.set_xlabel("time [s]"); ax.set_ylabel("position [m]")
    fig.tight_layout(); fig.savefig(os.path.join(output_dir, "planner_reference_vs_nmpc_state.png"), dpi=150); plt.close(fig)

    qa = states[:, dims["idx_joint_position"]]
    ref_qa = np.asarray([z_from_ref(traj.sample(time), config, layout)[dims["idx_joint_position"]] for time in t])
    fig, ax = plt.subplots(figsize=(9, 5))
    for i, name in enumerate(layout.arm_joint_names):
        ax.plot(t, qa[:, i], label=name)
        ax.plot(t, ref_qa[:, i], "--", lw=0.8)
    ax.grid(True); ax.legend(fontsize=7); ax.set_xlabel("time [s]"); ax.set_ylabel("joint [rad]")
    fig.tight_layout(); fig.savefig(os.path.join(output_dir, "joint_tracking.png"), dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t, commands[:, 0], label="T")
    ax.plot(t, commands[:, 1:4], label=["p", "q", "r"])
    ax.grid(True); ax.legend(); ax.set_xlabel("time [s]"); ax.set_ylabel("command")
    fig.tight_layout(); fig.savefig(os.path.join(output_dir, "command_trajectory.png"), dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(t, rates)
    ax.grid(True); ax.set_xlabel("time [s]"); ax.set_ylabel("command rate")
    fig.tight_layout(); fig.savefig(os.path.join(output_dir, "command_rate_trajectory.png"), dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 5))
    for key in ["J_p", "J_v", "J_R", "J_omega", "J_q", "J_dq", "J_u", "J_nu"]:
        ax.plot(t, [r[key] for r in rows], label=key)
    ax.grid(True); ax.legend(fontsize=7); ax.set_xlabel("time [s]"); ax.set_ylabel("cost component")
    fig.tight_layout(); fig.savefig(os.path.join(output_dir, "cost_components.png"), dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 4))
    ax.plot(t, [r["solve_time_s"] for r in rows])
    ax.grid(True); ax.set_xlabel("time [s]"); ax.set_ylabel("solve time [s]")
    fig.tight_layout(); fig.savefig(os.path.join(output_dir, "solve_time.png"), dpi=150); plt.close(fig)


if __name__ == "__main__":
    sys.exit(main())
