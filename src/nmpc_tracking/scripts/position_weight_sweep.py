#!/usr/bin/env python3
import copy
import csv
import os
import sys

import numpy as np
import yaml

from real_trajectory_replay_test import load_config, run_replay


ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
RESULTS = os.path.join(ROOT, "results", "nmpc_weight_sweep")


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


def case_config(base_config, qpm, qvm, qpfm):
    config = copy.deepcopy(base_config)
    config.setdefault("controller", {}).setdefault("weight_multipliers", {})
    config["controller"]["weight_multipliers"].update({
        "position": float(qpm),
        "velocity": float(qvm),
        "terminal_position": float(qpfm),
    })
    return config


def summarize_case(case_id, qpm, qvm, qpfm, summary, rows):
    command_keys = [k for k in rows[0] if k.startswith("cmd_") and not k.startswith("cmd_rate_")]
    rate_keys = [k for k in rows[0] if k.startswith("cmd_rate_")]
    commands = np.asarray([[r[k] for k in command_keys] for r in rows], dtype=float)
    rates = np.asarray([[r[k] for k in rate_keys] for r in rows], dtype=float)
    return {
        "case_id": case_id,
        "Qp_multiplier": qpm,
        "Qv_multiplier": qvm,
        "Qp_terminal_multiplier": qpfm,
        "initial_position_error_m": summary["initial_errors"]["e_p"],
        "final_position_error_m": summary["final_errors"]["e_p"],
        "maximum_position_error_m": summary["maximum_errors"]["e_p"],
        "final_attitude_error_rad": summary["final_errors"]["e_R"],
        "final_joint_error_rad": summary["final_errors"]["e_q"],
        "maximum_thrust_N": float(np.max(commands[:, 0])),
        "maximum_body_rate_radps": float(np.max(np.linalg.norm(commands[:, 1:4], axis=1))),
        "maximum_joint_velocity_radps": float(np.max(np.abs(commands[:, 4:]))),
        "maximum_command_rate": float(np.max(np.abs(rates))),
        "input_bound_ok": bool(summary["command_bounds_ok"]),
        "input_rate_bound_ok": bool(summary["command_rate_bounds_ok"]),
        "status_zero_count": int(summary["status_zero_count"]),
        "cycles": int(summary["cycles"]),
        "solver_build_count": int(summary["solver_build_count"]),
        "average_solve_time_s": float(summary["mean_solve_time_s"]),
        "maximum_solve_time_s": float(summary["max_solve_time_s"]),
        "p95_solve_time_s": float(summary["p95_solve_time_s"]),
        "final_position_error_less_than_initial": bool(summary["final_position_error_less_than_initial"]),
        "position_error_series": np.asarray([r["e_p"] for r in rows], dtype=float),
        "attitude_error_series": np.asarray([r["e_R"] for r in rows], dtype=float),
        "command_peak_series": np.max(np.abs(commands), axis=1),
        "solve_time_series": np.asarray([r["solve_time_s"] for r in rows], dtype=float),
    }


def save_plots(results, output_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plots = [
        ("position_error_series", "position_error_comparison.png", "position error [m]"),
        ("attitude_error_series", "attitude_error_comparison.png", "attitude error [rad]"),
        ("command_peak_series", "command_peak_comparison.png", "max abs command"),
        ("solve_time_series", "solve_time_comparison.png", "solve time [s]"),
    ]
    for key, filename, ylabel in plots:
        fig, ax = plt.subplots(figsize=(9, 5))
        for result in results:
            label = "case%d Qp%s Qv%s Qpf%s" % (
                result["case_id"], result["Qp_multiplier"],
                result["Qv_multiplier"], result["Qp_terminal_multiplier"])
            ax.plot(result[key], label=label)
        ax.grid(True)
        ax.legend(fontsize=6)
        ax.set_xlabel("cycle")
        ax.set_ylabel(ylabel)
        fig.tight_layout()
        fig.savefig(os.path.join(output_dir, filename), dpi=150)
        plt.close(fig)


def main():
    base_config = load_config()
    os.makedirs(RESULTS, exist_ok=True)
    # 最多 9 组：同时覆盖 Qp、Qv、Qp_terminal 的低/中/高组合，
    # 每组完整跑 planar4 trajectory + 5 秒 hold。
    combos = [
        (0, 1, 1, 3),
        (1, 1, 3, 3),
        (2, 1, 10, 10),
        (3, 3, 1, 3),
        (4, 3, 3, 10),
        (5, 3, 10, 10),
        (6, 10, 1, 3),
        (7, 10, 3, 10),
        (8, 10, 10, 10),
    ]
    results = []
    for case_id, qpm, qvm, qpfm in combos:
        print("running case", case_id, "Qp", qpm, "Qv", qvm, "Qp_terminal", qpfm)
        config = case_config(base_config, qpm, qvm, qpfm)
        summary, rows = run_replay(
            config,
            output_dir=os.path.join(RESULTS, "case_%02d" % case_id),
            build_dir=os.path.join(ROOT, "build", "nmpc_acados", "weight_sweep_%02d" % case_id),
            save_artifacts=False)
        results.append(summarize_case(case_id, qpm, qvm, qpfm, summary, rows))

    public_rows = [{k: v for k, v in result.items() if not k.endswith("_series")}
                   for result in results]
    eligible = [
        row for row in public_rows
        # 推荐参数必须真的让终端位置误差小于初始误差，且不依赖越界控制。
        if row["final_position_error_less_than_initial"]
        and row["input_bound_ok"]
        and row["input_rate_bound_ok"]
        and row["status_zero_count"] == row["cycles"]
        and row["solver_build_count"] == 1
    ]
    preferred = [row for row in eligible if row["final_position_error_m"] < 0.03]
    pool = preferred if preferred else eligible
    recommended = min(pool, key=lambda row: row["final_position_error_m"]) if pool else None

    with open(os.path.join(RESULTS, "weight_sweep_summary.csv"), "w") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(public_rows[0].keys()))
        writer.writeheader()
        writer.writerows(public_rows)
    with open(os.path.join(RESULTS, "weight_sweep_summary.yaml"), "w") as stream:
        yaml.safe_dump(as_plain({
            "cases": public_rows,
            "recommended": recommended,
            "selection_rule": "min final position error among full-trajectory cases with final<initial, all status=0, one build, and no bound violations; prefer final<0.03m",
        }), stream, sort_keys=False)
    save_plots(results, RESULTS)
    print("completed cases:", len(results))
    print("recommended:", recommended)
    if recommended is None:
        raise SystemExit("no weight case satisfied final_position_error < initial_position_error without bound violations")
    return 0


if __name__ == "__main__":
    sys.exit(main())
