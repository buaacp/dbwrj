"""Non-interactive P2 diagnostic plots."""

from pathlib import Path

import numpy as np

from .actuation import UamActuation
from .model_loader import UamModel
from .p2_planner import P2Solution
from .terminal_rest import terminal_rest_config
from .trajectory_io import trajectory_samples


def _title(solution: P2Solution, strategy: str = "P2") -> str:
    scenario = solution.scenario.get("scenario_name", "pregrasp")
    dt = float(solution.scenario.get("dt_s", solution.scenario.get("dt", 0.0)))
    horizon = len(solution.controls)
    mode = terminal_rest_config(solution.scenario)["enabled"]
    return f"{scenario} | {strategy} | N={horizon} dt={dt:g} | terminal_rest={mode}"


def save_plots(robot: UamModel, actuation: UamActuation,
               solution: P2Solution, output: Path, strategy: str = "P2") -> None:
    """Save separate trajectory, state, velocity, task, control, and cost plots."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output.mkdir(parents=True, exist_ok=True)
    samples = trajectory_samples(robot, solution)
    time = np.asarray([item["time"] for item in samples])
    control_time = time[:-1]
    base = np.asarray([item["base_position"] for item in samples])
    ee = np.asarray([item["ee_position"] for item in samples])
    rpy = np.asarray([item["base_rpy"] for item in samples])
    v_body = np.asarray([item["base_linear_body"] for item in samples])
    w_body = np.asarray([item["base_angular_body"] for item in samples])
    ee_v = np.asarray([item["ee_linear_world"] for item in samples])
    ee_w = np.asarray([item["ee_angular_world"] for item in samples])
    rest = terminal_rest_config(solution.scenario)
    window = int(rest["window_steps"])
    window_start = max(0, len(time) - 1 - window) * float(solution.scenario.get("dt_s", solution.scenario.get("dt", 0.0)))
    title = _title(solution, strategy)

    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(base[:, 0], base[:, 1], base[:, 2], label="UAV base")
    ax.plot(ee[:, 0], ee[:, 1], ee[:, 2], label="end effector")
    ax.scatter(base[0, 0], base[0, 1], base[0, 2], marker="o", label="base start")
    ax.scatter(base[-1, 0], base[-1, 1], base[-1, 2], marker="s", label="base end")
    ax.scatter(*solution.target_pose.translation, marker="x", s=80, label="EE target")
    ax.set(xlabel="world x [m]", ylabel="world y [m]", zlabel="world z [m]", title=title)
    ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(output / "uav_ee_trajectory_3d.png", dpi=160); fig.savefig(output / "trajectory_3d.png", dpi=160); plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(time, base); axes[0].set_ylabel("p_B [m]"); axes[0].legend(["x", "y", "z"])
    axes[1].plot(time, rpy); axes[1].set_ylabel("RPY [rad]"); axes[1].legend(["roll", "pitch", "yaw"]); axes[1].set_xlabel("time [s]")
    fig.suptitle(title); fig.tight_layout(); fig.savefig(output / "uav_state_trajectory.png", dpi=160); plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].plot(time, v_body); axes[0].plot(time, np.linalg.norm(v_body, axis=1), "k--", label="norm")
    axes[0].axhline(rest["base_linear_velocity_tolerance_mps"], color="r", ls=":", label="terminal tol")
    axes[0].set_ylabel("v_B^B [m/s]"); axes[0].legend(["vx", "vy", "vz", "norm", "terminal tol"], fontsize=8)
    axes[1].plot(time, w_body); axes[1].plot(time, np.linalg.norm(w_body, axis=1), "k--", label="norm")
    axes[1].axhline(rest["base_angular_velocity_tolerance_radps"], color="r", ls=":", label="terminal tol")
    axes[1].set_ylabel("omega_B^B [rad/s]"); axes[1].set_xlabel("time [s]")
    fig.suptitle(title); fig.tight_layout(); fig.savefig(output / "uav_velocity_angular_velocity.png", dpi=160); plt.close(fig)

    arm_q = solution.states[:, 7:7 + robot.n_arm]
    arm_v = solution.states[:, robot.model.nq + 6:]
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(time, arm_q); ax.set(xlabel="time [s]", ylabel="q_a [rad]", title=title)
    ax.legend(actuation.joint_names, fontsize=7); fig.tight_layout(); fig.savefig(output / "arm_joint_trajectory.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.plot(time, arm_v); ax.axhline(rest["arm_joint_velocity_tolerance_radps"], color="r", ls=":")
    ax.axhline(-rest["arm_joint_velocity_tolerance_radps"], color="r", ls=":")
    ax.set(xlabel="time [s]", ylabel="dq_a [rad/s]", title=title)
    ax.legend(actuation.joint_names, fontsize=7); fig.tight_layout(); fig.savefig(output / "arm_joint_velocity.png", dpi=160); plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)
    axes[0].semilogy(time, [item["ee_position_error"] for item in samples]); axes[0].set_ylabel("EE pos error [m]")
    axes[1].semilogy(time, [item["ee_rotation_error"] for item in samples]); axes[1].set_ylabel("EE rot error [rad]"); axes[1].set_xlabel("time [s]")
    fig.suptitle(title); fig.tight_layout(); fig.savefig(output / "task_error_convergence.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(9, 5))
    ax.plot(time, np.linalg.norm(v_body, axis=1), label="|v_B^B|")
    ax.plot(time, np.linalg.norm(w_body, axis=1), label="|omega_B^B|")
    ax.plot(time, np.max(np.abs(arm_v), axis=1), label="|dq_a|_inf")
    ax.plot(time, np.linalg.norm(ee_v, axis=1), label="|v_EE|")
    ax.plot(time, np.linalg.norm(ee_w, axis=1), label="|omega_EE|")
    ax.axvline(window_start, color="0.5", ls="--", label="rest window start")
    ax.axhline(rest["base_linear_velocity_tolerance_mps"], color="r", ls=":")
    ax.set(xlabel="time [s]", ylabel="speed / rate norm", title=title); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(output / "terminal_stationarity.png", dpi=160); plt.close(fig)

    lower, upper = actuation.control_bounds()
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    axes[0].plot(control_time, solution.controls[:, :actuation.n_rotors])
    for value in (lower[:actuation.n_rotors], upper[:actuation.n_rotors]):
        axes[0].plot(control_time, np.tile(value, (len(control_time), 1)), "k:", linewidth=0.7)
    axes[0].set_ylabel("rotor thrust [N]")
    axes[1].plot(control_time, solution.controls[:, actuation.n_rotors:])
    for value in (lower[actuation.n_rotors:], upper[actuation.n_rotors:]):
        axes[1].plot(control_time, np.tile(value, (len(control_time), 1)), "k:", linewidth=0.7)
    axes[1].set_ylabel("joint torque [N m]")
    axes[2].plot(control_time, np.sum(solution.controls[:, :actuation.n_rotors], axis=1))
    axes[2].set(xlabel="time [s]", ylabel="total thrust [N]")
    fig.suptitle(title); fig.tight_layout(); fig.savefig(output / "control_trajectory.png", dpi=160); plt.close(fig)

    fig, ax = plt.subplots(figsize=(7, 4))
    ax.semilogy(np.arange(len(solution.costs)), solution.costs, marker="o")
    if solution.costs:
        ax.scatter([0, len(solution.costs) - 1], [solution.costs[0], solution.costs[-1]], color="r", zorder=3)
    ax.set(xlabel="BoxFDDP iteration", ylabel="cost", title=title)
    ax.grid(True)
    fig.tight_layout(); fig.savefig(output / "optimization_cost_convergence.png", dpi=160); fig.savefig(output / "cost_convergence.png", dpi=160); plt.close(fig)

    # Backward-compatible combined figure for older scripts.
    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    axes[0, 0].plot(time, base); axes[0, 0].set_ylabel("base [m]")
    axes[0, 1].plot(time, rpy); axes[0, 1].set_ylabel("RPY [rad]")
    axes[1, 0].semilogy(time, [item["ee_position_error"] for item in samples], label="position")
    axes[1, 0].semilogy(time, [item["ee_rotation_error"] for item in samples], label="rotation"); axes[1, 0].legend()
    axes[1, 1].plot(control_time, solution.controls[:, :actuation.n_rotors]); axes[1, 1].set_ylabel("thrust [N]")
    fig.tight_layout(); fig.savefig(output / "state_control_timeseries.png", dpi=160); plt.close(fig)
