"""Non-interactive P2 diagnostic plots."""

from pathlib import Path

import numpy as np

from .actuation import UamActuation
from .model_loader import UamModel
from .p2_planner import P2Solution
from .trajectory_io import trajectory_samples


def save_plots(robot: UamModel, actuation: UamActuation,
               solution: P2Solution, output: Path) -> None:
    """Save 3D path, state/control diagnostics, and cost convergence."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    samples = trajectory_samples(robot, solution)
    time = np.asarray([item["time"] for item in samples])
    base = np.asarray([item["base_position"] for item in samples])
    ee = np.asarray([item["ee_position"] for item in samples])

    figure = plt.figure(figsize=(8, 6))
    axis = figure.add_subplot(111, projection="3d")
    axis.plot(base[:, 0], base[:, 1], base[:, 2], label="base")
    axis.plot(ee[:, 0], ee[:, 1], ee[:, 2], label="end effector")
    axis.scatter(*solution.target_pose.translation, marker="x", s=80, label="EE target")
    axis.set(xlabel="world x [m]", ylabel="world y [m]", zlabel="world z [m]")
    axis.legend()
    figure.tight_layout()
    figure.savefig(output / "trajectory_3d.png", dpi=160)
    plt.close(figure)

    figure, axes = plt.subplots(5, 2, figsize=(14, 18), sharex="col")
    axes = axes.ravel()
    axes[0].plot(time, base); axes[0].set_ylabel("base position [m]"); axes[0].legend(["x", "y", "z"])
    axes[1].plot(time, np.asarray([item["base_rpy"] for item in samples])); axes[1].set_ylabel("base RPY [rad]"); axes[1].legend(["roll", "pitch", "yaw"])
    axes[2].plot(time, np.asarray([item["base_linear_body"] for item in samples])); axes[2].set_ylabel("body linear v [m/s]")
    axes[3].plot(time, np.asarray([item["base_linear_world"] for item in samples])); axes[3].set_ylabel("world linear v [m/s]")
    axes[4].plot(time, solution.states[:, 7:7 + robot.n_arm]); axes[4].set_ylabel("arm joint q [rad]"); axes[4].legend(actuation.joint_names, fontsize=7)
    axes[5].plot(time, solution.states[:, robot.model.nq + 6:]); axes[5].set_ylabel("arm joint dq [rad/s]")
    control_time = time[:-1]
    axes[6].plot(control_time, solution.controls[:, :actuation.n_rotors]); axes[6].set_ylabel("rotor thrust [N]")
    axes[7].plot(control_time, solution.controls[:, actuation.n_rotors:]); axes[7].set_ylabel("joint torque [N m]")
    axes[8].plot(time, [item["ee_position_error"] for item in samples], label="position [m]"); axes[8].plot(time, [item["ee_rotation_error"] for item in samples], label="rotation [rad]"); axes[8].set_ylabel("EE error"); axes[8].legend()
    axes[9].plot(time, [np.linalg.norm(item["ee_linear_world"]) for item in samples], label="linear [m/s]"); axes[9].plot(time, [np.linalg.norm(item["ee_angular_world"]) for item in samples], label="angular [rad/s]"); axes[9].set_ylabel("EE speed"); axes[9].legend()
    for axis in axes[-2:]: axis.set_xlabel("time [s]")
    figure.tight_layout()
    figure.savefig(output / "state_control_timeseries.png", dpi=160)
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(7, 4))
    axis.semilogy(np.arange(len(solution.costs)), solution.costs, marker="o")
    axis.set(xlabel="BoxFDDP iteration", ylabel="cost", title="Cost convergence")
    axis.grid(True)
    figure.tight_layout()
    figure.savefig(output / "cost_convergence.png", dpi=160)
    plt.close(figure)

