#!/usr/bin/env python3
"""Run hover, single-rotor and arm-reaction P1 validation cases."""

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from uam_ocp.actuation import UamActuation
from uam_ocp.dynamics import generalized_acceleration
from uam_ocp.model_loader import load_uam_model


def main() -> int:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output = ROOT / "results" / "p1"
    output.mkdir(parents=True, exist_ok=True)
    robot = load_uam_model()
    actuation = UamActuation(robot)
    native = actuation.crocoddyl_model()
    q = robot.neutral_configuration(robot.config["initial_arm_configuration"])
    q[:3] = [0.0, 0.0, 1.0]
    v = np.zeros(robot.model.nv)

    equal = actuation.equal_hover_control()
    equal[actuation.n_rotors:] = actuation.gravity_compensated_hover_control(q)[actuation.n_rotors:]
    equal_acc = generalized_acceleration(robot, actuation, q, v, equal)
    trim = actuation.gravity_compensated_hover_control(q)
    trim_acc = generalized_acceleration(robot, actuation, q, v, trim)

    delta = 0.5
    rows = []
    for index, rotor in enumerate(actuation.rotors):
        thrusts = equal[:actuation.n_rotors].copy()
        baseline = actuation.rotor_thrust_to_wrench(thrusts)
        thrusts[index] += delta
        change = actuation.rotor_thrust_to_wrench(thrusts) - baseline
        rows.append({
            "rotor_id": int(rotor["id"]), "name": rotor["name"], "spin": rotor["spin"],
            "delta_thrust_N": delta, "Fx_N": change[0], "Fy_N": change[1], "Fz_N": change[2],
            "Mx_Nm": change[3], "My_Nm": change[4], "Mz_Nm": change[5],
        })
    with (output / "single_rotor_directions.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    baseline_control = trim.copy()
    baseline_acc = generalized_acceleration(robot, actuation, q, v, baseline_control)
    arm_control = baseline_control.copy()
    arm_control[actuation.n_rotors] += 0.1
    arm_acc = generalized_acceleration(robot, actuation, q, v, arm_control)
    angular_delta = arm_acc[3:6] - baseline_acc[3:6]

    fig, axis = plt.subplots(figsize=(8, 4))
    x = np.arange(actuation.n_rotors)
    axis.bar(x - 0.2, [row["Mx_Nm"] for row in rows], 0.2, label="Mx")
    axis.bar(x, [row["My_Nm"] for row in rows], 0.2, label="My")
    axis.bar(x + 0.2, [row["Mz_Nm"] for row in rows], 0.2, label="Mz")
    axis.set(xlabel="rotor id", ylabel="moment change [N m]", xticks=x)
    axis.legend()
    fig.tight_layout()
    fig.savefig(output / "single_rotor_moments.png", dpi=160)
    plt.close(fig)

    summary = {
        "pass": bool(np.linalg.norm(angular_delta) > 1e-8),
        "model": {"nq": robot.model.nq, "nv": robot.model.nv, "nu": actuation.nu,
                  "crocoddyl_nu": int(native.nu), "mapping_shape": list(actuation.mapping.shape)},
        "equal_hover": {"control": equal.tolist(), "acceleration": equal_acc.tolist(),
                        "base_linear_norm": float(np.linalg.norm(equal_acc[:3])),
                        "base_angular_norm": float(np.linalg.norm(equal_acc[3:6]))},
        "trim_hover": {"control": trim.tolist(), "acceleration": trim_acc.tolist(),
                       "base_linear_norm": float(np.linalg.norm(trim_acc[:3])),
                       "base_angular_norm": float(np.linalg.norm(trim_acc[3:6]))},
        "single_rotor": rows,
        "arm_reaction": {"joint": actuation.joint_names[0], "step_torque_Nm": 0.1,
                         "baseline_base_angular_accel": baseline_acc[3:6].tolist(),
                         "stepped_base_angular_accel": arm_acc[3:6].tolist(),
                         "delta_base_angular_accel": angular_delta.tolist(),
                         "delta_norm": float(np.linalg.norm(angular_delta))},
        "notes": [
            "Equal mg/4 thrust cannot exactly trim the offset manipulator center of mass.",
            "trim_hover is a bounded least-squares static allocation over all rotor and arm channels.",
            "Rotor reaction sign matches gazebo_motor_model.cpp: CCW gives negative body-z reaction torque.",
        ],
    }
    (output / "p1_summary.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    print("P1:", "PASS" if summary["pass"] else "FAIL")
    return 0 if summary["pass"] else 1


if __name__ == "__main__":
    sys.exit(main())

