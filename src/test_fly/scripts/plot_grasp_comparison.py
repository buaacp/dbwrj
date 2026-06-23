#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import argparse
import csv
import math
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def read_csv(path):
    rows = []
    with open(path, "r") as file_obj:
        reader = csv.DictReader(file_obj)
        for row in reader:
            parsed = {}
            for key, value in row.items():
                if key == "label":
                    parsed[key] = value
                else:
                    try:
                        parsed[key] = float(value)
                    except (TypeError, ValueError):
                        parsed[key] = float("nan")
            rows.append(parsed)
    return rows


def series(rows, key):
    return [row[key] for row in rows]


def finite_values(values):
    return [value for value in values if not (math.isnan(value) or math.isinf(value))]


def rmse(values):
    data = finite_values(values)
    if not data:
        return float("nan")
    return math.sqrt(sum(value * value for value in data) / len(data))


def max_abs(values):
    data = finite_values(values)
    if not data:
        return float("nan")
    return max(abs(value) for value in data)


def final_value(values):
    data = finite_values(values)
    return data[-1] if data else float("nan")


def summarize(label, rows):
    return {
        "label": label,
        "samples": len(rows),
        "duration_s": rows[-1]["t"] if rows else float("nan"),
        "final_state": rows[-1]["mission_state"] if rows else float("nan"),
        "final_ready": rows[-1]["arm_ready_state"] if rows else float("nan"),
        "uav_pos_rmse": rmse(series(rows, "uav_pos_error")),
        "uav_xy_rmse": rmse(series(rows, "uav_xy_error")),
        "uav_yaw_rmse_deg": rmse(series(rows, "uav_yaw_error_deg")),
        "arm_pos_final": final_value(series(rows, "arm_pos_error")),
        "arm_axis_final_deg": final_value(series(rows, "arm_axis_error_deg")),
        "arm_centerline_final": final_value(series(rows, "arm_centerline_error")),
        "arm_centerline_max": max_abs(series(rows, "arm_centerline_error")),
    }


def write_summary(path, summaries):
    keys = [
        "label",
        "samples",
        "duration_s",
        "final_state",
        "final_ready",
        "uav_pos_rmse",
        "uav_xy_rmse",
        "uav_yaw_rmse_deg",
        "arm_pos_final",
        "arm_axis_final_deg",
        "arm_centerline_final",
        "arm_centerline_max",
    ]
    with open(path, "w") as file_obj:
        writer = csv.DictWriter(file_obj, fieldnames=keys)
        writer.writeheader()
        for item in summaries:
            writer.writerow(item)


def plot_runs(runs, output_png):
    fig, axes = plt.subplots(3, 2, figsize=(13, 10), sharex=True)
    axes = axes.flatten()
    plots = [
        ("uav_pos_error", "UAV position error", "m"),
        ("uav_xy_error", "UAV horizontal error", "m"),
        ("uav_yaw_error_deg", "UAV yaw error", "deg"),
        ("arm_pos_error", "Arm gripper position error", "m"),
        ("arm_axis_error_deg", "Arm end-axis attitude error", "deg"),
        ("arm_centerline_error", "Arm centerline error", "m"),
    ]
    for ax, (key, title, ylabel) in zip(axes, plots):
        for label, rows in runs:
            ax.plot(series(rows, "t"), series(rows, key), label=label, linewidth=1.6)
        ax.set_title(title)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.3)
    for ax in axes[-2:]:
        ax.set_xlabel("time (s)")
    axes[0].legend(loc="best")
    fig.tight_layout()
    fig.savefig(output_png, dpi=160)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--off", required=True, help="CSV recorded with anti-orbit disabled")
    parser.add_argument("--on", required=True, help="CSV recorded with anti-orbit enabled")
    parser.add_argument("--output-png", required=True)
    parser.add_argument("--summary-csv", required=True)
    args = parser.parse_args()

    runs = [
        ("anti_orbit_off", read_csv(args.off)),
        ("anti_orbit_on", read_csv(args.on)),
    ]
    output_dir = os.path.dirname(os.path.abspath(args.output_png))
    if output_dir and not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    plot_runs(runs, args.output_png)
    write_summary(args.summary_csv, [summarize(label, rows) for label, rows in runs])


if __name__ == "__main__":
    main()
