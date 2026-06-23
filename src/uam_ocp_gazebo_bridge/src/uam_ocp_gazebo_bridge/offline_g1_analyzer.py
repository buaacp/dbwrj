"""Offline-only G1 telemetry analysis and plotting."""

import csv
from pathlib import Path
import yaml

from .safety_supervisor import assess_run


def _load_rows(path):
    telemetry = Path(path) / "telemetry.csv"
    if not telemetry.exists() or telemetry.stat().st_size == 0:
        return []
    with telemetry.open() as stream:
        return list(csv.DictReader(stream))


def analyze_run(input_directory, output_directory, thresholds=None):
    """Analyze one local run directory without contacting ROS."""
    source = Path(input_directory)
    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    rows = _load_rows(source)
    result = yaml.safe_load((source / "result.yaml").read_text()) if (source / "result.yaml").exists() else {}
    if thresholds is None:
        config = yaml.safe_load((source / "experiment_config.yaml").read_text()) if (source / "experiment_config.yaml").exists() else {}
        thresholds = config.get("pass_thresholds", {
            "peak_position_error_m": .15, "final_hold_position_error_m": .05,
            "peak_roll_pitch_deg": 10., "final_speed_mps": .05,
            "max_joint_tracking_error_deg": 3., "offboard_must_remain_active": True,
            "abort_must_not_trigger": True})
    abort = result.get("abort") or {}
    status, metrics = assess_run(rows, thresholds, bool(abort))
    if result.get("status") in ("CONFIGURATION_UNRESOLVED", "NOT_RUN_ENVIRONMENT_UNAVAILABLE", "NOT_RUN_INTERFACE_UNAVAILABLE"):
        status = result["status"]
    historical = None
    if abort.get("reason") == "STATE_TIMEOUT":
        historical = "ABORTED_FALSE_STALE_STATE"
    abort_state = abort.get("state_machine_state", abort.get("state"))
    if abort.get("reason") == "POSITION_ERROR" and abort_state == "ARM_AND_OFFBOARD":
        historical = "HISTORICAL_STARTUP_REFERENCE_BUG"
    summary = {"status": status, "historical_classification": historical,
               "metrics": metrics, "abort": abort or None, "input": str(source)}
    (output / "summary.yaml").write_text(yaml.safe_dump(summary, sort_keys=False))
    with (output / "summary.csv").open("w", newline="") as stream:
        fields = ["status"] + sorted(metrics)
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader(); writer.writerow({"status": status, **metrics})
    (output / "G1_VALIDATION.md").write_text(_markdown(summary))
    _plots(rows, source, output / "plots")
    return summary


def _markdown(summary):
    lines = ["# G1 validation", "", "Status: **%s**" % summary["status"], ""]
    if summary.get("abort"):
        lines += ["Abort: `%s`" % summary["abort"].get("reason", "UNKNOWN"), ""]
    if summary.get("historical_classification"):
        lines += ["Historical classification: `%s`" % summary["historical_classification"], ""]
    lines += ["## Metrics", ""] + ["- `%s`: %s" % item for item in sorted(summary["metrics"].items())]
    return "\n".join(lines) + "\n"


def _plots(rows, source, plot_directory):
    plot_directory = Path(plot_directory); plot_directory.mkdir(parents=True, exist_ok=True)
    try:
        import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    except ImportError:
        return
    required = ("position_error.png", "base_attitude.png", "joint_tracking_error.png",
                "state_machine_timeline.png", "topic_interarrival_times.png",
                "takeoff_transition_position_tracking.png","arm_neutralization_tracking.png")
    if not rows:
        for name in required:
            fig, ax = plt.subplots(); ax.text(.5, .5, "NO TELEMETRY", ha="center", va="center"); ax.set_axis_off(); fig.savefig(str(plot_directory / name)); plt.close(fig)
        return
    time_values = [float(row["sim_time_s"]) - float(rows[0]["sim_time_s"]) for row in rows]
    for name, keys, ylabel in (
            ("position_error.png", ("position_error_m",), "position error [m]"),
            ("base_attitude.png", ("roll_rad", "pitch_rad"), "angle [rad]"),
            ("joint_tracking_error.png", ("max_joint_error_rad",), "max joint error [rad]")):
        fig, ax = plt.subplots()
        for key in keys: ax.plot(time_values, [float(row[key]) for row in rows], label=key)
        ax.set(xlabel="simulation elapsed time [s]", ylabel=ylabel); ax.legend(); fig.tight_layout(); fig.savefig(str(plot_directory / name), dpi=150); plt.close(fig)
    states = []
    for row in rows:
        if row["state"] not in states: states.append(row["state"])
    fig, ax = plt.subplots(); ax.step(time_values, [states.index(row["state"]) for row in rows], where="post"); ax.set_yticks(range(len(states))); ax.set_yticklabels(states); ax.set_xlabel("simulation elapsed time [s]"); fig.tight_layout(); fig.savefig(str(plot_directory / "state_machine_timeline.png"), dpi=150); plt.close(fig)
    transition=[(time_values[i],row) for i,row in enumerate(rows) if row.get("active_reference_kind")=="TAKEOFF_TRANSITION"]
    fig,ax=plt.subplots()
    if transition:
        t=[item[0] for item in transition]
        for actual,reference,axis in (("px","ref_px","x"),("py","ref_py","y"),("pz","ref_pz","z")):
            ax.plot(t,[float(item[1][actual]) for item in transition],label=axis+" actual")
            ax.plot(t,[float(item[1][reference]) for item in transition],"--",label=axis+" reference")
        ax.set_ylabel("world position [m]");ax.legend(fontsize=7)
    else:ax.text(.5,.5,"NO TAKEOFF_TRANSITION DATA",ha="center",va="center",transform=ax.transAxes)
    ax.set_xlabel("simulation elapsed time [s]");fig.tight_layout();fig.savefig(str(plot_directory/"takeoff_transition_position_tracking.png"),dpi=150);plt.close(fig)
    metadata = yaml.safe_load((source / "run_metadata.yaml").read_text()) if (source / "run_metadata.yaml").exists() else {}
    arm_meta=metadata.get("arm_startup_reference",{});joint_names=arm_meta.get("joint_names",[]);neutral=arm_meta.get("q_neutral_rad",[])
    neutral_rows=[(time_values[i],row) for i,row in enumerate(rows) if row.get("state")=="ARM_NEUTRALIZE"]
    fig,axes=plt.subplots(max(1,len(joint_names)),1,figsize=(9,max(3,2*len(joint_names))),sharex=True);axes=[axes] if len(joint_names)<=1 else axes
    if neutral_rows and joint_names:
        t=[item[0] for item in neutral_rows]
        for index,name in enumerate(joint_names):
            axes[index].plot(t,[float(item[1]["q_"+name]) for item in neutral_rows],label="actual")
            axes[index].plot(t,[float(item[1]["qref_"+name]) for item in neutral_rows],"--",label="profile")
            axes[index].axhline(float(neutral[index]),color="k",linestyle=":",label="neutral");axes[index].set_ylabel(name);axes[index].legend(fontsize=6)
    else:axes[0].text(.5,.5,"NO ARM_NEUTRALIZE DATA",ha="center",va="center",transform=axes[0].transAxes)
    axes[-1].set_xlabel("simulation elapsed time [s]");fig.tight_layout();fig.savefig(str(plot_directory/"arm_neutralization_tracking.png"),dpi=150);plt.close(fig)
    rates = yaml.safe_load((source / "topic_rates.yaml").read_text()) if (source / "topic_rates.yaml").exists() else {"topic_rates_hz": {}}
    available = [(key, value.get("max_interarrival_s")) for key, value in rates.get("topic_rates_hz", {}).items() if value.get("max_interarrival_s") is not None]
    fig, ax = plt.subplots(); ax.bar([item[0] for item in available], [item[1] for item in available]); ax.set_ylabel("maximum monotonic interarrival [s]"); ax.tick_params(axis="x", rotation=45); fig.tight_layout(); fig.savefig(str(plot_directory / "topic_interarrival_times.png"), dpi=150); plt.close(fig)
    if metadata.get("joint_effort_available"):
        keys = [key for key in rows[0] if key.startswith("effort_")]
        fig, ax = plt.subplots()
        for key in keys: ax.plot(time_values, [float(row[key]) for row in rows], label=key)
        ax.legend(fontsize=6); fig.tight_layout(); fig.savefig(str(plot_directory / "joint_effort_if_available.png"), dpi=150); plt.close(fig)
