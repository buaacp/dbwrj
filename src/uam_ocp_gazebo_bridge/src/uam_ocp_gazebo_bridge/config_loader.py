"""Resolve G1 config from existing P2.6/P2.7 artifacts."""

from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import yaml


def load_yaml(path):
    with open(str(path), "r") as stream:
        return yaml.safe_load(stream)


def resolve_path(root, value):
    path = Path(value)
    return path if path.is_absolute() else root / path


def resolve_g1_config(config_path, workspace_root):
    """Load config and resolve four scenarios without duplicating joint angles."""
    root = Path(workspace_root)
    cfg = load_yaml(config_path)
    watchdog_path = resolve_path(root, cfg["watchdog_file"])
    cfg.update(load_yaml(watchdog_path))
    paths = {key: resolve_path(root, value) for key, value in cfg["paths"].items()}
    task = load_yaml(paths["task_joints"])
    static = load_yaml(paths["static_trim_scenarios"])
    joint_names = list(task["trajectory_active_joints"]) + list(task["gripper_or_knuckle_joints"])
    scenarios = {}
    for name in cfg["configurations"]:
        if name == "p2_terminal":
            trajectory = np.load(str(paths["p2_terminal_trajectory"]))
            q = trajectory["states"][-1, :13]
            joints = dict(zip(joint_names, q[7:13].tolist()))
            source = str(paths["p2_terminal_trajectory"])
        else:
            entry = static["scenarios"].get(name)
            if entry is None:
                scenarios[name] = {"status": "CONFIGURATION_UNRESOLVED", "missing": name}
                continue
            joints = dict(entry["joints"])
            source = entry["source"]
        for joint, value in task["fixed_pregrasp_joint_positions"].items():
            joints[joint] = float(value)
        missing = [joint for joint in joint_names if joint not in joints]
        status = "CONFIGURATION_UNRESOLVED" if missing else "RESOLVED"
        scenarios[name] = {"status": status, "joints": joints, "source": source, "missing": missing}
    cfg["resolved_paths"] = {key: str(value) for key, value in paths.items()}
    cfg["resolved_paths"]["watchdog"] = str(watchdog_path)
    cfg["joint_names"] = joint_names
    urdf_root = ET.parse(str(paths["model_urdf"])).getroot()
    urdf_joints = dict((element.attrib["name"], element) for element in urdf_root.findall("joint"))
    cfg["joint_limits"] = {}
    for name in joint_names:
        limit = urdf_joints[name].find("limit")
        if limit is None or "lower" not in limit.attrib or "upper" not in limit.attrib:
            raise ValueError("G1 requires finite URDF limits for joint: " + name)
        cfg["joint_limits"][name] = {"lower_rad":float(limit.attrib["lower"]),
                                     "upper_rad":float(limit.attrib["upper"]),
                                     "source":str(paths["model_urdf"])}
    cfg["scenarios"] = scenarios
    return cfg
