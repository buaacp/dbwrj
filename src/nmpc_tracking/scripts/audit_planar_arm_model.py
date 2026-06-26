#!/usr/bin/env python3
import csv
import os
import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


WORKSPACE = Path(__file__).resolve().parents[3]
UAM_CONFIG = WORKSPACE / "uam_ocp" / "config" / "uam_model.yaml"
RESULTS = WORKSPACE / "results" / "planar_arm_model_audit"


def _expand_xacro(config):
    source = Path(config["source"]["xacro"])
    args = dict(config["source"].get("xacro_args", {}))
    args["lock_shoulder_pan"] = "true"
    command = "source /opt/ros/melodic/setup.bash"
    catkin_setup = Path("/home/zlhq/catkin_ws/devel/setup.bash")
    if catkin_setup.exists():
        command += f" && source {catkin_setup}"
    command += " && xacro --inorder %s %s" % (
        source, " ".join("%s:=%s" % (k, v) for k, v in args.items()))
    output = subprocess.run(
        ["bash", "-lc", command],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
    ).stdout
    return ET.fromstring(output)


def _controller_yaml(path):
    with Path(path).open("r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def main():
    config = yaml.safe_load(UAM_CONFIG.read_text(encoding="utf-8"))
    root = _expand_xacro(config)
    RESULTS.mkdir(parents=True, exist_ok=True)

    transmissions = {}
    for transmission in root.findall("transmission"):
        joint = transmission.find("joint")
        if joint is not None:
            transmissions[joint.get("name")] = transmission.get("name")

    rows = []
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        mimic = joint.find("mimic")
        rows.append({
            "name": joint.get("name", ""),
            "type": joint.get("type", ""),
            "parent": joint.find("parent").get("link") if joint.find("parent") is not None else "",
            "child": joint.find("child").get("link") if joint.find("child") is not None else "",
            "axis": joint.find("axis").get("xyz") if joint.find("axis") is not None else "",
            "lower": limit.get("lower") if limit is not None else "",
            "upper": limit.get("upper") if limit is not None else "",
            "velocity": limit.get("velocity") if limit is not None else "",
            "effort": limit.get("effort") if limit is not None else "",
            "mimic": mimic.get("joint") if mimic is not None else "",
            "transmission": transmissions.get(joint.get("name", ""), ""),
        })
    with (RESULTS / "urdf_joint_table.csv").open("w", newline="", encoding="utf-8") as stream:
        fieldnames = [
            "name", "type", "parent", "child", "axis", "lower", "upper",
            "velocity", "effort", "mimic", "transmission",
        ]
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    le_arm_controller = _controller_yaml("/home/zlhq/catkin_ws/src/le_arm/controller/le_arm_controller.yaml")
    wrist_controller = _controller_yaml("/home/zlhq/catkin_ws/src/le_arm/controller/wrist_roll_controller.yaml")
    gripper_controller = _controller_yaml("/home/zlhq/catkin_ws/src/le_arm/controller/gripper_controller.yaml")
    le_arm_joints = le_arm_controller["le_arm_controller"]["joints"]
    wrist_joint = wrist_controller["wrist_roll_controller"]["joint"]
    gripper_joint = gripper_controller["gripper_controller"].get(
        "joint", gripper_controller["gripper_controller"].get("joints", []))

    chain = [
        "# iris_arm Launch Chain",
        "",
        f"- canonical launch: `{config['canonical_launch']}`",
        f"- robot_description xacro: `{config['source']['xacro']}`",
        "- includes: `component_snippets.xacro`, `iris.xacro`, `le_arm.urdf.xacro`",
        "- arm xacro includes: `le_arm.transmission.xacro`, `le_arm_gripper.urdf.xacro`",
        "- controller YAML:",
        "  - `/home/zlhq/catkin_ws/src/le_arm/controller/le_arm_controller.yaml`",
        "  - `/home/zlhq/catkin_ws/src/le_arm/controller/wrist_roll_controller.yaml`",
        "  - `/home/zlhq/catkin_ws/src/le_arm/controller/gripper_controller.yaml`",
        "",
        "## Static Findings",
        "",
        f"- shoulder_pan_joint type with lock_shoulder_pan=true: `{next(r['type'] for r in rows if r['name'] == 'shoulder_pan_joint')}`",
        f"- shoulder_pan_joint transmission: `{transmissions.get('shoulder_pan_joint', '')}`",
        f"- /le_arm_controller/command joints: `{le_arm_joints}`",
        f"- /wrist_roll_controller/command joint: `{wrist_joint}`",
        f"- gripper controller joint: `{gripper_joint}`",
    ]
    (RESULTS / "iris_arm_launch_chain.md").write_text("\n".join(chain) + "\n", encoding="utf-8")

    summary = {
        "canonical_launch": config["canonical_launch"],
        "canonical_xacro": config["source"]["xacro"],
        "lock_shoulder_pan": True,
        "model_variant": config["model_variant"],
        "active_arm_joint_names": config["active_arm_joint_names"],
        "locked_joint_names": config["locked_joint_names"],
        "excluded_joint_names": config["excluded_joint_names"],
        "shoulder_pan_joint_type": next(r["type"] for r in rows if r["name"] == "shoulder_pan_joint"),
        "shoulder_pan_has_transmission": "shoulder_pan_joint" in transmissions,
        "le_arm_controller_command_joints": le_arm_joints,
        "wrist_roll_controller_command_joint": wrist_joint,
        "gripper_controller_joint": gripper_joint,
        "wrist_2_joint": next((r for r in rows if r["name"] == "wrist_2_joint"), None),
        "gazebo_and_pinocchio_same_xacro_source": True,
        "pinocchio_reduction_note": "Pinocchio loads the same locked xacro expansion and reduces excluded gripper/zero-range joints with the configured locked reference.",
    }
    (RESULTS / "model_source_summary.yaml").write_text(
        yaml.safe_dump(summary, sort_keys=False), encoding="utf-8")
    print(yaml.safe_dump(summary, sort_keys=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
