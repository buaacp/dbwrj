#!/usr/bin/env python3
"""Expand the source Xacro and derive the Pinocchio optimization URDF."""

import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parent


def indent_xml(element: ET.Element, level: int = 0) -> None:
    """Indent XML on Python versions predating ElementTree.indent."""
    spacing = "\n" + level * "  "
    child_spacing = "\n" + (level + 1) * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_spacing
        for child in element:
            indent_xml(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = child_spacing
        element[-1].tail = spacing


def main() -> int:
    """Generate a deterministic URDF while preserving inertial parameters."""
    config = yaml.safe_load((ROOT / "config" / "uam_model.yaml").read_text(encoding="utf-8"))
    source = Path(config["source"]["xacro"])
    output = PROJECT_ROOT / config["generated_urdf"]
    command = ["xacro", "--inorder", str(source)]
    command.extend(f"{key}:={value}" for key, value in config["source"]["xacro_args"].items())
    expanded = subprocess.run(
        command, check=True, stdout=subprocess.PIPE, universal_newlines=True).stdout
    root = ET.fromstring(expanded)
    policy = config["generation_policy"]
    continuous_bounds = policy.get("continuous_joint_bounds", {})
    changes = []
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        joint_type = joint.get("type", "")
        limit = joint.find("limit")
        mimic = joint.find("mimic")
        zero_range = False
        if limit is not None and "lower" in limit.attrib and "upper" in limit.attrib:
            zero_range = abs(float(limit.get("upper")) - float(limit.get("lower"))) < 1e-12
        should_fix = (
            (policy.get("fix_rotor_joints", True) and name.startswith("rotor_"))
            or (policy.get("fix_zero_range_joints", True) and zero_range)
            or (policy.get("fix_mimic_joints", True) and mimic is not None)
        )
        if should_fix and joint_type != "fixed":
            joint.set("type", "fixed")
            if limit is not None:
                joint.remove(limit)
            if mimic is not None:
                joint.remove(mimic)
            changes.append(f"fixed:{name}")
        elif joint_type == "continuous" and name in continuous_bounds:
            bounds = continuous_bounds[name]
            joint.set("type", "revolute")
            if limit is None:
                limit = ET.SubElement(joint, "limit")
            limit.set("lower", str(bounds["lower"]))
            limit.set("upper", str(bounds["upper"]))
            limit.attrib.setdefault("effort", "0")
            limit.attrib.setdefault("velocity", "0")
            changes.append(f"bounded_continuous:{name}")
    for tag in ("gazebo", "transmission"):
        for element in list(root.findall(tag)):
            root.remove(element)
    output.parent.mkdir(parents=True, exist_ok=True)
    indent_xml(root)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)
    print(f"Generated {output}")
    print("Transformations:", ", ".join(changes))
    return 0


if __name__ == "__main__":
    sys.exit(main())
