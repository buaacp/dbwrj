#!/usr/bin/env python3
"""Expand the source Xacro and derive the Pinocchio optimization URDF."""

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = ROOT.parent
sys.path.insert(0, str(ROOT))
from uam_ocp.model_loader import _apply_generation_policy, _expand_canonical_xacro


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
    output = PROJECT_ROOT / config["generated_urdf"]
    _expand_canonical_xacro(config, output)
    print(f"Generated {output}")
    print("Canonical launch:", config.get("canonical_launch"))
    print("Model variant:", config.get("model_variant"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
