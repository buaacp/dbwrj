"""Load and inspect the canonical floating-base Pinocchio model."""

from dataclasses import dataclass
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET
from typing import Any, Dict, List, Mapping, Optional, Sequence, Tuple

import numpy as np
import pinocchio as pin
import yaml


MODULE_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = MODULE_ROOT.parent


def load_yaml(path: Path) -> Dict[str, Any]:
    """Load a YAML mapping and reject non-mapping documents."""
    with path.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f"Expected YAML mapping in {path}")
    return value


@dataclass(frozen=True)
class JointInfo:
    """Pinocchio index and limit metadata for one independent joint."""

    name: str
    joint_id: int
    idx_q: int
    nq: int
    idx_v: int
    nv: int
    lower: List[float]
    upper: List[float]
    velocity_limit: List[float]
    effort_limit: List[float]


@dataclass
class UamModel:
    """Loaded model together with configuration-derived frame metadata."""

    model: pin.Model
    data: pin.Data
    state: Any
    urdf_path: Path
    config: Dict[str, Any]
    base_frame: str
    end_effector_frame: str
    end_effector_frame_id: int
    arm_joints: List[JointInfo]
    inactive_joint_names: List[str]

    @property
    def n_arm(self) -> int:
        return sum(item.nv for item in self.arm_joints)

    @property
    def total_mass(self) -> float:
        return float(pin.computeTotalMass(self.model))

    def neutral_configuration(self, joint_values: Optional[Mapping[str, float]] = None) -> np.ndarray:
        """Return normalized free-flyer neutral q with optional named joints."""
        q = pin.neutral(self.model)
        for name, value in (joint_values or {}).items():
            if not self.model.existJointName(name):
                if name in self.inactive_joint_names:
                    continue
                raise KeyError(f"Unknown joint {name}")
            joint_id = self.model.getJointId(name)
            joint = self.model.joints[joint_id]
            if joint.nq != 1:
                raise ValueError(f"Joint {name} has nq={joint.nq}; generated model requires scalar joints")
            q[joint.idx_q] = float(value)
        return pin.normalize(self.model, q)


def _joint_info(model: pin.Model) -> List[JointInfo]:
    result: List[JointInfo] = []
    for joint_id in range(2, model.njoints):
        joint = model.joints[joint_id]
        result.append(JointInfo(
            name=str(model.names[joint_id]), joint_id=joint_id,
            idx_q=int(joint.idx_q), nq=int(joint.nq),
            idx_v=int(joint.idx_v), nv=int(joint.nv),
            lower=model.lowerPositionLimit[joint.idx_q:joint.idx_q + joint.nq].tolist(),
            upper=model.upperPositionLimit[joint.idx_q:joint.idx_q + joint.nq].tolist(),
            velocity_limit=model.velocityLimit[joint.idx_v:joint.idx_v + joint.nv].tolist(),
            effort_limit=model.effortLimit[joint.idx_v:joint.idx_v + joint.nv].tolist(),
        ))
    return result


def _expand_canonical_xacro(config: Dict[str, Any], output: Path) -> None:
    source = Path(config["source"]["xacro"])
    if not source.exists():
        raise FileNotFoundError(f"Canonical xacro does not exist: {source}")
    args = dict(config["source"].get("xacro_args", {}))
    if config.get("lock_shoulder_pan", True):
        args["lock_shoulder_pan"] = "true"
    command = "source /opt/ros/melodic/setup.bash"
    catkin_setup = Path("/home/zlhq/catkin_ws/devel/setup.bash")
    if catkin_setup.exists():
        command += f" && source {catkin_setup}"
    xacro_args = " ".join(f"{key}:={value}" for key, value in args.items())
    command += f" && xacro --inorder {source} {xacro_args}"
    expanded = subprocess.run(
        ["bash", "-lc", command],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
    ).stdout
    root = ET.fromstring(expanded)
    _apply_generation_policy(root, config)
    output.parent.mkdir(parents=True, exist_ok=True)
    _indent_xml(root)
    ET.ElementTree(root).write(output, encoding="utf-8", xml_declaration=True)


def _apply_generation_policy(root: ET.Element, config: Dict[str, Any]) -> None:
    policy = config.get("generation_policy", {})
    continuous_bounds = policy.get("continuous_joint_bounds", {})
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
        elif joint_type == "continuous" and name in continuous_bounds:
            bounds = continuous_bounds[name]
            joint.set("type", "revolute")
            if limit is None:
                limit = ET.SubElement(joint, "limit")
            limit.set("lower", str(bounds["lower"]))
            limit.set("upper", str(bounds["upper"]))
            limit.attrib.setdefault("effort", "0")
            limit.attrib.setdefault("velocity", "0")
    for tag in ("gazebo", "transmission"):
        for element in list(root.findall(tag)):
            root.remove(element)


def _indent_xml(element: ET.Element, level: int = 0) -> None:
    spacing = "\n" + level * "  "
    child_spacing = "\n" + (level + 1) * "  "
    if len(element):
        if not element.text or not element.text.strip():
            element.text = child_spacing
        for child in element:
            _indent_xml(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = child_spacing
        element[-1].tail = spacing


def _reference_configuration(model: pin.Model, values: Mapping[str, float]) -> np.ndarray:
    q = pin.neutral(model)
    for name, value in values.items():
        if not model.existJointName(name):
            continue
        joint_id = model.getJointId(name)
        joint = model.joints[joint_id]
        if joint.nq == 1:
            q[joint.idx_q] = float(value)
    return pin.normalize(model, q)


def _reduce_to_active_arm(model: pin.Model, config: Dict[str, Any]) -> Tuple[pin.Model, List[str]]:
    active = list(config.get("active_arm_joint_names", []))
    if not active:
        return model, []
    missing_active = [name for name in active if not model.existJointName(name)]
    if missing_active:
        raise ValueError(f"Active arm joints missing from canonical model: {missing_active}")
    inactive = set(config.get("locked_joint_names", [])) | set(config.get("excluded_joint_names", []))
    inactive.update(name for name in model.names[2:] if str(name) not in active)
    inactive_ids = [
        int(model.getJointId(name)) for name in sorted(inactive)
        if model.existJointName(name) and model.joints[model.getJointId(name)].nq > 0
    ]
    q_ref = _reference_configuration(model, config.get("initial_arm_configuration", {}))
    reduced = pin.buildReducedModel(model, inactive_ids, q_ref)
    reduced_names = [str(reduced.names[joint_id]) for joint_id in range(2, reduced.njoints)]
    if reduced_names != active:
        raise ValueError(f"Reduced model arm joint order {reduced_names} != configured active joints {active}")
    return reduced, sorted(inactive)


def load_uam_model(config_path: Optional[Path] = None) -> UamModel:
    """Build the free-flyer model from the canonical locked iris_arm xacro."""
    config_path = config_path or MODULE_ROOT / "config" / "uam_model.yaml"
    config = load_yaml(Path(config_path))
    urdf_path = PROJECT_ROOT / config["generated_urdf"]
    _expand_canonical_xacro(config, urdf_path)
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
    model, inactive_joint_names = _reduce_to_active_arm(model, config)
    model.gravity.linear = np.asarray(config.get("gravity", [0.0, 0.0, -9.81]), dtype=float)
    import crocoddyl
    state = crocoddyl.StateMultibody(model)
    ee_name = str(config["end_effector_frame"])
    if not model.existFrame(ee_name):
        raise ValueError(f"End-effector frame {ee_name!r} is absent")
    return UamModel(
        model=model, data=model.createData(), state=state, urdf_path=urdf_path,
        config=config, base_frame=str(config["base_frame"]),
        end_effector_frame=ee_name, end_effector_frame_id=int(model.getFrameId(ee_name)),
        arm_joints=_joint_info(model),
        inactive_joint_names=inactive_joint_names,
    )
