"""Load and inspect the generated floating-base Pinocchio model."""

from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, List, Mapping, Optional

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
            joint_id = self.model.getJointId(name)
            if joint_id == 0:
                raise KeyError(f"Unknown joint {name}")
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


def load_uam_model(config_path: Optional[Path] = None) -> UamModel:
    """Build the free-flyer model from the reproducibly generated URDF."""
    config_path = config_path or MODULE_ROOT / "config" / "uam_model.yaml"
    config = load_yaml(Path(config_path))
    urdf_path = PROJECT_ROOT / config["generated_urdf"]
    if not urdf_path.exists():
        raise FileNotFoundError(f"Generate the optimization URDF first: {urdf_path}")
    model = pin.buildModelFromUrdf(str(urdf_path), pin.JointModelFreeFlyer())
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
    )

