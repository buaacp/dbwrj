"""Numerical and source-level validation for the generated rigid-body model."""

import json
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Mapping

import numpy as np
import pinocchio as pin
import yaml

from .model_loader import UamModel


def _source_inertials(urdf_path: Path) -> Dict[str, Dict[str, Any]]:
    root = ET.parse(str(urdf_path)).getroot()
    result: Dict[str, Dict[str, Any]] = {}
    for link in root.findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            result[link.get("name", "")] = {"has_inertial": False}
            continue
        mass = float(inertial.find("mass").get("value"))
        inertia = inertial.find("inertia")
        matrix = np.array([
            [float(inertia.get("ixx")), float(inertia.get("ixy", 0.0)), float(inertia.get("ixz", 0.0))],
            [float(inertia.get("ixy", 0.0)), float(inertia.get("iyy")), float(inertia.get("iyz", 0.0))],
            [float(inertia.get("ixz", 0.0)), float(inertia.get("iyz", 0.0)), float(inertia.get("izz"))],
        ])
        result[link.get("name", "")] = {
            "has_inertial": True, "mass": mass,
            "inertia_eigenvalues": np.linalg.eigvalsh(matrix).tolist(),
            "valid": bool(mass > 0.0 and np.all(np.linalg.eigvalsh(matrix) > 0.0)),
        }
    return result


def evaluate_configuration(robot: UamModel, q: np.ndarray) -> Dict[str, Any]:
    """Evaluate FK, EE Jacobian, CRBA matrix and nonlinear effects."""
    model, data = robot.model, robot.model.createData()
    v = np.zeros(model.nv)
    pin.forwardKinematics(model, data, q, v)
    pin.updateFramePlacements(model, data)
    placement = data.oMf[robot.end_effector_frame_id]
    jacobian = pin.computeFrameJacobian(
        model, data, q, robot.end_effector_frame_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    mass_matrix = pin.crba(model, data, q)
    mass_matrix = 0.5 * (mass_matrix + mass_matrix.T)
    nle = pin.nonLinearEffects(model, data, q, v)
    return {
        "q": q.tolist(),
        "ee_position_world": placement.translation.tolist(),
        "ee_rotation_world": placement.rotation.tolist(),
        "ee_jacobian_shape": list(jacobian.shape),
        "ee_jacobian_rank": int(np.linalg.matrix_rank(jacobian)),
        "mass_matrix_shape": list(mass_matrix.shape),
        "mass_matrix_min_eigenvalue": float(np.linalg.eigvalsh(mass_matrix).min()),
        "mass_matrix_condition": float(np.linalg.cond(mass_matrix)),
        "nonlinear_effects": nle.tolist(),
        "finite": bool(np.all(np.isfinite(jacobian)) and np.all(np.isfinite(mass_matrix)) and np.all(np.isfinite(nle))),
    }


def validate_model(robot: UamModel) -> Dict[str, Any]:
    """Return the complete machine-readable P0 validation summary."""
    model = robot.model
    expected_nq = 7 + robot.n_arm
    expected_nv = 6 + robot.n_arm
    neutral = robot.neutral_configuration()
    nonzero = robot.neutral_configuration(robot.config["nonzero_validation_configuration"])
    inertials = _source_inertials(robot.urdf_path)
    invalid_links = [name for name, item in inertials.items()
                     if item.get("has_inertial") and not item.get("valid")]
    no_inertial = [name for name, item in inertials.items() if not item.get("has_inertial")]
    quaternion_norm = float(np.linalg.norm(neutral[3:7]))
    joints = [item.__dict__ for item in robot.arm_joints]
    checks = {
        "dimensions": model.nq == expected_nq and model.nv == expected_nv,
        "quaternion": abs(quaternion_norm - 1.0) < 1e-12,
        "end_effector": model.existFrame(robot.end_effector_frame),
        "positive_total_mass": robot.total_mass > 0.0,
        "source_inertials": not invalid_links,
    }
    neutral_result = evaluate_configuration(robot, neutral)
    nonzero_result = evaluate_configuration(robot, nonzero)
    checks["neutral_dynamics"] = neutral_result["finite"] and neutral_result["mass_matrix_min_eigenvalue"] > 0.0
    checks["nonzero_dynamics"] = nonzero_result["finite"] and nonzero_result["mass_matrix_min_eigenvalue"] > 0.0
    return {
        "pass": all(checks.values()), "checks": checks,
        "urdf_path": str(robot.urdf_path), "base_frame": robot.base_frame,
        "end_effector_frame": robot.end_effector_frame,
        "n_arm": robot.n_arm, "nq": model.nq, "nv": model.nv,
        "expected_nq": expected_nq, "expected_nv": expected_nv,
        "total_mass_kg": robot.total_mass, "quaternion_norm": quaternion_norm,
        "joints": joints, "source_link_inertials": inertials,
        "links_without_inertial": no_inertial, "invalid_inertial_links": invalid_links,
        "neutral": neutral_result, "nonzero": nonzero_result,
        "risks": [
            "base_link has no inertial tag but is fixed to massive base_link_inertia; Pinocchio merges the fixed body inertia.",
            "Arm masses are only 5-40 g per link and require calibration against the physical arm.",
            "URDF effort=1000 N m is not credible for this arm and is not used as an optimization bound.",
            "Mimic gripper joints are fixed at zero in the P0-P2 model; gripper motion/contact is outside scope.",
            "wrist_roll_joint is continuous in Gazebo and is represented by a TODO finite interval in optimization.",
        ],
    }


def save_summary(summary: Mapping[str, Any], output_dir: Path) -> None:
    """Write equivalent JSON and YAML P0 summaries."""
    output_dir.mkdir(parents=True, exist_ok=True)
    (output_dir / "p0_model_summary.json").write_text(
        json.dumps(summary, indent=2, ensure_ascii=True), encoding="utf-8")
    (output_dir / "p0_model_summary.yaml").write_text(
        yaml.safe_dump(dict(summary), sort_keys=False), encoding="utf-8")

