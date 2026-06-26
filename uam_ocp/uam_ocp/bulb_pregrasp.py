"""Bulb-frame target resolution and active-joint damped least-squares IK."""

import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import numpy as np
import pinocchio as pin

from .model_loader import MODULE_ROOT, UamModel, load_yaml


class IKSeedUnreachable(RuntimeError):
    """Raised when the configured active joints cannot satisfy the pregrasp pose."""


def load_pregrasp_configuration(scenario_name: str = "scene_bulb_pregrasp") -> Tuple[Dict[str, Any], Dict[str, Any]]:
    """Load scenario and explicit task-joint semantics."""
    scenarios = load_yaml(MODULE_ROOT / "config" / "bulb_pregrasp_scenarios.yaml")
    joints = load_yaml(MODULE_ROOT / "config" / "uam_task_joints.yaml")
    scenario = _resolve_scenario(scenarios, scenario_name)
    return scenario, joints


def _resolve_scenario(scenarios: Dict[str, Any], scenario_name: str) -> Dict[str, Any]:
    """Resolve optional base_scenario inheritance for offline stress tests."""
    if scenario_name not in scenarios:
        raise KeyError(scenario_name)
    scenario = dict(scenarios[scenario_name])
    if "base_scenario" not in scenario:
        scenario["scenario_name"] = scenario_name
        return scenario
    base = _resolve_scenario(scenarios, str(scenario["base_scenario"]))
    merged = _deep_update(base, scenario)
    merged["scenario_name"] = scenario_name
    offset = np.asarray(merged.get("target_offset_world_m", [0.0, 0.0, 0.0]), dtype=float)
    if np.linalg.norm(offset) > 0.0:
        position = np.asarray(merged["bulb_pose_world"]["position"], dtype=float) + offset
        merged["bulb_pose_world"] = dict(merged["bulb_pose_world"])
        merged["bulb_pose_world"]["position"] = position.tolist()
        merged["scene_file"] = None
        merged["scene_model_name"] = None
    return merged


def _deep_update(base: Dict[str, Any], update: Dict[str, Any]) -> Dict[str, Any]:
    result = dict(base)
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_update(result[key], value)
        else:
            result[key] = value
    return result


def resolve_bulb_pose(scenario: Dict[str, Any]) -> Tuple[pin.SE3, Dict[str, Any]]:
    """Resolve the bulb pose and verify a scene-file source when configured."""
    source = str(scenario["pose_source"]).lower()
    configured_position = np.asarray(scenario["bulb_pose_world"]["position"], dtype=float)
    configured_quaternion = pin.Quaternion(np.asarray(
        scenario["bulb_pose_world"]["quaternion_xyzw"], dtype=float))
    configured_quaternion.normalize()
    diagnostics: Dict[str, Any] = {"pose_source": source.upper(), "verified": False}
    if source == "scene_file":
        path = Path(scenario["scene_file"])
        root = ET.parse(str(path)).getroot()
        model = next((item for item in root.iter("model")
                      if item.get("name") == scenario["scene_model_name"]), None)
        if model is None or model.find("pose") is None:
            raise ValueError(f"Model {scenario['scene_model_name']} has no pose in {path}")
        values = [float(value) for value in model.find("pose").text.split()]
        scene_position = np.asarray(values[:3])
        scene_rotation = pin.rpy.rpyToMatrix(*values[3:6])
        if not np.allclose(scene_position, configured_position, atol=1e-12):
            raise ValueError("Configured bulb position differs from scene file")
        if not np.allclose(scene_rotation, configured_quaternion.matrix(), atol=1e-12):
            raise ValueError("Configured bulb orientation differs from scene file")
        diagnostics.update(verified=True, scene_file=str(path), model_name=scenario["scene_model_name"])
    elif source == "manual_unvalidated":
        diagnostics["pose_source"] = "MANUAL_UNVALIDATED"
    else:
        raise ValueError(f"Unsupported offline pose source {source}")
    return pin.SE3(configured_quaternion.matrix(), configured_position), diagnostics


def compute_pregrasp_target(scenario: Dict[str, Any], bulb_pose: pin.SE3) -> Tuple[pin.SE3, Dict[str, Any]]:
    """Build world pregrasp pose from bulb-local axis, distance, and alignment."""
    bulb_axis = np.asarray(scenario["bulb_axis_local"], dtype=float)
    bulb_axis /= np.linalg.norm(bulb_axis)
    gripper_axis = np.asarray(scenario["gripper_axis_local"], dtype=float)
    gripper_axis /= np.linalg.norm(gripper_axis)
    distance = float(scenario["pregrasp_distance_m"])
    alignment = np.asarray(scenario["target_alignment_matrix"], dtype=float)
    if not np.allclose(alignment.T @ alignment, np.eye(3), atol=1e-9) or np.linalg.det(alignment) < 0.0:
        raise ValueError("target_alignment_matrix must be a proper rotation")
    local_offset = -distance * bulb_axis
    local = pin.SE3(alignment, local_offset)
    target = bulb_pose * local
    alignment_error = float(np.linalg.norm(
        target.rotation @ gripper_axis - bulb_pose.rotation @ bulb_axis))
    return target, {
        "bulb_axis_local": bulb_axis.tolist(), "gripper_axis_local": gripper_axis.tolist(),
        "offset_bulb_frame": local_offset.tolist(), "distance_m": distance,
        "axis_alignment_error": alignment_error,
    }


def initial_configuration(robot: UamModel, scenario: Dict[str, Any]) -> np.ndarray:
    q = robot.neutral_configuration(scenario["initial_joint_positions"])
    q[:3] = np.asarray(scenario["initial_base_pose"]["position"], dtype=float)
    q[3:7] = np.asarray(scenario["initial_base_pose"]["quaternion_xyzw"], dtype=float)
    return pin.normalize(robot.model, q)


def solve_terminal_ik(robot: UamModel, scenario: Dict[str, Any], task_joints: Dict[str, Any],
                      target: pin.SE3) -> Tuple[np.ndarray, Dict[str, Any]]:
    """Run manifold damped least-squares IK over explicitly active joints."""
    q = initial_configuration(robot, scenario)
    q[:3] = np.asarray(scenario["base_seed_pose"]["position"], dtype=float)
    q[3:7] = np.asarray(scenario["base_seed_pose"]["quaternion_xyzw"], dtype=float)
    for name, value in (task_joints.get("fixed_pregrasp_joint_positions") or {}).items():
        if not robot.model.existJointName(name):
            continue
        joint = robot.model.joints[robot.model.getJointId(name)]
        q[joint.idx_q] = float(value)
    active = [robot.model.getJointId(name) for name in task_joints["trajectory_active_joints"]]
    velocity_indices = [robot.model.joints[joint_id].idx_v for joint_id in active]
    settings = scenario["ik"]
    history = []
    for iteration in range(int(settings["max_iterations"])):
        data = robot.model.createData()
        pin.forwardKinematics(robot.model, data, q)
        pin.updateFramePlacements(robot.model, data)
        current = data.oMf[robot.end_effector_frame_id]
        position_error = target.translation - current.translation
        rotation_error = pin.log3(target.rotation @ current.rotation.T)
        error = np.concatenate((position_error, rotation_error))
        history.append([float(np.linalg.norm(position_error)), float(np.linalg.norm(rotation_error))])
        if history[-1][0] <= float(settings["position_tolerance_m"]) and history[-1][1] <= float(settings["orientation_tolerance_rad"]):
            break
        jacobian = pin.computeFrameJacobian(
            robot.model, data, q, robot.end_effector_frame_id,
            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
        active_jacobian = jacobian[:, velocity_indices]
        damping = float(settings["damping"])
        delta_active = active_jacobian.T @ np.linalg.solve(
            active_jacobian @ active_jacobian.T + damping ** 2 * np.eye(6), error)
        tangent = np.zeros(robot.model.nv)
        tangent[velocity_indices] = float(settings["step_size"]) * delta_active
        q = pin.integrate(robot.model, q, tangent)
        for joint_id in active:
            joint = robot.model.joints[joint_id]
            q[joint.idx_q] = np.clip(
                q[joint.idx_q], robot.model.lowerPositionLimit[joint.idx_q],
                robot.model.upperPositionLimit[joint.idx_q])
    success = bool(history[-1][0] <= float(settings["position_tolerance_m"])
                   and history[-1][1] <= float(settings["orientation_tolerance_rad"]))
    report = {
        "success": success, "status": "PASS" if success else "IK_SEED_UNREACHABLE",
        "iterations": len(history), "position_error_m": history[-1][0],
        "orientation_error_rad": history[-1][1], "active_joints": task_joints["trajectory_active_joints"],
        "fixed_joints": task_joints["fixed_pregrasp_joint_positions"],
        "history": history,
    }
    if not success:
        raise IKSeedUnreachable(str(report))
    return q, report
