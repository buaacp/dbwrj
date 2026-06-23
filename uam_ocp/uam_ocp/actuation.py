"""Configuration-driven rotor and arm-joint actuation mappings."""

from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import pinocchio as pin

from .model_loader import MODULE_ROOT, UamModel, load_yaml


def rotation_from_z(direction: np.ndarray) -> np.ndarray:
    """Return a proper rotation whose local z axis equals direction."""
    z = np.asarray(direction, dtype=float)
    z /= np.linalg.norm(z)
    helper = np.array([1.0, 0.0, 0.0]) if abs(z[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    y = np.cross(z, helper)
    y /= np.linalg.norm(y)
    x = np.cross(y, z)
    return np.column_stack((x, y, z))


class UamActuation:
    """Map physical rotor thrusts and named joint torques to generalized force."""

    def __init__(self, robot: UamModel, config_path: Optional[Path] = None):
        self.robot = robot
        self.config: Dict[str, Any] = load_yaml(
            config_path or MODULE_ROOT / "config" / "uam_actuation.yaml")
        self.rotors = sorted(self.config["rotors"], key=lambda item: int(item["id"]))
        if [int(item["id"]) for item in self.rotors] != list(range(len(self.rotors))):
            raise ValueError("Rotor IDs must be contiguous and start at zero")
        self.joint_names = [joint.name for joint in robot.arm_joints]
        configured = self.config["joint_torque_limits"]
        missing = sorted(set(self.joint_names) - set(configured))
        extra = sorted(set(configured) - set(self.joint_names))
        if missing or extra:
            raise ValueError(f"Joint torque YAML mismatch: missing={missing}, extra={extra}")
        self.n_rotors = len(self.rotors)
        self.nu = self.n_rotors + robot.n_arm
        self.mapping = self._build_mapping()

    def _build_mapping(self) -> np.ndarray:
        mapping = np.zeros((self.robot.model.nv, self.nu))
        for column, rotor in enumerate(self.rotors):
            direction = np.asarray(rotor["direction"], dtype=float)
            direction /= np.linalg.norm(direction)
            position = np.asarray(rotor["position"], dtype=float)
            reaction_sign = -1.0 if str(rotor["spin"]).lower() == "ccw" else 1.0
            mapping[:3, column] = direction
            mapping[3:6, column] = (
                np.cross(position, direction)
                + reaction_sign * float(rotor["torque_per_thrust"]) * direction)
        for offset, joint in enumerate(self.robot.arm_joints):
            if joint.nv != 1:
                raise ValueError(f"Only scalar independent joints are supported: {joint.name}")
            mapping[joint.idx_v, self.n_rotors + offset] = 1.0
        return mapping

    def rotor_thrust_to_wrench(self, thrusts: np.ndarray) -> np.ndarray:
        """Map rotor thrust vector to body [force, moment]."""
        thrusts = np.asarray(thrusts, dtype=float).reshape(self.n_rotors)
        return self.mapping[:6, :self.n_rotors] @ thrusts

    def physical_control_to_generalized_torque(self, control: np.ndarray) -> np.ndarray:
        """Map [rotor thrusts, independent arm torques] to Pinocchio tau."""
        control = np.asarray(control, dtype=float).reshape(self.nu)
        return self.mapping @ control

    def control_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Return YAML-sourced physical lower and upper bounds."""
        lower = [float(item["min_thrust"]) for item in self.rotors]
        upper = [float(item["max_thrust"]) for item in self.rotors]
        limits = self.config["joint_torque_limits"]
        lower.extend(float(limits[name]["lower"]) for name in self.joint_names)
        upper.extend(float(limits[name]["upper"]) for name in self.joint_names)
        return np.asarray(lower), np.asarray(upper)

    def equal_hover_control(self, gravity: float = 9.81) -> np.ndarray:
        """Use total URDF mass and equal rotor sharing; arm torque is zero."""
        value = self.robot.total_mass * float(gravity) / self.n_rotors
        control = np.zeros(self.nu)
        control[:self.n_rotors] = value
        return control

    def gravity_compensated_hover_control(self, q: np.ndarray) -> np.ndarray:
        """Return strict configuration trim through the canonical trim solver."""
        from .static_trim import StaticTrimSolver
        result = StaticTrimSolver(self.robot, self).solve_trim(q)
        if not result.strict_feasible:
            raise RuntimeError(
                f"Strict static trim failed with status {result.status}: "
                f"residual={np.linalg.norm(result.generalized_force_residual, ord=np.inf):.3e}")
        return result.u_eq.copy()

    def crocoddyl_model(self):
        """Build and numerically verify Crocoddyl's native mixed actuation."""
        import crocoddyl
        thrusters: List[Any] = []
        for rotor in self.rotors:
            pose = pin.SE3(
                rotation_from_z(np.asarray(rotor["direction"], dtype=float)),
                np.asarray(rotor["position"], dtype=float))
            kind = (crocoddyl.ThrusterType.CCW if str(rotor["spin"]).lower() == "ccw"
                    else crocoddyl.ThrusterType.CW)
            thrusters.append(crocoddyl.Thruster(
                pose, float(rotor["torque_per_thrust"]), kind,
                float(rotor["min_thrust"]), float(rotor["max_thrust"])))
        model = crocoddyl.ActuationModelFloatingBaseThrusters(self.robot.state, thrusters)
        native_mapping = np.asarray(model.Wthrust)
        if model.nu != self.nu or native_mapping.shape != self.mapping.shape:
            raise RuntimeError(
                f"Crocoddyl actuation dimensions mismatch: nu={model.nu}, W={native_mapping.shape}, expected={self.mapping.shape}")
        if not np.allclose(native_mapping, self.mapping, atol=1e-12):
            raise RuntimeError(f"Crocoddyl mapping differs from physical mapping:\n{native_mapping - self.mapping}")
        return model
