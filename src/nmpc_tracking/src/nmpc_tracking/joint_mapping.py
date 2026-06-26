from typing import Dict, Iterable, List, Sequence

import numpy as np


class JointMapping:
    def __init__(self, joint_names: Sequence[str], velocity_signs: Sequence[float],
                 velocity_limits_radps: Sequence[float]):
        self.joint_names = list(joint_names)
        self.velocity_signs = np.asarray(velocity_signs, dtype=float)
        self.velocity_limits_radps = np.asarray(velocity_limits_radps, dtype=float)
        n = len(self.joint_names)
        if n == 0:
            raise ValueError("arm.joint_names must not be empty")
        if len(set(self.joint_names)) != n:
            raise ValueError("arm.joint_names contains duplicates")
        if self.velocity_signs.shape != (n,):
            raise ValueError("arm.velocity_signs length mismatch")
        if self.velocity_limits_radps.shape != (n,):
            raise ValueError("arm.velocity_limits_radps length mismatch")
        if np.any(self.velocity_limits_radps <= 0.0):
            raise ValueError("arm velocity limits must be positive")

    @property
    def size(self) -> int:
        return len(self.joint_names)

    def indices_for_joint_state(self, observed_names: Sequence[str]) -> List[int]:
        index = {name: i for i, name in enumerate(observed_names)}
        missing = [name for name in self.joint_names if name not in index]
        if missing:
            raise KeyError("joint_states missing joints: " + ", ".join(missing))
        return [index[name] for name in self.joint_names]

    def positions_from_joint_state(self, observed_names: Sequence[str],
                                   positions: Sequence[float]) -> np.ndarray:
        positions = np.asarray(positions, dtype=float)
        indices = self.indices_for_joint_state(observed_names)
        return positions[indices]

    def velocities_from_joint_state(self, observed_names: Sequence[str],
                                    velocities: Sequence[float]) -> np.ndarray:
        velocities = np.asarray(velocities, dtype=float)
        indices = self.indices_for_joint_state(observed_names)
        return velocities[indices]

    def command_to_message_order(self, command_radps: Iterable[float]) -> np.ndarray:
        cmd = np.asarray(command_radps, dtype=float).reshape(self.size)
        signed = cmd * self.velocity_signs
        return np.clip(signed, -self.velocity_limits_radps, self.velocity_limits_radps)

    def saturation_mask(self, command_radps: Iterable[float]) -> np.ndarray:
        cmd = np.asarray(command_radps, dtype=float).reshape(self.size)
        return np.abs(cmd * self.velocity_signs) > self.velocity_limits_radps


def mapping_from_config(config: Dict) -> JointMapping:
    arm = config.get("arm", config)
    limits = arm.get("velocity_limits_radps")
    if limits is None:
        from .robot_layout import load_robot_layout
        layout = load_robot_layout(config)
        limits = layout.velocity_limits.tolist()
    return JointMapping(
        arm["joint_names"],
        arm.get("velocity_signs", [1.0] * len(arm["joint_names"])),
        limits,
    )
