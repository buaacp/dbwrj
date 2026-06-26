from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np


@dataclass
class TrajectorySnapshot:
    t0: float
    dt: float
    states: np.ndarray
    controls: np.ndarray
    total_thrust_reference: np.ndarray
    body_rate_reference: np.ndarray
    joint_position_reference: np.ndarray
    joint_velocity_reference: np.ndarray
    solve_time: float
    converged: bool
    terminal_metrics: Dict[str, Any] = field(default_factory=dict)
    q_names: List[str] = field(default_factory=list)
    v_names: List[str] = field(default_factory=list)
    control_names: List[str] = field(default_factory=list)
    target_translation: Optional[np.ndarray] = None
    target_rotation: Optional[np.ndarray] = None
    version: int = 0

    @property
    def duration(self) -> float:
        if self.states.shape[0] <= 1:
            return 0.0
        return float(self.dt) * float(self.states.shape[0] - 1)

    @property
    def arm_joint_count(self) -> int:
        return int(self.joint_position_reference.shape[1])


@dataclass
class RuntimeReference:
    t: float
    position_w: np.ndarray
    velocity_w: np.ndarray
    quaternion_xyzw: np.ndarray
    euler_rpy: np.ndarray
    body_rate: np.ndarray
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    command: np.ndarray
