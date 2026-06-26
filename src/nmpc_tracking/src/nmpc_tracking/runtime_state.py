from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class MeasuredState:
    stamp: float
    position_w: np.ndarray
    velocity_w: np.ndarray
    quaternion_xyzw: np.ndarray
    body_rate: np.ndarray
    joint_position: np.ndarray
    joint_velocity: np.ndarray
    last_command: np.ndarray

    def finite(self) -> bool:
        arrays = [
            self.position_w, self.velocity_w, self.quaternion_xyzw, self.body_rate,
            self.joint_position, self.joint_velocity, self.last_command,
        ]
        return all(np.all(np.isfinite(a)) for a in arrays)


class TopicReadiness:
    def __init__(self):
        self.pose = False
        self.velocity = False
        self.state = False
        self.joints = False
        self.target = False

    def ready(self) -> bool:
        return self.pose and self.velocity and self.state and self.joints and self.target
