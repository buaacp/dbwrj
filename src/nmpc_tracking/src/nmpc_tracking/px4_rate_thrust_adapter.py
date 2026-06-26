from typing import Iterable, Optional

import numpy as np


class ThrustMapper:
    def __init__(self, mass_kg: float, gravity_mps2: float, hover_thrust_norm: float,
                 min_norm: float = 0.0, max_norm: float = 1.0):
        self.mass_kg = float(mass_kg)
        self.gravity_mps2 = abs(float(gravity_mps2))
        self.hover_thrust_norm = float(hover_thrust_norm)
        self.min_norm = float(min_norm)
        self.max_norm = float(max_norm)
        if self.mass_kg <= 0.0:
            raise ValueError("mass_kg must be positive")
        if self.gravity_mps2 <= 0.0:
            raise ValueError("gravity_mps2 must be positive")
        if not (0.0 < self.hover_thrust_norm <= 1.0):
            raise ValueError("hover_thrust_norm must be in (0, 1]")
        if self.min_norm < 0.0 or self.max_norm > 1.0 or self.min_norm > self.max_norm:
            raise ValueError("invalid normalized thrust bounds")

    @property
    def hover_force_n(self) -> float:
        return self.mass_kg * self.gravity_mps2

    def force_to_normalized(self, thrust_n: float) -> float:
        raw = self.hover_thrust_norm * float(thrust_n) / self.hover_force_n
        return float(np.clip(raw, self.min_norm, self.max_norm))


class Px4RateThrustAdapter:
    def __init__(self, thrust_mapper: ThrustMapper, body_rate_frame: str = "FLU"):
        self.thrust_mapper = thrust_mapper
        self.body_rate_frame = body_rate_frame

    def make_attitude_target(self, thrust_n: float, body_rate: Iterable[float],
                             stamp: Optional[object] = None):
        try:
            from mavros_msgs.msg import AttitudeTarget
        except Exception as exc:
            raise RuntimeError("mavros_msgs/AttitudeTarget is not available") from exc
        msg = AttitudeTarget()
        if stamp is not None:
            msg.header.stamp = stamp
        msg.type_mask = AttitudeTarget.IGNORE_ATTITUDE
        rate = np.asarray(body_rate, dtype=float).reshape(3)
        msg.body_rate.x = float(rate[0])
        msg.body_rate.y = float(rate[1])
        msg.body_rate.z = float(rate[2])
        msg.thrust = self.thrust_mapper.force_to_normalized(float(thrust_n))
        return msg
