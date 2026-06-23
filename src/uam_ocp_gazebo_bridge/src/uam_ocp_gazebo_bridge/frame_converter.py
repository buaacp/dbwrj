"""Resolved world-to-MAVROS frame conversion for G1.

The current bridge accepts ROS ENU values on MAVROS setpoint_raw/local. MAVROS
owns the ENU-to-PX4-NED conversion, so this module intentionally applies the
identity transform and centralizes that decision.
"""

import math
import numpy as np


class FrameConverter(object):
    """Convert Gazebo/ROS world ENU references to MAVROS local ENU payloads."""

    source_frame = "WORLD_ENU"
    destination_frame = "MAVROS_LOCAL_ENU"

    @staticmethod
    def _vector(value):
        result = np.asarray(value, dtype=float).reshape(3)
        if not np.all(np.isfinite(result)):
            raise ValueError("frame input contains non-finite values")
        return result.copy()

    def world_to_setpoint_position(self, value):
        return self._vector(value)

    def world_to_setpoint_velocity(self, value):
        return self._vector(value)

    def world_to_setpoint_acceleration(self, value):
        return self._vector(value)

    def world_to_setpoint_yaw(self, yaw):
        if not math.isfinite(float(yaw)):
            raise ValueError("yaw must be finite")
        return math.atan2(math.sin(float(yaw)), math.cos(float(yaw)))
