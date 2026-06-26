import math
from typing import Iterable, Tuple

import numpy as np


def _as3(value: Iterable[float]) -> np.ndarray:
    arr = np.asarray(value, dtype=float).reshape(3)
    if not np.all(np.isfinite(arr)):
        raise ValueError("vector contains non-finite values")
    return arr


def enu_to_ned_position(v_enu: Iterable[float]) -> np.ndarray:
    x, y, z = _as3(v_enu)
    return np.array([y, x, -z], dtype=float)


def ned_to_enu_position(v_ned: Iterable[float]) -> np.ndarray:
    x, y, z = _as3(v_ned)
    return np.array([y, x, -z], dtype=float)


enu_to_ned_velocity = enu_to_ned_position
ned_to_enu_velocity = ned_to_enu_position


def flu_to_frd_body_rate(rate_flu: Iterable[float]) -> np.ndarray:
    p, q, r = _as3(rate_flu)
    return np.array([p, -q, -r], dtype=float)


def frd_to_flu_body_rate(rate_frd: Iterable[float]) -> np.ndarray:
    p, q, r = _as3(rate_frd)
    return np.array([p, -q, -r], dtype=float)


def quat_xyzw_normalize(q: Iterable[float]) -> np.ndarray:
    arr = np.asarray(q, dtype=float).reshape(4)
    n = np.linalg.norm(arr)
    if n <= 0.0 or not np.isfinite(n):
        raise ValueError("invalid quaternion")
    return arr / n


def quat_xyzw_to_matrix(q: Iterable[float]) -> np.ndarray:
    x, y, z, w = quat_xyzw_normalize(q)
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
        [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
        [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
    ], dtype=float)


def matrix_to_euler_rpy(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])
    singular = sy < 1e-9
    if not singular:
        roll = math.atan2(R[2, 1], R[2, 2])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = math.atan2(R[1, 0], R[0, 0])
    else:
        roll = math.atan2(-R[1, 2], R[1, 1])
        pitch = math.atan2(-R[2, 0], sy)
        yaw = 0.0
    return np.array([roll, pitch, yaw], dtype=float)


def quat_xyzw_to_euler_rpy(q: Iterable[float]) -> np.ndarray:
    return matrix_to_euler_rpy(quat_xyzw_to_matrix(q))


def slerp_xyzw(q0: Iterable[float], q1: Iterable[float], alpha: float) -> np.ndarray:
    q0 = quat_xyzw_normalize(q0)
    q1 = quat_xyzw_normalize(q1)
    alpha = float(alpha)
    dot = float(np.dot(q0, q1))
    if dot < 0.0:
        q1 = -q1
        dot = -dot
    dot = min(1.0, max(-1.0, dot))
    if dot > 0.9995:
        return quat_xyzw_normalize(q0 + alpha * (q1 - q0))
    theta0 = math.acos(dot)
    sin_theta0 = math.sin(theta0)
    theta = theta0 * alpha
    s0 = math.sin(theta0 - theta) / sin_theta0
    s1 = math.sin(theta) / sin_theta0
    return quat_xyzw_normalize(s0 * q0 + s1 * q1)


def body_linear_velocity_to_world(quaternion_xyzw: Iterable[float],
                                  velocity_body: Iterable[float]) -> np.ndarray:
    return quat_xyzw_to_matrix(quaternion_xyzw).dot(_as3(velocity_body))


def rotation_log(R: np.ndarray) -> np.ndarray:
    R = np.asarray(R, dtype=float).reshape(3, 3)
    cos_angle = (np.trace(R) - 1.0) * 0.5
    cos_angle = min(1.0, max(-1.0, float(cos_angle)))
    angle = math.acos(cos_angle)
    if angle < 1e-9:
        return np.zeros(3)
    skew = (R - R.T) / (2.0 * math.sin(angle))
    return angle * np.array([skew[2, 1], skew[0, 2], skew[1, 0]], dtype=float)
