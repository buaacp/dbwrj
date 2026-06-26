import bisect
from typing import Optional

import numpy as np

from .frames import body_linear_velocity_to_world, quat_xyzw_to_euler_rpy, slerp_xyzw
from .robot_layout import LEGACY_6DOF_MESSAGE
from .trajectory_types import RuntimeReference, TrajectorySnapshot


def _safe_npz(path: str):
    return np.load(path, allow_pickle=False)


def _cubic_hermite(y0, y1, dy0, dy1, h, alpha):
    a = float(alpha)
    h00 = 2.0 * a ** 3 - 3.0 * a ** 2 + 1.0
    h10 = a ** 3 - 2.0 * a ** 2 + a
    h01 = -2.0 * a ** 3 + 3.0 * a ** 2
    h11 = a ** 3 - a ** 2
    return h00 * y0 + h10 * h * dy0 + h01 * y1 + h11 * h * dy1


def _linear(y0, y1, alpha):
    return (1.0 - alpha) * y0 + alpha * y1


class TrajectoryReference:
    def __init__(self, snapshot: TrajectorySnapshot, hold_after_s: float = 5.0):
        self.snapshot = snapshot
        self.hold_after_s = float(hold_after_s)
        self.times = np.arange(snapshot.states.shape[0], dtype=float) * float(snapshot.dt)

    @classmethod
    def from_npz(cls, path: str, hold_after_s: float = 5.0, version: int = 0):
        data = _safe_npz(path)
        states = np.asarray(data["states"], dtype=float)
        controls = np.asarray(data["controls"], dtype=float)
        q_names = [str(v) for v in data["q_names"].tolist()]
        v_names = [str(v) for v in data["v_names"].tolist()]
        control_names = [str(v) for v in data["control_names"].tolist()]
        arm_names = q_names[7:]
        if len(arm_names) == 6 and (
                "shoulder_pan_joint" in arm_names or "left_knuckle_joint" in arm_names):
            raise ValueError(LEGACY_6DOF_MESSAGE)
        dt = float(np.asarray(data["dt_s"]).reshape(-1)[0])
        nq = len(q_names)
        arm_q = states[:, 7:nq]
        arm_dq = states[:, nq + 6:]
        rotor_controls = controls[:, :4] if controls.size else np.zeros((max(0, states.shape[0] - 1), 4))
        total_thrust = np.sum(rotor_controls, axis=1)
        total_thrust = np.r_[total_thrust, total_thrust[-1] if total_thrust.size else 0.0]
        body_rate = states[:, nq + 3:nq + 6]
        target_translation = np.asarray(data["target_translation"], dtype=float) if "target_translation" in data.files else None
        target_rotation = np.asarray(data["target_rotation"], dtype=float) if "target_rotation" in data.files else None
        snapshot = TrajectorySnapshot(
            t0=0.0, dt=dt, states=states, controls=controls,
            total_thrust_reference=total_thrust,
            body_rate_reference=body_rate,
            joint_position_reference=arm_q,
            joint_velocity_reference=arm_dq,
            solve_time=0.0, converged=True, terminal_metrics={},
            q_names=q_names, v_names=v_names, control_names=control_names,
            target_translation=target_translation, target_rotation=target_rotation,
            version=version,
        )
        return cls(snapshot, hold_after_s=hold_after_s)

    def sample(self, t: float) -> RuntimeReference:
        t = float(t)
        duration = self.snapshot.duration
        if t >= duration:
            return self._terminal_sample(min(t, duration))
        if t <= 0.0:
            return self._node_sample(0, 0.0)
        i = bisect.bisect_right(self.times, t) - 1
        i = max(0, min(i, len(self.times) - 2))
        h = self.times[i + 1] - self.times[i]
        alpha = (t - self.times[i]) / h
        if abs(alpha) < 1e-12:
            return self._node_sample(i, t)
        return self._interpolate(i, alpha, h, t)

    def _state_parts(self, i: int):
        s = self.snapshot.states[i]
        nq = len(self.snapshot.q_names)
        return {
            "p": s[:3],
            "q": s[3:7],
            "v_body": s[nq:nq + 3],
            "w_body": s[nq + 3:nq + 6],
            "qa": s[7:nq],
            "dqa": s[nq + 6:],
        }

    def _node_sample(self, i: int, t: float) -> RuntimeReference:
        p = self._state_parts(i)
        v_w = body_linear_velocity_to_world(p["q"], p["v_body"])
        cmd = np.r_[self.snapshot.total_thrust_reference[i], p["w_body"], p["dqa"]]
        return RuntimeReference(
            t=t, position_w=p["p"].copy(), velocity_w=v_w,
            quaternion_xyzw=p["q"].copy(), euler_rpy=quat_xyzw_to_euler_rpy(p["q"]),
            body_rate=p["w_body"].copy(), joint_position=p["qa"].copy(),
            joint_velocity=p["dqa"].copy(), command=cmd,
        )

    def _interpolate(self, i: int, alpha: float, h: float, t: float) -> RuntimeReference:
        a = self._state_parts(i)
        b = self._state_parts(i + 1)
        va_w = body_linear_velocity_to_world(a["q"], a["v_body"])
        vb_w = body_linear_velocity_to_world(b["q"], b["v_body"])
        pos = _cubic_hermite(a["p"], b["p"], va_w, vb_w, h, alpha)
        quat = slerp_xyzw(a["q"], b["q"], alpha)
        qa = _cubic_hermite(a["qa"], b["qa"], a["dqa"], b["dqa"], h, alpha)
        vel = _linear(va_w, vb_w, alpha)
        w = _linear(a["w_body"], b["w_body"], alpha)
        dqa = _linear(a["dqa"], b["dqa"], alpha)
        thrust = _linear(self.snapshot.total_thrust_reference[i],
                         self.snapshot.total_thrust_reference[i + 1], alpha)
        return RuntimeReference(
            t=t, position_w=pos, velocity_w=vel, quaternion_xyzw=quat,
            euler_rpy=quat_xyzw_to_euler_rpy(quat), body_rate=w,
            joint_position=qa, joint_velocity=dqa,
            command=np.r_[thrust, w, dqa],
        )

    def _terminal_sample(self, t: float) -> RuntimeReference:
        ref = self._node_sample(self.snapshot.states.shape[0] - 1, t)
        ref.velocity_w[:] = 0.0
        ref.body_rate[:] = 0.0
        ref.joint_velocity[:] = 0.0
        ref.command[1:] = 0.0
        return ref

    def resample(self, dt: float, until_s: Optional[float] = None) -> list:
        end = self.snapshot.duration if until_s is None else float(until_s)
        count = int(round(end / float(dt))) + 1
        return [self.sample(k * float(dt)) for k in range(count)]
