"""Simulation-time quintic arm reference profiles."""

import math
import numpy as np


def quintic_scale(elapsed_s, duration_s):
    """Return quintic position scale and its time derivative."""
    if duration_s <= 0.0:
        raise ValueError("duration_s must be positive")
    r = min(1.0, max(0.0, float(elapsed_s) / float(duration_s)))
    s = 10.0 * r ** 3 - 15.0 * r ** 4 + 6.0 * r ** 5
    sd = (30.0 * r ** 2 - 60.0 * r ** 3 + 30.0 * r ** 4) / duration_s
    return s, sd


def quintic_blend(elapsed_s, duration_s):
    """Return a clamped zero-end-velocity quintic blend in [0, 1]."""
    return quintic_scale(elapsed_s, max(float(duration_s), 1.0e-6))[0]


def takeoff_reference(p0, p1, elapsed_s, duration_s):
    """Interpolate a world-frame takeoff position using simulation time."""
    start = np.asarray(p0, dtype=float).reshape(3)
    target = np.asarray(p1, dtype=float).reshape(3)
    return start + quintic_blend(elapsed_s, duration_s) * (target - start)


def takeoff_abort_reason(actual_position, reference_position, initial_position,
                         elapsed_s, no_ascent_check_after_s,
                         min_ascent_after_check_m,
                         transition_position_error_m):
    """Classify takeoff tracking faults without changing thresholds."""
    actual = np.asarray(actual_position, dtype=float).reshape(3)
    reference = np.asarray(reference_position, dtype=float).reshape(3)
    initial = np.asarray(initial_position, dtype=float).reshape(3)
    actual_ascent = float(actual[2] - initial[2])
    reference_ascent = float(reference[2] - initial[2])
    if (float(elapsed_s) > float(no_ascent_check_after_s) and
            actual_ascent < float(min_ascent_after_check_m) and
            reference_ascent >= float(min_ascent_after_check_m)):
        return "TAKEOFF_NO_ASCENT"
    if np.linalg.norm(actual - reference) > float(transition_position_error_m):
        return "TAKEOFF_TRANSITION_POSITION_ERROR"
    return None


def post_offboard_state(initial_to_hold_distance_m, min_transition_distance_m):
    """Select the mandatory post-confirmation takeoff state."""
    return ("TAKEOFF_HOLD" if initial_to_hold_distance_m < min_transition_distance_m
            else "TAKEOFF_TRANSITION")


def takeoff_hold_ready(position_error_m, speed_mps, roll_rad, pitch_rad,
                       max_joint_error_rad):
    """Evaluate instantaneous TAKEOFF_HOLD readiness thresholds."""
    return (position_error_m < 0.05 and speed_mps < 0.05
            and abs(roll_rad) < math.radians(10.0)
            and abs(pitch_rad) < math.radians(10.0)
            and max_joint_error_rad < math.radians(3.0))


def update_ready_since(sim_time_s, instant_ready, ready_since):
    """Track the start of a continuous readiness interval."""
    if not instant_ready:
        return None
    return float(sim_time_s) if ready_since is None else float(ready_since)


def ready_duration_satisfied(sim_time_s, ready_since, required_duration_s):
    return (ready_since is not None
            and float(sim_time_s) - float(ready_since) >= float(required_duration_s))


def post_telemetry_state(neutralize_enabled):
    """Select the pre-flight arm preparation state after telemetry readiness."""
    return "ARM_NEUTRALIZE" if neutralize_enabled else "ARM_AND_OFFBOARD"


def neutralization_timed_out(elapsed_s, timeout_s):
    return float(elapsed_s) > float(timeout_s)


def required_duration(q0, q1, velocity_limits, requested_s):
    """Increase duration so the quintic peak joint speed respects limits."""
    q0 = np.asarray(q0, dtype=float)
    q1 = np.asarray(q1, dtype=float)
    limits = np.asarray(velocity_limits, dtype=float)
    if q0.shape != q1.shape or q0.shape != limits.shape:
        raise ValueError("profile vectors must have identical shapes")
    if np.any(limits <= 0.0):
        raise ValueError("velocity limits must be positive")
    # max(ds/dr)=1.875 at r=0.5, hence max(qdot)=1.875*dq/T.
    minimum = float(np.max(1.875 * np.abs(q1 - q0) / limits))
    return max(float(requested_s), minimum)


def sample_profile(q0, q1, elapsed_s, duration_s):
    """Sample position and velocity of a multi-joint quintic profile."""
    q0 = np.asarray(q0, dtype=float)
    delta = np.asarray(q1, dtype=float) - q0
    s, sd = quintic_scale(elapsed_s, duration_s)
    return q0 + s * delta, sd * delta


def deg(value):
    return float(value) * math.pi / 180.0
