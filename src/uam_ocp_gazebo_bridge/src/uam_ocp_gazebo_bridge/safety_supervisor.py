"""Per-topic monotonic watchdog and physical safety checks for G1."""

import math
import numpy as np


ACTIVE_FLIGHT_STATES = (
    "TAKEOFF_TRANSITION", "TAKEOFF_HOLD", "ARM_DEPLOY", "CONFIGURATION_HOLD", "ARM_RETRACT", "FINAL_HOLD")
REQUIRED_TELEMETRY = ("mavros_state", "pose", "velocity", "imu", "joint_state")


class StartupTelemetryGate(object):
    """Wait for first telemetry frames without applying stale semantics."""

    def __init__(self, config, start_wall_time):
        self.start_wall_time = float(start_wall_time)
        self.telemetry_grace = float(config["initial_telemetry_grace_s"])
        self.state_grace = float(config["initial_state_grace_s"])
        self.required = list(REQUIRED_TELEMETRY)
        if bool(config["require_clock_before_execution"]):
            self.required.append("clock")
        self.first_seen = {}
        self.ready_wall_time = None

    def update(self, receive_times):
        for key in self.required:
            if key in receive_times:
                self.first_seen.setdefault(key, float(receive_times[key]))
        if self.ready() and self.ready_wall_time is None:
            self.ready_wall_time = max(self.first_seen.values())

    def missing(self):
        return [key for key in self.required if key not in self.first_seen]

    def ready(self):
        return not self.missing()

    def seen_flags(self):
        return dict(("seen_" + key, key in self.first_seen)
                    for key in ("mavros_state", "pose", "velocity", "imu", "joint_state", "clock"))

    def failure_reason(self, now):
        """Return a startup reason only after the configured grace expires."""
        elapsed = float(now) - self.start_wall_time
        missing = self.missing()
        if not missing:
            return None
        if "mavros_state" in missing and elapsed > self.state_grace:
            return "INITIAL_TELEMETRY_NOT_RECEIVED"
        if elapsed > self.telemetry_grace:
            return "INITIAL_CLOCK_NOT_RECEIVED" if missing == ["clock"] else "INITIAL_TELEMETRY_NOT_RECEIVED"
        return None

    def diagnostics(self, now):
        result = {}
        for key in self.required:
            value = self.first_seen.get(key)
            result[key + "_first_seen_wall_s"] = None if value is None else value - self.start_wall_time
        result["all_required_topics_seen"] = self.ready()
        result.update(self.seen_flags())
        end = self.ready_wall_time if self.ready_wall_time is not None else float(now)
        result["initial_telemetry_wait_s"] = max(0.0, end - self.start_wall_time)
        result["missing_topics"] = self.missing()
        return result


class SafetySupervisor(object):
    """Evaluate explicit faults immediately and confirm only low-rate staleness."""

    def __init__(self, safety, watchdog, absolute_safety=None):
        self.position_limit = float(safety["position_error_m"])
        self.attitude_limit = math.radians(float(safety["roll_pitch_deg"]))
        self.joint_limit = math.radians(float(safety["joint_tracking_error_deg"]))
        absolute_safety = absolute_safety or {
            "max_distance_from_initial_m": float("inf"),
            "max_altitude_above_initial_m": float("inf"),
            "min_altitude_below_initial_m": float("inf"),
            "max_roll_pitch_deg": safety["roll_pitch_deg"],
            "max_joint_tracking_error_deg": safety["joint_tracking_error_deg"],
        }
        self.absolute_distance_limit = float(absolute_safety["max_distance_from_initial_m"])
        self.absolute_above_limit = float(absolute_safety["max_altitude_above_initial_m"])
        self.absolute_below_limit = float(absolute_safety["min_altitude_below_initial_m"])
        self.attitude_limit = math.radians(float(absolute_safety["max_roll_pitch_deg"]))
        self.joint_limit = math.radians(float(absolute_safety["max_joint_tracking_error_deg"]))
        self.timeouts = {
            "mavros_state": float(watchdog["mavros_state_timeout_s"]),
            "pose": float(watchdog["pose_timeout_s"]),
            "velocity": float(watchdog["velocity_timeout_s"]),
            "imu": float(watchdog["imu_timeout_s"]),
            "joint_state": float(watchdog["joint_state_timeout_s"]),
            "clock": float(watchdog["clock_timeout_s"]),
            "setpoint_publish": float(watchdog["setpoint_publish_timeout_s"]),
        }
        self.confirm_cycles = int(watchdog["stale_confirm_cycles"])
        self.state_stale_count = 0

    def evaluate(self, wall_now, state, receive_times, position_error_m, roll_rad, pitch_rad,
                 joint_error, offboard, armed, startup_complete=True,
                 offboard_confirmed=True, armed_confirmed=True,
                 clock_expected=True, position_tracking_enabled=True,
                 joint_tracking_enabled=True,
                 position_error_reason="POSITION_ERROR", actual_position=None,
                 initial_position=None, position_error_limit_m=None,
                 joint_error_reason="JOINT_TRACKING_ERROR",
                 joint_error_limit_rad=None, joint_position=None,
                 joint_lower=None, joint_upper=None,
                 controller_ok=True):
        """Return the first abort reason, or None when all checks are healthy."""
        # PRESTREAM startup availability is owned by StartupTelemetryGate.
        if not startup_complete:
            self.state_stale_count = 0
            return None

        # Physical limit violations and controller faults are never delayed.
        tracking_limit = self.position_limit if position_error_limit_m is None else float(position_error_limit_m)
        joint_tracking_limit = self.joint_limit if joint_error_limit_rad is None else float(joint_error_limit_rad)
        immediate = [
            (position_tracking_enabled and float(position_error_m) > tracking_limit,
             position_error_reason),
            (max(abs(float(roll_rad)), abs(float(pitch_rad))) > self.attitude_limit,
             "BASE_ATTITUDE"),
            (joint_tracking_enabled and np.max(np.abs(joint_error)) > joint_tracking_limit,
             joint_error_reason),
            (not controller_ok, "ARM_CONTROLLER_ERROR"),
        ]
        for triggered, reason in immediate:
            if triggered:
                return reason

        if actual_position is not None and initial_position is not None:
            actual = np.asarray(actual_position, dtype=float)
            initial = np.asarray(initial_position, dtype=float)
            delta = actual - initial
            absolute = [
                (np.linalg.norm(delta) > self.absolute_distance_limit,
                 "ABSOLUTE_DISTANCE_FROM_INITIAL"),
                (delta[2] > self.absolute_above_limit, "ABSOLUTE_ALTITUDE_ABOVE_INITIAL"),
                (-delta[2] > self.absolute_below_limit, "ABSOLUTE_ALTITUDE_BELOW_INITIAL"),
            ]
            for triggered, reason in absolute:
                if triggered:
                    return reason

        if joint_position is not None and joint_lower is not None and joint_upper is not None:
            q = np.asarray(joint_position,dtype=float)
            if np.any(q < np.asarray(joint_lower,dtype=float)) or np.any(q > np.asarray(joint_upper,dtype=float)):
                return "JOINT_LIMIT_VIOLATION"

        # PRESTREAM is explicitly allowed to be unarmed and outside Offboard.
        if (offboard_confirmed and armed_confirmed and state in ACTIVE_FLIGHT_STATES
                and (not offboard or not armed)):
            return "MAVROS_STATE_MODE_OR_ARMING_LOST"

        # State staleness is meaningful only after both transitions were seen.
        if (state != "PRESTREAM_SETPOINTS" and offboard_confirmed and armed_confirmed
                and "mavros_state" in receive_times):
            state_age = self._age(wall_now, receive_times, "mavros_state")
            if state_age > self.timeouts["mavros_state"]:
                self.state_stale_count += 1
                if self.state_stale_count >= self.confirm_cycles:
                    return "MAVROS_STATE_STALE"
            else:
                self.state_stale_count = 0
        else:
            self.state_stale_count = 0

        # High-rate signals, clock, and setpoint publication fail immediately.
        high_rate = (
            ("pose", "POSE_STALE"),
            ("velocity", "VELOCITY_STALE"),
            ("imu", "IMU_STALE"),
            ("joint_state", "JOINT_STATE_STALE"),
            ("setpoint_publish", "SETPOINT_PUBLISH_STALE"),
        )
        for key, reason in high_rate:
            if key in receive_times and self._age(wall_now, receive_times, key) > self.timeouts[key]:
                return reason
        if (clock_expected and "clock" in receive_times
                and self._age(wall_now, receive_times, "clock") > self.timeouts["clock"]):
            return "SIM_CLOCK_STALE_OR_GAZEBO_PAUSED"
        return None

    @staticmethod
    def _age(now, timestamps, key):
        value = timestamps.get(key)
        return float("inf") if value is None else max(0.0, float(now) - float(value))


def assess_run(rows, thresholds, aborted):
    """Evaluate recorded telemetry using the G1 acceptance thresholds."""
    if not rows:
        return "NOT_RUN_ENVIRONMENT_UNAVAILABLE", {}
    target = [row for row in rows if row["state"] == "CONFIGURATION_HOLD"]
    if not target:
        return ("ABORTED" if aborted else "FAIL"), {}
    end_time = float(target[-1]["sim_time_s"])
    final = [row for row in target if float(row["sim_time_s"]) >= end_time - 3.0]
    metrics = {
        "peak_position_error_m": max(float(row["position_error_m"]) for row in rows),
        "final_hold_position_error_m": sum(float(row["position_error_m"]) for row in final) / len(final),
        "peak_roll_pitch_deg": math.degrees(max(max(abs(float(row["roll_rad"])), abs(float(row["pitch_rad"]))) for row in rows)),
        "final_speed_mps": max(float(row["speed_mps"]) for row in final),
        "max_joint_tracking_error_deg": math.degrees(max(float(row["max_joint_error_rad"]) for row in rows)),
        "offboard_remained_active": all(_as_bool(row["offboard"]) for row in rows
                                         if row["state"] in ACTIVE_FLIGHT_STATES),
    }
    passed = (
        metrics["peak_position_error_m"] <= thresholds["peak_position_error_m"] and
        metrics["final_hold_position_error_m"] <= thresholds["final_hold_position_error_m"] and
        metrics["peak_roll_pitch_deg"] <= thresholds["peak_roll_pitch_deg"] and
        metrics["final_speed_mps"] <= thresholds["final_speed_mps"] and
        metrics["max_joint_tracking_error_deg"] <= thresholds["max_joint_tracking_error_deg"] and
        (metrics["offboard_remained_active"] or not thresholds["offboard_must_remain_active"]) and
        (not aborted or not thresholds["abort_must_not_trigger"]))
    return ("PASS" if passed else ("ABORTED" if aborted else "FAIL")), metrics


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("true", "1", "yes")
