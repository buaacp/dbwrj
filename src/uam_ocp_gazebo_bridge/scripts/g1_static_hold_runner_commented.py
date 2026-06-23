#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""G1：Gazebo/PX4 静态机械臂构型保持实验。

【用途】
G1 用于验证“无人机固定悬停 + 机械臂保持某一指定构型”能否在
Gazebo + PX4 + MAVROS 的闭环中稳定执行。它不负责灯泡识别、抓取、
接触、拧螺纹或在线 Crocoddyl MPC。

【执行流程】
PRESTREAM_SETPOINTS
  -> ARM_NEUTRALIZE
  -> ARM_AND_OFFBOARD
  -> TAKEOFF_TRANSITION
  -> TAKEOFF_HOLD
  -> ARM_DEPLOY
  -> CONFIGURATION_HOLD
  -> ARM_RETRACT
  -> FINAL_HOLD
  -> COMPLETE

【核心设计约束】
1. 每一个控制周期只生成一个“当前活动无人机位置参考”和一个“当前活动
   关节参考”，发布、误差计算与日志必须共享同一个参考对象。
2. 位置与关节误差必须相对当前阶段的活动参考计算，不能在起飞或启动阶段
   直接拿最终悬停点 / 最终构型做硬中止判据。
3. 起飞轨迹与关节插值使用仿真时间（/clock）；服务重试和接口启动等待使用
   单调真实时间（time.monotonic）。两种时间域绝不直接相减。
4. 原 SafetySupervisor 继续负责通用遥测、姿态、setpoint 频率等安全检查；
   本脚本按阶段屏蔽不适用的跟踪误差，并独立处理 MAVROS state freshness，
   避免 Gazebo 低实时因子造成假 MAVROS_STATE_STALE。

【兼容性】
面向 ROS Melodic + Python 3.6：不使用 dataclass、类型注解或新的第三方依赖。
"""

from __future__ import print_function

import argparse
from datetime import datetime
import math
from pathlib import Path
import subprocess
import threading
import time

import numpy as np
import rospy
import yaml
from mavros_msgs.msg import State as MavrosState
from rosgraph_msgs.msg import Clock

from uam_ocp_gazebo_bridge.arm_reference_interface import ArmReferenceInterface
from uam_ocp_gazebo_bridge.config_loader import load_yaml, resolve_g1_config
from uam_ocp_gazebo_bridge.frame_converter import FrameConverter
from uam_ocp_gazebo_bridge.hover_offboard_interface import HoverOffboardInterface
from uam_ocp_gazebo_bridge.result_writer import ResultWriter
from uam_ocp_gazebo_bridge.safety_supervisor import (
    SafetySupervisor,
    StartupTelemetryGate,
    assess_run,
)
from uam_ocp_gazebo_bridge.state_monitor import StateMonitor
from uam_ocp_gazebo_bridge.trajectory_profile import required_duration, sample_profile


# 状态名称必须集中维护：结果日志、离线分析器和单元测试都依赖这些字符串。
# 因此不要随意改名；若新增状态，也要同步更新分析与测试。
STATES = (
    "IDLE",
    "PRESTREAM_SETPOINTS",
    "ARM_NEUTRALIZE",
    "ARM_AND_OFFBOARD",
    "TAKEOFF_TRANSITION",
    "TAKEOFF_HOLD",
    "ARM_DEPLOY",
    "CONFIGURATION_HOLD",
    "ARM_RETRACT",
    "FINAL_HOLD",
    "COMPLETE",
    "ABORT",
)

# 所有中止原因统一定义为常量，并写入 result.yaml。
# 这样日志、单元测试和离线分析不会因字符串拼写不一致而失配。
ABORT_CONFIGURATION_UNRESOLVED = "CONFIGURATION_UNRESOLVED"
ABORT_PHASE_TIMEOUT = "PHASE_TIMEOUT"
ABORT_OFFBOARD_NOT_CONFIRMED = "OFFBOARD_NOT_CONFIRMED"
ABORT_ARMING_NOT_CONFIRMED = "ARMING_NOT_CONFIRMED"
ABORT_ARM_NEUTRALIZE_TRACKING = "ARM_NEUTRALIZE_TRACKING_ERROR"
ABORT_ARM_NEUTRALIZE_TIMEOUT = "ARM_NEUTRALIZE_TIMEOUT"
ABORT_TAKEOFF_TRANSITION_POSITION = "TAKEOFF_TRANSITION_POSITION_ERROR"
ABORT_CLOCK_STALE = "CLOCK_STALE"
ABORT_MAVROS_STATE_STALE = "MAVROS_STATE_STALE"
ABORT_OFFBOARD_LOST = "OFFBOARD_LOST"
ABORT_ARMING_LOST = "ARMING_LOST"
ABORT_ABSOLUTE_POSITION = "ABSOLUTE_POSITION_LIMIT"
ABORT_ABSOLUTE_ALTITUDE = "ABSOLUTE_ALTITUDE_LIMIT"
ABORT_ABSOLUTE_ATTITUDE = "ABSOLUTE_ATTITUDE_LIMIT"
ABORT_SAFETY_INTERFACE = "SAFETY_INTERFACE_ERROR"


class ActiveReference(object):
    """单个控制周期的完整“活动参考”。

    状态机先构造该对象，再统一用于：
    1) 发布 UAV 悬停/起飞参考；2) 发布机械臂关节参考；
    3) 计算安全误差；4) 记录 telemetry。
    因此命令、误差和日志始终指向同一个参考，不会出现“发布的是 A，
    但 watchdog 却按 B 判错”的问题。
    """

    def __init__(self, position, yaw_rad, joint_position, joint_velocity,
                 position_kind, joint_kind, position_tracking_enabled,
                 joint_tracking_enabled):
        self.position = np.asarray(position, dtype=float).copy()
        self.yaw_rad = float(yaw_rad)
        self.joint_position = np.asarray(joint_position, dtype=float).copy()
        self.joint_velocity = np.asarray(joint_velocity, dtype=float).copy()
        self.position_kind = str(position_kind)
        self.joint_kind = str(joint_kind)
        self.position_tracking_enabled = bool(position_tracking_enabled)
        self.joint_tracking_enabled = bool(joint_tracking_enabled)


def git_value(args, cwd):
    """Return a git value without allowing metadata collection to stop G1."""
    try:
        return subprocess.check_output(args, cwd=str(cwd)).decode().strip()
    except Exception:
        return "UNAVAILABLE"


def trim_reference(path, scenario):
    """Load the matching offline static-trim entry for result provenance only."""
    try:
        data = load_yaml(path)
    except Exception:
        return {
            "status": "UNRESOLVED_INTERFACE",
            "note": "Static trim result could not be loaded; never sent to PX4.",
        }
    entries = data.get("entries", [])
    candidates = [entry for entry in entries if entry.get("name") == scenario]
    return candidates[0] if candidates else {
        "status": "UNRESOLVED_INTERFACE",
        "note": "Static trim entry not found; never sent to PX4.",
    }


def quintic_blend(elapsed_s, duration_s):
    """Minimum-jerk scalar blend s in [0, 1] using simulation time.

    s(r) = 10 r^3 - 15 r^4 + 6 r^5.
    """
    duration = max(float(duration_s), 1e-9)
    ratio = min(max(float(elapsed_s) / duration, 0.0), 1.0)
    return 10.0 * ratio ** 3 - 15.0 * ratio ** 4 + 6.0 * ratio ** 5


def quintic_position(start, goal, elapsed_s, duration_s):
    """Return a min-jerk position between two three-dimensional points."""
    start = np.asarray(start, dtype=float)
    goal = np.asarray(goal, dtype=float)
    return start + quintic_blend(elapsed_s, duration_s) * (goal - start)


def nested_get(mapping, keys, default=None):
    """Read a nested dict safely; old YAML files remain valid."""
    value = mapping
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def topic_from_interfaces(topics, aliases, fallback=None):
    """Find a topic path while tolerating minor historical YAML key changes."""
    for alias in aliases:
        value = topics.get(alias)
        if isinstance(value, str) and value:
            return value
        if isinstance(value, dict):
            for nested_key in ("topic", "name", "path"):
                nested = value.get(nested_key)
                if isinstance(nested, str) and nested:
                    return nested
    return fallback


class RuntimeFreshness(object):
    """分别跟踪 MAVROS state 与 /clock 的新鲜度，且严格区分两套时间基准。

    StateMonitor 仍负责项目通用遥测。本类仅补充 G1 的特定问题：
    Gazebo 低于实时运行时，不能只用很短的 wall-time 阈值判定 MAVROS state
    失效，否则会把“仿真慢”误判为 “MAVROS 断流”。
    """

    def __init__(self, topics, watchdog_cfg):
        self._lock = threading.Lock()
        self._last_state_wall = None
        self._last_state_sim = None
        self._state_count = 0
        self._last_mode = ""
        self._last_armed = None
        self._last_connected = None

        self._last_clock_wall = None
        self._last_clock_sim = None
        self._clock_count = 0

        self.state_stale_sim_s = float(watchdog_cfg.get(
            "mavros_state_stale_sim_s", 6.0))
        self.state_hard_stale_wall_s = float(watchdog_cfg.get(
            "mavros_state_hard_stale_wall_s", 20.0))
        self.state_stale_consecutive_checks = int(watchdog_cfg.get(
            "mavros_state_stale_consecutive_checks", 2))
        self.clock_stale_wall_s = float(watchdog_cfg.get(
            "clock_stale_wall_s", 5.0))
        self._consecutive_state_stale = 0

        state_topic = topic_from_interfaces(
            topics,
            ("mavros_state", "state", "vehicle_state"),
            None,
        )
        clock_topic = topic_from_interfaces(topics, ("clock",), "/clock")

        self._state_sub = None
        self._clock_sub = None
        if state_topic:
            self._state_sub = rospy.Subscriber(
                state_topic, MavrosState, self._state_callback, queue_size=10)
        else:
            rospy.logwarn("G1 runtime freshness: MAVROS state topic not found in interfaces YAML")
        if clock_topic:
            self._clock_sub = rospy.Subscriber(
                clock_topic, Clock, self._clock_callback, queue_size=10)

    def _state_callback(self, message):
        wall_now = time.monotonic()
        sim_now = rospy.get_time()
        with self._lock:
            self._last_state_wall = wall_now
            self._last_state_sim = sim_now
            self._state_count += 1
            self._last_mode = str(message.mode)
            self._last_armed = bool(message.armed)
            self._last_connected = bool(message.connected)

    def _clock_callback(self, message):
        wall_now = time.monotonic()
        with self._lock:
            self._last_clock_wall = wall_now
            self._last_clock_sim = message.clock.to_sec()
            self._clock_count += 1

    def snapshot(self, wall_now, sim_now):
        """Return only same-domain age calculations (never wall minus sim)."""
        with self._lock:
            state_wall_age = None if self._last_state_wall is None else wall_now - self._last_state_wall
            state_sim_age = None if self._last_state_sim is None else sim_now - self._last_state_sim
            clock_wall_age = None if self._last_clock_wall is None else wall_now - self._last_clock_wall
            return {
                "state_last_wall_s": self._last_state_wall,
                "state_last_sim_s": self._last_state_sim,
                "state_age_wall_s": state_wall_age,
                "state_age_sim_s": state_sim_age,
                "state_receive_count": self._state_count,
                "mode": self._last_mode,
                "armed": self._last_armed,
                "connected": self._last_connected,
                "clock_last_wall_s": self._last_clock_wall,
                "clock_last_sim_s": self._last_clock_sim,
                "clock_age_wall_s": clock_wall_age,
                "clock_receive_count": self._clock_count,
            }

    def state_receive_count(self):
        """Return the current MAVROS-state callback count safely."""
        with self._lock:
            return int(self._state_count)

    def evaluate(self, wall_now, sim_now, offboard_confirmed, armed_confirmed,
                 confirmation_state_count):
        """Return a dedicated freshness/loss abort reason, or None.

        `MAVROS_STATE_STALE` starts only after OFFBOARD and arming were
        confirmed.  Explicit newer state messages showing OFFBOARD/arming loss
        are acted upon immediately.
        """
        info = self.snapshot(wall_now, sim_now)

        # /clock 停止推进是独立故障，优先报告 CLOCK_STALE。
        # 不能仅因为仿真时间冻结，错误地报告 MAVROS_STATE_STALE。
        if info["clock_receive_count"] > 0 and info["clock_age_wall_s"] is not None:
            if info["clock_age_wall_s"] > self.clock_stale_wall_s:
                return ABORT_CLOCK_STALE, info

        if not (offboard_confirmed and armed_confirmed):
            self._consecutive_state_stale = 0
            return None, info

        # 新收到的 state 如果明确表明已退出 OFFBOARD 或失去解锁，
        # 其证据强度高于“长时间未更新”，应立即中止。
        # 但只检查确认 Offboard+arming 之后收到的新回调；否则一个更早的
        # pre-offboard 状态消息会在切换瞬间被误判为 OFFBOARD_LOST。
        if info["state_receive_count"] > int(confirmation_state_count):
            if info["mode"] and info["mode"] != "OFFBOARD":
                return ABORT_OFFBOARD_LOST, info
            if info["armed"] is False:
                return ABORT_ARMING_LOST, info

        sim_stale = (
            info["state_age_sim_s"] is not None and
            info["state_age_sim_s"] > self.state_stale_sim_s
        )
        hard_wall_stale = (
            info["state_age_wall_s"] is not None and
            info["state_age_wall_s"] > self.state_hard_stale_wall_s
        )

        if sim_stale or hard_wall_stale:
            self._consecutive_state_stale += 1
        else:
            self._consecutive_state_stale = 0

        if self._consecutive_state_stale >= self.state_stale_consecutive_checks:
            return ABORT_MAVROS_STATE_STALE, info
        return None, info


# ------------------------------------------------------------------
# G1 状态机总览（便于现场排查日志）
#
# PRESTREAM_SETPOINTS : 等待首次完整遥测，同时预发送安全 setpoint。
# ARM_NEUTRALIZE      : 机械臂从首次实测 q 平滑回 neutral，飞行器未解锁。
# ARM_AND_OFFBOARD    : 保持初始位置，完成 OFFBOARD 与 arming 确认。
# TAKEOFF_TRANSITION  : 从初始位置按五次轨迹飞向最终 hold 点。
# TAKEOFF_HOLD        : 在 hold 点稳定后，才允许机械臂展开。
# ARM_DEPLOY          : neutral -> target 的关节 profile。
# CONFIGURATION_HOLD  : G1 的主要静态构型保持验证阶段。
# ARM_RETRACT         : target -> neutral 的关节 profile。
# FINAL_HOLD          : 收臂后的最终悬停。
# ABORT               : 保持最后安全机体参考，并受控收臂。
# ------------------------------------------------------------------


class G1Runner(object):
    """G1 主执行器：按阶段生成唯一活动参考并完成一次静态构型验证。"""

    def __init__(self, cfg, interfaces, scenario, root):
        self.cfg = cfg
        self.interfaces = interfaces
        self.scenario = scenario
        self.root = Path(root)
        self.joint_names = list(cfg["joint_names"])
        self.target = cfg["scenarios"][scenario]
        self.neutral = cfg["scenarios"]["neutral"]

        self.converter = FrameConverter()
        self.monitor = StateMonitor(interfaces["topics"], self.joint_names)
        self.hover = HoverOffboardInterface(
            interfaces["topics"], interfaces["services"], self.converter)
        self.arm = ArmReferenceInterface(
            interfaces["topics"], cfg["arm_control"], self.joint_names)
        self.safety = SafetySupervisor(cfg["safety"], cfg["watchdog"])
        self.runtime_freshness = RuntimeFreshness(
            interfaces["topics"], cfg.get("watchdog", {}))

        self.run_directory = Path(cfg["resolved_paths"]["output"]) / cfg["run_id"]
        self.writer = ResultWriter(self.run_directory, self.joint_names)
        self.rows = []
        self.commands = []
        self.abort = None
        self.startup_gate = None
        self.startup_diagnostics = {}

        self.hold_position = np.asarray(
            cfg["hold_setpoint_world_enu"]["position_m"], dtype=float)
        self.hold_yaw = float(cfg["hold_setpoint_world_enu"]["yaw_rad"])
        self.q_neutral = np.asarray(
            [self.neutral["joints"][name] for name in self.joint_names], dtype=float)
        self.q_target = np.asarray(
            [self.target["joints"][name] for name in self.joint_names], dtype=float)
        velocity_limits = np.asarray(
            [cfg["arm_control"]["velocity_limit_rad_s"][name] for name in self.joint_names],
            dtype=float,
        )

        timing = cfg["timing"]
        self.setpoint_rate_hz = float(timing["setpoint_rate_hz"])
        self.deploy_duration = required_duration(
            self.q_neutral,
            self.q_target,
            velocity_limits,
            timing["deploy_duration_s"],
        )
        self.default_ready_hold_duration = float(timing["ready_hold_duration_s"])
        self.default_state_timeout = float(timing["state_timeout_s"])
        self.target_hold_duration = float(timing["target_hold_duration_s"])
        self.final_hold_duration = float(timing["final_hold_duration_s"])
        self.prestream_duration = float(timing["prestream_duration_s"])

        neutralize = cfg.get("arm_neutralize", {})
        self.neutralize_duration = max(float(neutralize.get("duration_s", 3.0)), 1e-3)
        self.neutralize_min_duration = max(float(neutralize.get("min_duration_s", 1.0)), 0.0)
        self.neutralize_stable_duration = max(float(neutralize.get("stable_duration_s", 1.0)), 0.0)
        self.neutralize_tracking_error_rad = math.radians(
            float(neutralize.get("tracking_error_deg", 15.0)))
        self.neutralize_final_error_rad = math.radians(
            float(neutralize.get("final_error_deg", 3.0)))
        self.neutralize_timeout = max(float(neutralize.get("timeout_s", 8.0)), self.neutralize_duration)

        takeoff = cfg.get("takeoff", {})
        self.takeoff_duration = max(float(takeoff.get("transition_duration_s", 4.0)), 1e-3)
        self.takeoff_min_distance = max(float(takeoff.get("min_transition_distance_m", 0.02)), 0.0)
        self.takeoff_transition_error_m = float(takeoff.get("transition_position_error_m", 0.50))
        self.takeoff_hold_ready_duration = float(
            takeoff.get("hold_ready_duration_s", self.default_ready_hold_duration))

        absolute = cfg.get("absolute_safety", {})
        self.absolute_max_distance_m = float(absolute.get("max_distance_from_initial_m", 2.0))
        self.absolute_max_altitude_above_m = float(absolute.get("max_altitude_above_initial_m", 2.0))
        self.absolute_min_altitude_below_m = float(absolute.get("min_altitude_below_initial_m", 0.30))
        self.absolute_max_roll_pitch_rad = math.radians(
            float(absolute.get("max_roll_pitch_deg", 30.0)))

        # 首次完整 telemetry 到达后锁存的初始条件。
        # 在 StartupTelemetryGate ready 之前，这些值都不能用于控制或安全误差。
        self.initial_position = None
        self.initial_joint_position = None
        self.initial_yaw = None
        self.takeoff_start_position = None

        # 阶段持续时间、五次起飞轨迹和关节 profile 只使用仿真时间。
        self.state_start_sim = None
        self.profile_start_sim = None
        self.profile_start_q = None
        self.stable_since_sim = None

        # Offboard 服务请求重试与服务确认超时只使用单调真实时间。
        self.last_mode_request_wall = 0.0
        self.offboard_request_start_wall = None
        self.arm_request_start_wall = None
        self.offboard_confirmed = False
        self.armed_confirmed = False
        # Number of direct MAVROS state callbacks observed when both flags
        # become true.  Explicit loss checks only use newer callbacks.
        self.confirmation_state_receive_count = 0

        self.abort_position_reference = None
        self.abort_joint_start = None
        self.abort_start_sim = None
        self.last_active_reference = None
        self.last_freshness = {}

        logging_cfg = cfg["logging"]
        watchdog_cfg = cfg["watchdog"]
        self.next_watchdog_wall = 0.0
        self.next_log_wall = 0.0
        self.next_flush_wall = 0.0
        self.watchdog_period_s = 1.0 / float(watchdog_cfg["watchdog_rate_hz"])
        self.log_period_s = 1.0 / float(logging_cfg["telemetry_rate_hz"])
        self.flush_period_s = float(logging_cfg["flush_period_s"])

    # ------------------------------------------------------------------
    # Startup and state transition helpers
    # ------------------------------------------------------------------

    def _capture_initial_reference(self, snapshot, wall_now):
        """首次 telemetry 完整后锁定初始位置/关节角。

        后续 PRESTREAM、ARM_NEUTRALIZE、ARM_AND_OFFBOARD 都以此为安全参考，
        避免无人机尚未起飞或机械臂尚未回中性位时，就被最终目标误差误中止。
        """
        self.initial_position = snapshot["position"].copy()
        self.initial_joint_position = snapshot["q"].copy()
        self.initial_yaw = float(snapshot["rpy"][2])
        self.startup_diagnostics = self.startup_gate.diagnostics(wall_now)

        joint_error = self.initial_joint_position - self.q_neutral
        joint_error_deg = np.degrees(joint_error)
        rospy.logwarn("G1 initial telemetry ready")
        rospy.logwarn(
            "G1 startup reference: p_initial=%s p_hold=%s distance=%.3f frame=MAVROS_LOCAL_ENU",
            self.initial_position.tolist(),
            self.hold_position.tolist(),
            float(np.linalg.norm(self.hold_position - self.initial_position)),
        )
        rospy.logwarn(
            "G1 arm startup reference: joint_names=%s q_initial_rad=%s q_neutral_rad=%s "
            "q_initial_minus_neutral_rad=%s q_initial_minus_neutral_deg=%s "
            "max_abs_initial_error_deg=%.3f",
            self.joint_names,
            self.initial_joint_position.tolist(),
            self.q_neutral.tolist(),
            joint_error.tolist(),
            joint_error_deg.tolist(),
            float(np.max(np.abs(joint_error_deg))),
        )

    def _transition(self, old_state, new_state, sim_now, snapshot):
        """统一完成状态切换、计时器重置与一次性入口动作。

        任何状态进入时都只能从这里设置 profile 起点或起飞起点，避免多个分支
        分散写状态导致时间参考不一致。
        """
        if old_state == new_state:
            return new_state

        rospy.logwarn("G1 state: %s -> %s", old_state, new_state)
        self.state_start_sim = sim_now
        self.stable_since_sim = None

        if new_state == "ARM_NEUTRALIZE":
            self.profile_start_sim = sim_now
            self.profile_start_q = snapshot["q"].copy()

        elif new_state == "TAKEOFF_TRANSITION":
            self.takeoff_start_position = snapshot["position"].copy()

        elif new_state == "ARM_DEPLOY":
            self.profile_start_sim = sim_now
            self.profile_start_q = snapshot["q"].copy()

        elif new_state == "ARM_RETRACT":
            self.profile_start_sim = sim_now
            self.profile_start_q = snapshot["q"].copy()

        elif new_state == "ABORT":
            self.abort_start_sim = sim_now
            self.abort_joint_start = snapshot["q"].copy()
            if self.last_active_reference is not None:
                self.abort_position_reference = self.last_active_reference.position.copy()
            elif self.initial_position is not None:
                self.abort_position_reference = self.initial_position.copy()
            else:
                self.abort_position_reference = self.hold_position.copy()
            self.profile_start_sim = sim_now
            self.profile_start_q = snapshot["q"].copy()

        return new_state

    def _set_abort(self, reason, state, sim_now, snapshot, reference=None):
        """Latch the first abort reason and preserve context for result.yaml."""
        if self.abort is None:
            position = snapshot["position"].tolist() if snapshot is not None else None
            joints = snapshot["q"].tolist() if snapshot is not None else None
            self.abort = {
                "reason": str(reason),
                "state": str(state),
                "sim_time_s": float(sim_now),
                "actual_position": position,
                "actual_joint_position_rad": joints,
                "active_reference_position": (
                    reference.position.tolist() if reference is not None else None),
                "active_reference_kind": (
                    reference.position_kind if reference is not None else None),
                "active_joint_reference_rad": (
                    reference.joint_position.tolist() if reference is not None else None),
                "active_joint_reference_kind": (
                    reference.joint_kind if reference is not None else None),
            }
            rospy.logerr("G1 ABORT: %s", reason)

    # ------------------------------------------------------------------
    # Reference construction
    # ------------------------------------------------------------------

    def _initial_position_or_hold(self):
        """Use final hold only before the first valid position is known.

        This fallback is harmless because the vehicle is not armed.  Immediately
        after telemetry is ready, all pre-arm phases use the captured initial
        position instead.
        """
        if self.initial_position is not None:
            return self.initial_position.copy()
        return self.hold_position.copy()

    def _profile_reference(self, start_q, goal_q, elapsed_sim, duration_s):
        """Reuse project sample_profile so arm motion is consistent everywhere."""
        q, qd = sample_profile(start_q, goal_q, elapsed_sim, duration_s)
        return np.asarray(q, dtype=float), np.asarray(qd, dtype=float)

    def _build_active_reference(self, state, sim_now, snapshot):
        """为当前状态构造唯一的 UAV 位置参考与机械臂关节参考。

        注意：这里不发布命令，也不改变状态；只负责“此时应该跟踪什么”。
        发布、误差检查和记录由主循环统一使用返回的 ActiveReference。
        """
        zeros = np.zeros(len(self.joint_names), dtype=float)
        initial_position = self._initial_position_or_hold()
        initial_joint = snapshot["q"].copy()

        # PRESTREAM：遥测未完全就绪时，位置/关节跟踪误差没有物理意义。
        # 只预发送安全悬停 setpoint，不允许 POSITION_ERROR 或
        # JOINT_TRACKING_ERROR 参与中止。
        if state == "PRESTREAM_SETPOINTS":
            return ActiveReference(
                initial_position,
                self.hold_yaw,
                initial_joint,
                zeros,
                "INITIAL_HOLD" if self.initial_position is not None else "FALLBACK_HOLD_BEFORE_TELEMETRY",
                "MEASURED_JOINT_HOLD",
                False,
                False,
            )

        # ARM_NEUTRALIZE：无人机未解锁时，机械臂从“首次实测关节角”按
        # 五次 profile 平滑回到 neutral；机体位置保持在首次实测位置。
        if state == "ARM_NEUTRALIZE":
            elapsed = sim_now - self.profile_start_sim
            q, qd = self._profile_reference(
                self.profile_start_q, self.q_neutral, elapsed, self.neutralize_duration)
            return ActiveReference(
                initial_position, self.hold_yaw, q, qd,
                "INITIAL_HOLD", "NEUTRALIZE_PROFILE", False, True)

        # ARM_AND_OFFBOARD：切模式与解锁期间仍保持初始位置，
        # 不允许提前命令飞到最终 hold 高度。
        if state == "ARM_AND_OFFBOARD":
            return ActiveReference(
                initial_position, self.hold_yaw, self.q_neutral, zeros,
                "INITIAL_HOLD", "NEUTRAL_HOLD", False, True)

        # TAKEOFF_TRANSITION：唯一使用“随时间变化位置参考”的阶段。
        # 此时位置误差相对五次起飞轨迹计算，而不是相对最终 hold 点。
        if state == "TAKEOFF_TRANSITION":
            elapsed = sim_now - self.state_start_sim
            position = quintic_position(
                self.takeoff_start_position,
                self.hold_position,
                elapsed,
                self.takeoff_duration,
            )
            return ActiveReference(
                position, self.hold_yaw, self.q_neutral, zeros,
                "TAKEOFF_TRANSITION", "NEUTRAL_HOLD", True, True)

        if state == "TAKEOFF_HOLD":
            return ActiveReference(
                self.hold_position, self.hold_yaw, self.q_neutral, zeros,
                "FINAL_HOLD", "NEUTRAL_HOLD", True, True)

        if state == "ARM_DEPLOY":
            elapsed = sim_now - self.profile_start_sim
            q, qd = self._profile_reference(
                self.profile_start_q, self.q_target, elapsed, self.deploy_duration)
            return ActiveReference(
                self.hold_position, self.hold_yaw, q, qd,
                "FINAL_HOLD", "DEPLOY_PROFILE", True, True)

        if state == "CONFIGURATION_HOLD":
            return ActiveReference(
                self.hold_position, self.hold_yaw, self.q_target, zeros,
                "FINAL_HOLD", "TARGET_HOLD", True, True)

        if state == "ARM_RETRACT":
            elapsed = sim_now - self.profile_start_sim
            q, qd = self._profile_reference(
                self.profile_start_q, self.q_neutral, elapsed, self.deploy_duration)
            return ActiveReference(
                self.hold_position, self.hold_yaw, q, qd,
                "FINAL_HOLD", "RETRACT_PROFILE", True, True)

        if state == "FINAL_HOLD":
            return ActiveReference(
                self.hold_position, self.hold_yaw, self.q_neutral, zeros,
                "FINAL_HOLD", "NEUTRAL_HOLD", True, True)

        # ABORT：机体保持最后一个已知安全位置参考，机械臂平滑收回 neutral。
        # 第一条中止原因会被锁存，后续控制不得覆盖它。
        if state == "ABORT":
            elapsed = sim_now - self.profile_start_sim
            q, qd = self._profile_reference(
                self.profile_start_q, self.q_neutral, elapsed, self.deploy_duration)
            position = self.abort_position_reference
            if position is None:
                position = initial_position
            return ActiveReference(
                position, self.hold_yaw, q, qd,
                "ABORT_SAFE_HOLD", "ABORT_RETRACT_PROFILE", False, False)

        # COMPLETE is never actively controlled for an extra iteration; this
        # return keeps the function total and safe for unit/static checks.
        return ActiveReference(
            self.hold_position, self.hold_yaw, self.q_neutral, zeros,
            "FINAL_HOLD", "NEUTRAL_HOLD", True, True)

    # ------------------------------------------------------------------
    # Safety helpers
    # ------------------------------------------------------------------

    def _absolute_safety_reason(self, snapshot):
        """检查所有阶段都必须满足的绝对安全边界。

        这是“是否飞离安全包络”的判断，和是否准确跟踪最终 hold 点是两回事。
        即使 PRESTREAM/ARM_AND_OFFBOARD 不启用普通位置误差，也保留这里的保护。
        """
        if self.initial_position is None:
            return None
        delta = snapshot["position"] - self.initial_position
        if np.linalg.norm(delta) > self.absolute_max_distance_m:
            return ABORT_ABSOLUTE_POSITION
        if delta[2] > self.absolute_max_altitude_above_m:
            return ABORT_ABSOLUTE_ALTITUDE
        if delta[2] < -self.absolute_min_altitude_below_m:
            return ABORT_ABSOLUTE_ALTITUDE
        if (abs(float(snapshot["rpy"][0])) > self.absolute_max_roll_pitch_rad or
                abs(float(snapshot["rpy"][1])) > self.absolute_max_roll_pitch_rad):
            return ABORT_ABSOLUTE_ATTITUDE
        return None

    def _legacy_safety_reason(self, wall_now, state, snapshot, reference):
        """兼容调用已有 SafetySupervisor，并按阶段屏蔽不适用的跟踪误差。

        Existing projects often have SafetySupervisor.evaluate() with the old
        positional signature.  This compatibility adapter keeps that contract
        while ensuring pre-arm states cannot be aborted merely for being far
        from the final hold point or neutral joint configuration.
        """
        raw_receive = self.monitor.receive_times()
        receive = dict(raw_receive)
        receive["setpoint_publish"] = self.hover.last_publish_wall_time

        # RuntimeFreshness 单独负责 MAVROS heartbeat。
        # 这里只把旧 supervisor 的 mavros_state 接收时间刷新为 wall_now，
        # 从而避免旧版固定 2.5 s wall-time 规则在低 RTF Gazebo 中误中止。
        # pose/imu/joint_state 等其他 topic 的旧 freshness 逻辑保持不变。
        for key in ("mavros_state", "state", "vehicle_state"):
            if key in receive:
                receive[key] = wall_now

        position_error = float(np.linalg.norm(snapshot["position"] - reference.position))
        joint_error = snapshot["q"] - reference.joint_position

        if not reference.position_tracking_enabled:
            position_error = 0.0
        # 起飞阶段使用独立且更宽松的轨迹跟踪阈值。
        # 不能让面向 FINAL_HOLD 的普通位置阈值中止正常爬升。
        if state == "TAKEOFF_TRANSITION":
            position_error = 0.0
        if not reference.joint_tracking_enabled:
            joint_error = np.zeros(len(self.joint_names), dtype=float)
        # Neutralization also owns a profile-specific joint tracking bound.
        if state == "ARM_NEUTRALIZE":
            joint_error = np.zeros(len(self.joint_names), dtype=float)

        try:
            reason = self.safety.evaluate(
                wall_now,
                state,
                receive,
                position_error,
                snapshot["rpy"][0],
                snapshot["rpy"][1],
                joint_error,
                snapshot["offboard"],
                snapshot["armed"],
                startup_complete=True,
                offboard_confirmed=self.offboard_confirmed,
                armed_confirmed=self.armed_confirmed,
                clock_expected=self.cfg["startup"]["require_clock_before_execution"],
            )
        except TypeError as exc:
            rospy.logerr("G1 SafetySupervisor API is incompatible: %s", exc)
            return ABORT_SAFETY_INTERFACE
        except Exception as exc:
            rospy.logerr("G1 SafetySupervisor failed: %s", exc)
            return ABORT_SAFETY_INTERFACE

        # Make the abort reason phase-specific where the same generic legacy
        # supervisor returns POSITION_ERROR / JOINT_TRACKING_ERROR.
        if state == "TAKEOFF_TRANSITION" and reason == "POSITION_ERROR":
            return ABORT_TAKEOFF_TRANSITION_POSITION
        if state == "ARM_NEUTRALIZE" and reason == "JOINT_TRACKING_ERROR":
            return ABORT_ARM_NEUTRALIZE_TRACKING
        return reason

    def _safety_reason(self, wall_now, sim_now, state, snapshot, reference):
        """Evaluate local hard safety, freshness, and legacy project safety."""
        reason = self._absolute_safety_reason(snapshot)
        if reason:
            return reason

        # ARM_NEUTRALIZE has an explicit profile error limit.  This gives a
        # clear reason even if the legacy supervisor uses a different default.
        if state == "ARM_NEUTRALIZE":
            error = float(np.max(np.abs(snapshot["q"] - reference.joint_position)))
            if error > self.neutralize_tracking_error_rad:
                return ABORT_ARM_NEUTRALIZE_TRACKING

        # Takeoff uses a separate error limit relative to the moving quintic
        # reference.  The ordinary final-hold position threshold is not valid
        # while the aircraft is deliberately climbing.
        if state == "TAKEOFF_TRANSITION":
            error = float(np.linalg.norm(snapshot["position"] - reference.position))
            if error > self.takeoff_transition_error_m:
                return ABORT_TAKEOFF_TRANSITION_POSITION

        freshness_reason, freshness = self.runtime_freshness.evaluate(
            wall_now, sim_now, self.offboard_confirmed, self.armed_confirmed,
            self.confirmation_state_receive_count)
        self.last_freshness = freshness
        if freshness_reason:
            return freshness_reason

        return self._legacy_safety_reason(wall_now, state, snapshot, reference)

    # ------------------------------------------------------------------
    # State progression
    # ------------------------------------------------------------------

    def _request_offboard_and_arm(self, wall_now, snapshot):
        """先请求 OFFBOARD，再请求解锁；以真实时间控制服务重试频率。"""
        if not self.offboard_confirmed and snapshot["offboard"]:
            self.offboard_confirmed = True
        if self.offboard_confirmed and not self.armed_confirmed and snapshot["armed"]:
            self.armed_confirmed = True

        if wall_now - self.last_mode_request_wall < 1.0:
            return None

        try:
            if not self.offboard_confirmed:
                if self.offboard_request_start_wall is None:
                    self.offboard_request_start_wall = wall_now
                self.hover.request_offboard()
                rospy.logwarn("G1 requested OFFBOARD")
            elif not self.armed_confirmed:
                if self.arm_request_start_wall is None:
                    self.arm_request_start_wall = wall_now
                self.hover.request_arm()
                rospy.logwarn("G1 requested arming")
        except Exception as exc:
            rospy.logwarn("G1 service request failed: %s", exc)

        self.last_mode_request_wall = wall_now
        return None

    def _advance_state(self, state, sim_now, wall_now, snapshot, reference):
        """只根据当前状态与测量决定下一个状态，不负责发布命令或写日志。"""
        elapsed = sim_now - self.state_start_sim

        if state == "PRESTREAM_SETPOINTS":
            if elapsed >= self.prestream_duration:
                return "ARM_NEUTRALIZE"
            return state

        if state == "ARM_NEUTRALIZE":
            neutral_error = float(np.max(np.abs(snapshot["q"] - self.q_neutral)))
            profile_finished = elapsed >= self.neutralize_duration
            if profile_finished and neutral_error <= self.neutralize_final_error_rad:
                if self.stable_since_sim is None:
                    self.stable_since_sim = sim_now
                stable_elapsed = sim_now - self.stable_since_sim
                if (elapsed >= self.neutralize_min_duration and
                        stable_elapsed >= self.neutralize_stable_duration):
                    return "ARM_AND_OFFBOARD"
            else:
                self.stable_since_sim = None

            if elapsed > self.neutralize_timeout:
                self._set_abort(ABORT_ARM_NEUTRALIZE_TIMEOUT, state, sim_now,
                                snapshot, reference)
                return "ABORT"
            return state

        if state == "ARM_AND_OFFBOARD":
            self._request_offboard_and_arm(wall_now, snapshot)

            offboard_timeout = float(
                self.cfg["offboard_transition"]["offboard_confirm_timeout_s"])
            arm_timeout = float(
                self.cfg["offboard_transition"]["arm_confirm_timeout_s"])

            if (self.offboard_request_start_wall is not None and
                    not self.offboard_confirmed and
                    wall_now - self.offboard_request_start_wall > offboard_timeout):
                self._set_abort(ABORT_OFFBOARD_NOT_CONFIRMED, state, sim_now,
                                snapshot, reference)
                return "ABORT"
            if (self.arm_request_start_wall is not None and
                    not self.armed_confirmed and
                    wall_now - self.arm_request_start_wall > arm_timeout):
                self._set_abort(ABORT_ARMING_NOT_CONFIRMED, state, sim_now,
                                snapshot, reference)
                return "ABORT"
            if self.offboard_confirmed and self.armed_confirmed:
                self.confirmation_state_receive_count = self.runtime_freshness.state_receive_count()
                if np.linalg.norm(snapshot["position"] - self.hold_position) <= self.takeoff_min_distance:
                    return "TAKEOFF_HOLD"
                return "TAKEOFF_TRANSITION"
            return state

        if state == "TAKEOFF_TRANSITION":
            if elapsed >= self.takeoff_duration:
                return "TAKEOFF_HOLD"
            return state

        if state == "TAKEOFF_HOLD":
            readiness = self.cfg["readiness"]
            ready = (
                np.linalg.norm(snapshot["position"] - self.hold_position) <
                float(readiness["position_error_m"]) and
                np.linalg.norm(snapshot["velocity"]) <
                float(readiness["speed_mps"]) and
                np.max(np.abs(snapshot["q"] - self.q_neutral)) <
                math.radians(float(readiness["joint_error_deg"]))
            )
            if ready:
                if self.stable_since_sim is None:
                    self.stable_since_sim = sim_now
                if sim_now - self.stable_since_sim >= self.takeoff_hold_ready_duration:
                    return "ARM_DEPLOY"
            else:
                self.stable_since_sim = None
            return state

        if state == "ARM_DEPLOY":
            if elapsed >= self.deploy_duration:
                return "CONFIGURATION_HOLD"
            return state

        if state == "CONFIGURATION_HOLD":
            if elapsed >= self.target_hold_duration:
                return "ARM_RETRACT"
            return state

        if state == "ARM_RETRACT":
            if elapsed >= self.deploy_duration:
                return "FINAL_HOLD"
            return state

        if state == "FINAL_HOLD":
            if elapsed >= self.final_hold_duration:
                return "COMPLETE"
            return state

        if state == "ABORT":
            # ABORT 后仍留出一个关节收回 profile 加最终保持时间。
            # 这段期间不得覆盖最初锁存的中止原因。
            if elapsed >= self.deploy_duration + self.final_hold_duration:
                return "COMPLETE"
            return state

        return state

    def _generic_phase_timeout(self, state, sim_now, snapshot, reference):
        """Timeout only truly waiting states; motion phases own their duration."""
        if state not in ("PRESTREAM_SETPOINTS", "ARM_AND_OFFBOARD", "TAKEOFF_HOLD"):
            return None
        if sim_now - self.state_start_sim > self.default_state_timeout:
            self._set_abort(ABORT_PHASE_TIMEOUT, state, sim_now, snapshot, reference)
            return "ABORT"
        return None

    # ------------------------------------------------------------------
    # Logging and final result writing
    # ------------------------------------------------------------------

    def _record(self, sim_now, start_wall, state, snapshot, reference):
        """记录实际量、当前活动参考，以及相对活动参考/最终参考的两类误差。

        这样离线分析可区分“正常起飞中尚未到 hold 点”和“真正偏离当前轨迹”。
        """
        position_error_active = float(np.linalg.norm(
            snapshot["position"] - reference.position))
        position_error_hold = float(np.linalg.norm(
            snapshot["position"] - self.hold_position))
        position_error_initial = (
            float(np.linalg.norm(snapshot["position"] - self.initial_position))
            if self.initial_position is not None else float("nan"))
        joint_error_active = snapshot["q"] - reference.joint_position
        joint_error_neutral = snapshot["q"] - self.q_neutral

        row = {
            "sim_time_s": float(sim_now),
            "wall_time_s": float(time.monotonic() - start_wall),
            "state": state,
            "active_position_reference_kind": reference.position_kind,
            "active_joint_reference_kind": reference.joint_kind,
            "px": float(snapshot["position"][0]),
            "py": float(snapshot["position"][1]),
            "pz": float(snapshot["position"][2]),
            "ref_px": float(reference.position[0]),
            "ref_py": float(reference.position[1]),
            "ref_pz": float(reference.position[2]),
            "hold_ref_px": float(self.hold_position[0]),
            "hold_ref_py": float(self.hold_position[1]),
            "hold_ref_pz": float(self.hold_position[2]),
            "position_error_m": position_error_active,
            "position_error_to_active_reference_m": position_error_active,
            "position_error_to_final_hold_m": position_error_hold,
            "position_error_to_initial_m": position_error_initial,
            "vx": float(snapshot["velocity"][0]),
            "vy": float(snapshot["velocity"][1]),
            "vz": float(snapshot["velocity"][2]),
            "speed_mps": float(np.linalg.norm(snapshot["velocity"])),
            "roll_rad": float(snapshot["rpy"][0]),
            "pitch_rad": float(snapshot["rpy"][1]),
            "yaw_rad": float(snapshot["rpy"][2]),
            "wx": float(snapshot["omega"][0]),
            "wy": float(snapshot["omega"][1]),
            "wz": float(snapshot["omega"][2]),
            "yaw_error_rad": float(math.atan2(
                math.sin(snapshot["rpy"][2] - self.hold_yaw),
                math.cos(snapshot["rpy"][2] - self.hold_yaw))),
            "max_joint_error_rad": float(np.max(np.abs(joint_error_active))),
            "max_joint_error_to_active_reference_deg": float(
                np.degrees(np.max(np.abs(joint_error_active)))),
            "max_joint_error_to_neutral_deg": float(
                np.degrees(np.max(np.abs(joint_error_neutral)))),
            "offboard": bool(snapshot["offboard"]),
            "armed": bool(snapshot["armed"]),
            "mavros_state_age_sim_s": self.last_freshness.get("state_age_sim_s"),
            "mavros_state_age_wall_s": self.last_freshness.get("state_age_wall_s"),
            "mavros_state_receive_count": self.last_freshness.get("state_receive_count"),
        }
        command = {
            "sim_time_s": float(sim_now),
            "state": state,
            "active_position_reference_kind": reference.position_kind,
            "active_joint_reference_kind": reference.joint_kind,
            "ref_px": float(reference.position[0]),
            "ref_py": float(reference.position[1]),
            "ref_pz": float(reference.position[2]),
        }
        for index, name in enumerate(self.joint_names):
            row["q_" + name] = float(snapshot["q"][index])
            row["qd_" + name] = float(snapshot["qd"][index])
            row["qref_" + name] = float(reference.joint_position[index])
            row["qcmd_" + name] = float(reference.joint_position[index])
            row["effort_" + name] = float("nan")
            command["qcmd_" + name] = float(reference.joint_position[index])
            command["qdcmd_" + name] = float(reference.joint_velocity[index])
        self.rows.append(row)
        self.commands.append(command)

    def _finish(self, status, metrics):
        """无论成功或提前中止，都写出兼容现有离线分析的结果文件。"""
        trim = trim_reference(
            self.cfg["resolved_paths"]["static_trim_results"], self.scenario)
        topic_rates = self.monitor.topic_rates()
        if self.startup_gate is not None:
            self.startup_diagnostics = self.startup_gate.diagnostics(time.monotonic())

        arrivals = np.asarray(self.hover.publish_arrivals, dtype=float)
        intervals = np.diff(arrivals)
        positive = intervals[intervals > 0.0]
        topic_rates["setpoint_publish"] = {
            "samples": int(len(arrivals)),
            "mean_hz": float(1.0 / np.mean(positive)) if len(positive) else None,
            "min_hz": float(np.min(1.0 / positive)) if len(positive) else None,
            "max_hz": float(np.max(1.0 / positive)) if len(positive) else None,
            "max_interarrival_s": float(np.max(intervals)) if len(intervals) else None,
        }

        metadata = {
            "run_id": self.cfg["run_id"],
            "scenario": self.scenario,
            "git_commit": git_value(["git", "rev-parse", "HEAD"], self.root),
            "ros_version": "ROS1 melodic",
            "gazebo_version": "9.19.0",
            "px4_commit": git_value(
                ["git", "rev-parse", "HEAD"], "/home/zlhq/PX4_Firmware"),
            "topics": self.interfaces["topics"],
            "startup_telemetry": self.startup_diagnostics,
            "takeoff_reference": {
                "initial_position_world": (
                    self.initial_position.tolist()
                    if self.initial_position is not None else None),
                "transition_start_position_world": (
                    self.takeoff_start_position.tolist()
                    if self.takeoff_start_position is not None else None),
                "hold_position_world": self.hold_position.tolist(),
                "transition_duration_s": self.takeoff_duration,
            },
            "arm_startup_reference": {
                "joint_names": self.joint_names,
                "q_initial_rad": (
                    self.initial_joint_position.tolist()
                    if self.initial_joint_position is not None else None),
                "q_neutral_rad": self.q_neutral.tolist(),
                "q_initial_minus_neutral_rad": (
                    (self.initial_joint_position - self.q_neutral).tolist()
                    if self.initial_joint_position is not None else None),
            },
            "mavros_state_freshness": self.last_freshness,
            "simulation_time_used": bool(self.rows),
            "simulation_time_status": "SIM_TIME_AVAILABLE" if self.rows else "SIM_TIME_UNAVAILABLE",
            "wall_duration_s": self.rows[-1]["wall_time_s"] if self.rows else 0.0,
            "sim_duration_s": (
                self.rows[-1]["sim_time_s"] - self.rows[0]["sim_time_s"]
                if len(self.rows) >= 2 else 0.0),
            "clock_enabled": bool(self.rows),
            "motor_output_available": False,
            "joint_effort_available": False,
            "joint_effort_note": (
                "JOINT_EFFORT_UNAVAILABLE: /joint_states effort is not used as "
                "a measured actuator torque in G1."),
            "abort_triggered": self.abort is not None,
            "actual_deploy_duration_s": float(self.deploy_duration),
        }
        self.writer.write_run(
            self.rows,
            self.commands,
            self.target,
            trim,
            metadata,
            status,
            metrics,
            self.abort,
            topic_rates,
            self.startup_diagnostics,
        )
        rospy.logwarn("G1 %s: %s; files: %s", self.scenario, status, self.run_directory)
        return status

    # ------------------------------------------------------------------
    # Main control loop
    # ------------------------------------------------------------------

    def run(self):
        """Execute one finite G1 experiment and write a result directory."""
        if self.target.get("status") != "RESOLVED":
            return self._finish("CONFIGURATION_UNRESOLVED", {})

        loop_period_s = 1.0 / max(self.setpoint_rate_hz, 1.0)
        start_wall = time.monotonic()
        self.startup_gate = StartupTelemetryGate(self.cfg["startup"], start_wall)

        state = "PRESTREAM_SETPOINTS"
        self.state_start_sim = None
        waiting_logged = False
        startup_ready = False

        rospy.logwarn("G1 state: IDLE -> PRESTREAM_SETPOINTS")

        while not rospy.is_shutdown() and state != "COMPLETE":
            wall_now = time.monotonic()
            sim_now = rospy.get_time()
            snapshot = self.monitor.snapshot()

            # 首次消息未收齐前，只有 StartupTelemetryGate 有权判断状态。
            # 不对不完整数据执行 profile、状态推进或常规安全误差判断。
            self.startup_gate.update(self.monitor.receive_times())
            if not self.startup_gate.ready():
                # 解锁前预发送一个安全点。首次 telemetry 就绪后会立刻替换为
                # 实际初始位置，因此不会把最终 hold 点误用于启动期跟踪。
                bootstrap_ref = ActiveReference(
                    self._initial_position_or_hold(), self.hold_yaw,
                    np.zeros(len(self.joint_names)),
                    np.zeros(len(self.joint_names)),
                    "FALLBACK_HOLD_BEFORE_TELEMETRY", "NO_JOINT_REFERENCE",
                    False, False)
                self.hover.publish(bootstrap_ref.position, bootstrap_ref.yaw_rad)

                if not waiting_logged:
                    rospy.logwarn("G1 waiting for initial telemetry: missing=%s",
                                  self.startup_gate.missing())
                    waiting_logged = True
                failure = self.startup_gate.failure_reason(wall_now)
                if failure:
                    self.abort = {
                        "reason": failure,
                        "state": state,
                        "sim_time_s": float(sim_now),
                    }
                    self.startup_diagnostics = self.startup_gate.diagnostics(wall_now)
                    rospy.logerr("G1 startup unavailable: %s", failure)
                    return self._finish("NOT_RUN_INTERFACE_UNAVAILABLE", {})
                time.sleep(loop_period_s)
                continue

            if snapshot is None or snapshot.get("missing_joints"):
                self.abort = {
                    "reason": ABORT_CONFIGURATION_UNRESOLVED,
                    "state": state,
                    "sim_time_s": float(sim_now),
                    "missing_joints": (
                        snapshot.get("missing_joints", []) if snapshot else self.joint_names),
                }
                return self._finish("CONFIGURATION_UNRESOLVED", {})

            if not startup_ready:
                self._capture_initial_reference(snapshot, wall_now)
                self.state_start_sim = sim_now
                startup_ready = True

            # 为“当前状态”生成并发布唯一活动参考：本轮的 UAV、机械臂、
            # watchdog 与日志都必须使用同一个 reference。
            reference = self._build_active_reference(state, sim_now, snapshot)
            self.last_active_reference = reference
            self.hover.publish(reference.position, reference.yaw_rad)
            self.arm.publish(
                reference.joint_position,
                reference.joint_velocity,
                snapshot["q"],
            )

            # 先做安全检查，再做正常状态切换；这样发生中止时，结果文件保留的
            # 就是故障瞬间实际生效的参考，而不是下一阶段参考。
            published_state = state
            if wall_now >= self.next_watchdog_wall and self.abort is None:
                reason = self._safety_reason(
                    wall_now, sim_now, state, snapshot, reference)
                self.next_watchdog_wall = wall_now + self.watchdog_period_s
                if reason:
                    self._set_abort(reason, state, sim_now, snapshot, reference)
                    state = self._transition(state, "ABORT", sim_now, snapshot)

            # 正常运行按状态机推进。发生中止后，只允许 ABORT 的收臂/保持过程
            # 推进到 COMPLETE，禁止继续进入后续任务阶段。
            if self.abort is None:
                timeout_state = self._generic_phase_timeout(
                    state, sim_now, snapshot, reference)
                if timeout_state is not None:
                    state = self._transition(state, timeout_state, sim_now, snapshot)
                else:
                    next_state = self._advance_state(
                        state, sim_now, wall_now, snapshot, reference)
                    if next_state != state:
                        state = self._transition(state, next_state, sim_now, snapshot)
            elif state == "ABORT":
                next_state = self._advance_state(
                    state, sim_now, wall_now, snapshot, reference)
                if next_state != state:
                    state = self._transition(state, next_state, sim_now, snapshot)

            # 记录本周期真正发布的命令/参考。状态切换发生在发布之后，因此日志
            # 必须保留 published_state；新状态从下一周期才真正开始生效。
            if wall_now >= self.next_log_wall:
                self._record(sim_now, start_wall, published_state, snapshot, reference)
                self.next_log_wall = wall_now + self.log_period_s

            if wall_now >= self.next_flush_wall:
                self.writer.checkpoint(self.rows, self.commands)
                self.next_flush_wall = wall_now + self.flush_period_s

            time.sleep(loop_period_s)

        # COMPLETE 既可能表示正常完成，也可能表示中止后已完成受控收臂/保持。
        # assess_run 根据 abort 标志区分两者，并执行常规指标评估。
        status, metrics = assess_run(
            self.rows, self.cfg["pass_thresholds"], self.abort is not None)
        return self._finish(status, metrics)


def main():
    """解析 ROS 安全的命令行参数，并创建一次 G1 实验运行器。"""
    rospy.init_node("g1_static_hold_runner", log_level=rospy.WARN)

    parser = argparse.ArgumentParser(
        description="Run one G1 Gazebo/PX4 static arm-configuration hold experiment.")
    parser.add_argument("--config", default=None, help="Path to g1_static_hold.yaml")
    parser.add_argument("--scenario", default=None, help="Scenario name, e.g. neutral")
    parser.add_argument("--run-id", default=None, help="Optional output run identifier")
    args = parser.parse_args(rospy.myargv()[1:])

    config_argument = args.config or rospy.get_param("~config")
    config_path = Path(config_argument)
    scenario = args.scenario or rospy.get_param("~scenario", "neutral")
    root = config_path.resolve().parents[3]

    cfg = resolve_g1_config(config_path, root)
    interfaces = load_yaml(root / cfg["interfaces_file"])
    if scenario not in cfg["scenarios"]:
        raise RuntimeError("unknown G1 scenario: " + scenario)

    cfg["run_id"] = args.run_id or (
        scenario + "_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
    output = Path(cfg["resolved_paths"]["output"]) / cfg["run_id"]
    output.mkdir(parents=True, exist_ok=True)
    (output / "resolved_interfaces.yaml").write_text(
        yaml.safe_dump(interfaces, sort_keys=False))
    (output / "experiment_config.yaml").write_text(
        yaml.safe_dump(cfg, sort_keys=False))

    status = G1Runner(cfg, interfaces, scenario, root).run()
    if status not in (
            "PASS",
            "NOT_RUN_ENVIRONMENT_UNAVAILABLE",
            "NOT_RUN_INTERFACE_UNAVAILABLE"):
        raise SystemExit(2)


if __name__ == "__main__":
    main()
