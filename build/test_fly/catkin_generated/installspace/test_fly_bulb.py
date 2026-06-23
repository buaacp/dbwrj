#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

"""Unified UAV state machine for the bulb replacement task.

This script is the current cleaned-up bulb flight mission.  It keeps the
same mission states and ROS topic contract.  Every flying state is handled by
the same velocity tracker: build a target pose, run PD position feedback,
clamp velocity, and publish a MAVROS raw setpoint.
"""

import math
import threading

import numpy as np
import rospy
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray, Int32

from l1_adaptive import L1AdaptiveAugmentation, L1Params
from multirotor_communication import Communication


MAX_VELOCITY = 1.0
PD_GAIN = 2.0

STATE_IDLE = 0
STATE_PICK_ALIGN = 1
STATE_GRASP_BULB = 2
STATE_UNSCREW_BULB = 3
STATE_TRANSFER_TO_SOCKET = 4
STATE_FINE_ALIGN_SOCKET = 5
STATE_SCREW_IN_BULB = 6
STATE_FINISH = 7
STATE_ABORT = 99

STATE_NAMES = {
    STATE_IDLE: "IDLE",
    STATE_PICK_ALIGN: "PICK_ALIGN",
    STATE_GRASP_BULB: "GRASP_BULB",
    STATE_UNSCREW_BULB: "UNSCREW_BULB",
    STATE_TRANSFER_TO_SOCKET: "TRANSFER_TO_SOCKET",
    STATE_FINE_ALIGN_SOCKET: "FINE_ALIGN_SOCKET",
    STATE_SCREW_IN_BULB: "SCREW_IN_BULB",
    STATE_FINISH: "FINISH",
    STATE_ABORT: "ABORT",
}


class ArmFeedback:
    """Latest mechanical-arm state published by arm_control_bulb."""

    def __init__(self):
        self.ready_state = 0
        self.error = [float("nan"), float("nan"), float("nan")]

    def ready_callback(self, msg):
        self.ready_state = int(msg.data)

    def error_callback(self, msg):
        values = [float(item) for item in msg.data[:3]]
        while len(values) < 3:
            values.append(float("nan"))
        self.error = values

    def ready_for(self, state, required):
        return True if not required else self.ready_state == state


class MissionConfig:
    """All tunable parameters used by the UAV mission state machine."""

    def __init__(self):
        self.vehicle_type = rospy.get_param("~vehicle_type", "iris")
        self.vehicle_id = str(rospy.get_param("~vehicle_id", "0"))
        self.pd_gain = float(rospy.get_param("~pd_gain", PD_GAIN))
        self.default_height = float(rospy.get_param("~default_height", 1.0))
        self.default_max_velocity = float(rospy.get_param("~default_max_velocity", MAX_VELOCITY))
        self.target_max_velocity = float(rospy.get_param("~target_max_velocity", 0.25))
        self.hold_max_velocity = float(rospy.get_param("~hold_max_velocity", 0.05))
        self.max_vertical_velocity = float(rospy.get_param("~max_vertical_velocity", 0.5))
        self.position_tolerance = float(rospy.get_param("~position_tolerance", 0.07))
        self.auto_start = parse_bool(rospy.get_param("~auto_start", True))
        self.require_arm_ready = parse_bool(rospy.get_param("~require_arm_ready", True))
        self.grasp_duration = float(rospy.get_param("~grasp_duration", 3.0))
        self.unscrew_duration = float(rospy.get_param("~unscrew_duration", 6.0))
        self.screw_duration = float(rospy.get_param("~screw_duration", 6.0))
        self.finish_duration = float(rospy.get_param("~finish_duration", 4.0))
        self.bulb_center_axis_offset = float(rospy.get_param("~bulb_center_axis_offset", -0.07))
        self.socket_center_axis_offset = float(rospy.get_param("~socket_center_axis_offset", -0.15))
        self.bulb_axis_local = vector_param("~bulb_axis_local", [0.0, 0.0, -1.0])
        self.socket_axis_local = vector_param("~socket_axis_local", [0.0, 0.0, -1.0])
        self.pick_uav_offset = vector_param("~pick_uav_offset", [0.0, 0.096, 0.150])
        self.socket_uav_offset = vector_param("~socket_uav_offset", [0.0, 0.096, 0.150])
        self.retreat_offset = vector_param("~retreat_offset", [-0.35, 0.0, 0.25])
        fixed_yaw = rospy.get_param("~target_yaw_deg", "")
        self.fixed_yaw = None if str(fixed_yaw).strip() == "" else math.radians(float(fixed_yaw))
        self.smooth_trajectory_enabled = parse_bool(rospy.get_param("~smooth_trajectory_enabled", True))
        self.smooth_idle_duration = float(rospy.get_param("~smooth_idle_duration", 3.0))
        self.smooth_pick_duration = float(rospy.get_param("~smooth_pick_duration", 5.0))
        self.smooth_finish_duration = float(rospy.get_param("~smooth_finish_duration", 4.0))
        self.smooth_min_segment_duration = float(rospy.get_param("~smooth_min_segment_duration", 0.8))
        self.safe_lift_height = float(rospy.get_param("~safe_lift_height", 0.25))
        self.trajectory_feedforward_enabled = parse_bool(
            rospy.get_param("~trajectory_feedforward_enabled", True))
        self.trajectory_feedforward_gain = float(rospy.get_param("~trajectory_feedforward_gain", 1.0))
        self.enable_l1_adaptive = parse_bool(rospy.get_param("~enable_l1_adaptive", False))
        self.l1_velocity_time_constant = float(rospy.get_param("~l1_velocity_time_constant", 0.45))
        self.l1_velocity_correction_gain = float(rospy.get_param("~l1_velocity_correction_gain", 1.0))
        self.max_l1_velocity_correction = float(rospy.get_param("~max_l1_velocity_correction", 0.12))
        self.l1_params = L1Params(
            enabled=self.enable_l1_adaptive,
            as_gain=float(rospy.get_param("~l1_as_gain", 8.0)),
            wc_xy=float(rospy.get_param("~l1_wc_xy", 4.0)),
            wc_z=float(rospy.get_param("~l1_wc_z", 3.0)),
            max_accel_xy=float(rospy.get_param("~l1_max_accel_xy", 1.5)),
            max_accel_z=float(rospy.get_param("~l1_max_accel_z", 0.8)),
            max_sigma=float(rospy.get_param("~l1_max_sigma", 6.0)),
            use_pos_feedback=parse_bool(rospy.get_param("~l1_use_pos_feedback", False)),
            k_pos_i_xy=float(rospy.get_param("~l1_k_pos_i_xy", 0.25)),
            k_pos_i_z=float(rospy.get_param("~l1_k_pos_i_z", 0.15)),
            k_pos_p_xy=float(rospy.get_param("~l1_k_pos_p_xy", 0.0)),
            k_pos_p_z=float(rospy.get_param("~l1_k_pos_p_z", 0.0)),
            max_pos_integral_xy=float(rospy.get_param("~l1_max_pos_integral_xy", 0.8)),
            max_pos_integral_z=float(rospy.get_param("~l1_max_pos_integral_z", 0.5)),
        )


class SmoothSegment:
    """Fixed-duration quintic segment with zero endpoint velocity."""

    def __init__(self, start, end, duration):
        self.start = copy_pose(start)
        self.end = copy_pose(end)
        self.duration = max(float(duration), 1e-3)

    def sample(self, elapsed):
        tau = clamp(float(elapsed) / self.duration, 0.0, 1.0)
        blend = 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5
        return interpolate_pose(self.start, self.end, blend)

    def velocity(self, elapsed):
        tau = clamp(float(elapsed) / self.duration, 0.0, 1.0)
        blend_rate = (30.0 * tau ** 2 - 60.0 * tau ** 3 + 30.0 * tau ** 4) / self.duration
        delta = np.array([
            self.end.position.x - self.start.position.x,
            self.end.position.y - self.start.position.y,
            self.end.position.z - self.start.position.z,
        ], dtype=float)
        return blend_rate * delta


class SmoothTrajectory:
    """Piecewise fixed-duration min-jerk trajectory through pose waypoints."""

    def __init__(self, poses, duration, min_segment_duration=0.8):
        if len(poses) < 2:
            raise ValueError("SmoothTrajectory requires at least two poses")
        distances = [pose_distance(poses[i], poses[i + 1]) for i in range(len(poses) - 1)]
        total_distance = sum(distances)
        segment_count = len(distances)
        minimum_duration = max(float(min_segment_duration), 1e-3)
        total_duration = max(float(duration), minimum_duration * segment_count)
        distributable_duration = total_duration - minimum_duration * segment_count
        if total_distance < 1e-6:
            durations = [total_duration / segment_count] * segment_count
        else:
            durations = [minimum_duration + distributable_duration * d / total_distance
                         for d in distances]
        self.segments = [SmoothSegment(poses[i], poses[i + 1], durations[i]) for i in range(segment_count)]
        self.duration = sum(segment.duration for segment in self.segments)

    def _segment_at(self, elapsed):
        remaining = max(float(elapsed), 0.0)
        for segment in self.segments:
            if remaining <= segment.duration:
                return segment, remaining
            remaining -= segment.duration
        return self.segments[-1], self.segments[-1].duration

    def sample(self, elapsed):
        segment, segment_elapsed = self._segment_at(elapsed)
        return segment.sample(segment_elapsed)

    def velocity(self, elapsed):
        if float(elapsed) >= self.duration:
            return np.zeros(3, dtype=float)
        segment, segment_elapsed = self._segment_at(elapsed)
        return segment.velocity(segment_elapsed)


class VelocityTracker:
    """Single UAV control law shared by all mission states."""

    def __init__(self, communication, config):
        self.communication = communication
        self.config = config
        self.l1 = L1AdaptiveAugmentation(config.l1_params)
        self.last_l1_time = None
        self.last_l1_accel = np.zeros(3, dtype=float)
        self.last_l1_velocity_correction = np.zeros(3, dtype=float)

    def hold_zero_velocity(self):
        return self.set_velocity(0.0, 0.0, 0.0, self.current_yaw())

    def hold_pose(self, target, yaw):
        """Hold a fixed point with the same velocity-control path as other states."""
        return self.track_pose(target, self.config.hold_max_velocity, yaw) is not None

    def current_yaw(self):
        yaw = self.communication.current_yaw
        return yaw if math.isfinite(float(yaw)) else 0.0

    def track_pose(self, target, max_xy_velocity, yaw, feedforward_velocity=None,
                   max_vertical_velocity=None):
        """Track a world-frame target pose using bounded velocity feedback."""
        current = self.communication.current_position
        if not finite_position(current) or not finite_pose(target):
            return None

        max_vz = self.config.max_vertical_velocity if max_vertical_velocity is None else float(max_vertical_velocity)
        ex = target.position.x - current.x
        ey = target.position.y - current.y
        ez = target.position.z - current.z
        vx = ex * self.config.pd_gain
        vy = ey * self.config.pd_gain
        vz = clamp(ez * self.config.pd_gain, -max_vz, max_vz)
        if feedforward_velocity is not None and self.config.trajectory_feedforward_enabled:
            feedforward = np.array(feedforward_velocity, dtype=float).reshape(3)
            if np.all(np.isfinite(feedforward)):
                vx += self.config.trajectory_feedforward_gain * feedforward[0]
                vy += self.config.trajectory_feedforward_gain * feedforward[1]
                vz += self.config.trajectory_feedforward_gain * feedforward[2]
        vx, vy = limit_horizontal_velocity(vx, vy, max_xy_velocity)
        vz = clamp(vz, -max_vz, max_vz)
        vx, vy, vz = self.apply_l1_compensation(
            vx, vy, vz,
            np.array([current.x - target.position.x, current.y - target.position.y, current.z - target.position.z]),
            max_xy_velocity, max_vz,
        )

        if not self.set_velocity(vx, vy, vz, yaw):
            return None
        distance = math.sqrt(ex * ex + ey * ey + ez * ez)
        self.communication.last_control_error = distance
        self.communication.last_control_target = (target.position.x, target.position.y, target.position.z)
        return distance

    def set_velocity(self, vx, vy, vz, yaw):
        if not finite_values(vx, vy, vz, yaw):
            rospy.logerr("拒绝发布非法 UAV 速度: vx=%s vy=%s vz=%s yaw=%s", vx, vy, vz, yaw)
            return False
        self.communication.last_control_velocity = (float(vx), float(vy), float(vz))
        self.communication.last_control_yaw = normalize_angle(yaw)
        self.communication.coordinate_frame = 1
        self.communication.motion_type = 1
        self.communication.target_motion = self.communication.construct_target(
            vx=vx,
            vy=vy,
            vz=vz,
            yaw=normalize_angle(yaw),
        )
        return True

    def apply_l1_compensation(self, vx, vy, vz, pos_error, max_xy_velocity, max_vertical_velocity):
        if not self.config.enable_l1_adaptive:
            self.l1.set_enabled(False)
            self.last_l1_accel[:] = 0.0
            self.last_l1_velocity_correction[:] = 0.0
            return vx, vy, vz
        velocity_msg = self.communication.current_velocity
        if velocity_msg is None:
            return vx, vy, vz
        measured_v = np.array([
            velocity_msg.twist.linear.x,
            velocity_msg.twist.linear.y,
            velocity_msg.twist.linear.z,
        ], dtype=float)
        baseline_v = np.array([vx, vy, vz], dtype=float)
        if not np.all(np.isfinite(measured_v)) or not np.all(np.isfinite(baseline_v)):
            self.l1.reset()
            return vx, vy, vz
        now = rospy.Time.now()
        if self.last_l1_time is None:
            self.last_l1_time = now
            self.l1.reset(measured_v)
            return vx, vy, vz
        dt = max((now - self.last_l1_time).to_sec(), 1e-3)
        self.last_l1_time = now
        tau = max(float(self.config.l1_velocity_time_constant), 1e-3)
        nominal_accel = (baseline_v - measured_v) / tau
        self.last_l1_accel = self.l1.step(dt, measured_v, nominal_accel, pos_error)
        correction = self.config.l1_velocity_correction_gain * tau * self.last_l1_accel
        correction = limit_vector_norm(correction, self.config.max_l1_velocity_correction)
        self.last_l1_velocity_correction = correction
        corrected = baseline_v + correction
        corrected[0], corrected[1] = limit_horizontal_velocity(corrected[0], corrected[1], max_xy_velocity)
        corrected[2] = clamp(corrected[2], -max_vertical_velocity, max_vertical_velocity)
        return float(corrected[0]), float(corrected[1]), float(corrected[2])

    def reset_adaptive_state(self):
        velocity_msg = self.communication.current_velocity
        if velocity_msg is not None:
            measured_v = np.array([
                velocity_msg.twist.linear.x,
                velocity_msg.twist.linear.y,
                velocity_msg.twist.linear.z,
            ], dtype=float)
            self.l1.reset(measured_v if np.all(np.isfinite(measured_v)) else None)
        else:
            self.l1.reset()
        self.last_l1_time = None
        self.last_l1_accel[:] = 0.0
        self.last_l1_velocity_correction[:] = 0.0


class BulbUavMission:
    """Mission state machine: each state only chooses target and transition."""

    def __init__(self):
        rospy.init_node("test_fly_bulb")
        self.config = MissionConfig()
        self.communication = Communication(self.config.vehicle_type, self.config.vehicle_id)
        self.communication.default_pose.position.z = self.config.default_height
        self.tracker = VelocityTracker(self.communication, self.config)
        self.arm = ArmFeedback()
        self.state_pub = rospy.Publisher("/mission_state", Int32, queue_size=10, latch=True)
        rospy.Subscriber("/arm_control_bulb/ready_state", Int32, self.arm.ready_callback)
        rospy.Subscriber("/arm_control_bulb/error", Float64MultiArray, self.arm.error_callback)
        self.state = STATE_IDLE
        self.last_state = None
        self.state_enter_time = rospy.Time.now()
        self.locked_targets = {}
        self.smooth_trajectories = {}
        self.smooth_start_times = {}

    def run(self):
        self.start_communication_thread()
        if not self.prepare_offboard():
            return

        rospy.loginfo("统一版 UAV 灯泡任务启动: %s", describe_states())
        rate = rospy.Rate(10)
        aborted = False
        while not rospy.is_shutdown():
            if aborted:
                self.state = STATE_ABORT
            self.on_state_entry()

            if self.communication.current_position is None:
                rospy.logwarn_throttle(2.0, "等待 PX4 本地位置反馈，保持零速度 setpoint")
                self.tracker.hold_zero_velocity()
                rate.sleep()
                continue
            if not finite_position(self.communication.current_position):
                rospy.logwarn_throttle(1.0, "PX4 local_position 非法，暂停状态推进")
                self.tracker.hold_zero_velocity()
                rate.sleep()
                continue

            ok = self.step_state()
            aborted = aborted or not ok
            self.publish_diagnostics()
            rate.sleep()

    def start_communication_thread(self):
        thread = threading.Thread(target=self.communication.start)
        thread.daemon = True
        thread.start()

    def prepare_offboard(self):
        namespace = "/%s_%s" % (self.config.vehicle_type, self.config.vehicle_id)
        try:
            rospy.wait_for_service(namespace + "/mavros/cmd/arming", timeout=30)
            rospy.wait_for_service(namespace + "/mavros/set_mode", timeout=30)
        except rospy.ROSException as exc:
            rospy.logerr("等待 MAVROS 服务超时: %s", exc)
            return False

        if not wait_for_condition("MAVROS连接", lambda: self.communication.mavros_state.connected, self.communication, 30.0):
            return False
        if not wait_for_condition("PX4稳定本地位置反馈", lambda: stable_local_position(self.communication), self.communication, 30.0):
            return False
        if not self.publish_startup_setpoints(5.0):
            return False
        if not set_offboard_mode(self.communication):
            return False
        if not self.publish_startup_setpoints(1.0):
            return False
        if not arm_vehicle(self.communication):
            return False
        return True

    def publish_startup_setpoints(self, duration):
        deadline = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if not stable_local_position(self.communication):
                rospy.logwarn_throttle(1.0, "等待稳定本地位置后再预发布 OFFBOARD setpoint")
                return False
            self.tracker.hold_zero_velocity()
            rate.sleep()
        return True

    def on_state_entry(self):
        if self.state == self.last_state:
            return
        rospy.loginfo("UAV 状态: %d %s", self.state, state_name(self.state))
        self.state_pub.publish(Int32(data=int(self.state)))
        self.state_enter_time = rospy.Time.now()
        if self.state != STATE_UNSCREW_BULB:
            self.locked_targets.pop("unscrew", None)
        if self.state != STATE_SCREW_IN_BULB:
            self.locked_targets.pop("screw", None)
        if self.state != STATE_TRANSFER_TO_SOCKET:
            self.locked_targets.pop("transfer_entry", None)
            self.locked_targets.pop("transfer_target", None)
        self.smooth_trajectories.clear()
        self.smooth_start_times.clear()
        self.tracker.reset_adaptive_state()
        self.last_state = self.state

    def step_state(self):
        elapsed = self.elapsed()
        yaw = self.config.fixed_yaw if self.config.fixed_yaw is not None else self.tracker.current_yaw()

        if self.state == STATE_IDLE:
            target = self.default_pose()
            distance = self.track_smooth_pose(
                "idle", [target], self.config.smooth_idle_duration,
                self.config.default_max_velocity, yaw, "IDLE")
            if distance is not None and self.config.auto_start and elapsed > 2.0:
                # Keep the original transition rule: IDLE only waits for
                # takeoff height, not for XY convergence to the default pose.
                current = self.communication.current_position
                height_error = abs(current.z - self.communication.default_pose.position.z)
                if height_error < self.config.position_tolerance:
                    self.state = STATE_PICK_ALIGN
                else:
                    rospy.loginfo_throttle(
                        1.0,
                        "IDLE 等待无人机到达默认悬停高度: z=%.3f target=%.3f err=%.3f",
                        current.z,
                        self.communication.default_pose.position.z,
                        height_error,
                    )
            return distance is not None

        if self.state == STATE_PICK_ALIGN:
            distance = self.track_smooth_task_target(
                "pick_align", "bulb", self.config.bulb_center_axis_offset,
                self.config.pick_uav_offset, self.config.smooth_pick_duration,
                self.config.target_max_velocity, yaw, "PICK_ALIGN")
            if distance is None:
                return False
            if self.arm.ready_for(STATE_PICK_ALIGN, self.config.require_arm_ready):
                self.state = STATE_GRASP_BULB
            return True

        if self.state == STATE_GRASP_BULB:
            distance = self.track_locked_task_target(
                "pick_align_target", "bulb", self.config.bulb_center_axis_offset,
                self.config.pick_uav_offset, self.config.hold_max_velocity, yaw, "GRASP_BULB")
            if distance is None:
                return False
            if elapsed > self.config.grasp_duration and self.arm.ready_for(STATE_GRASP_BULB, self.config.require_arm_ready):
                self.transition_to_hold_state(STATE_UNSCREW_BULB, "unscrew", yaw)
            return True

        if self.state == STATE_UNSCREW_BULB:
            target = self.lock_current_pose("unscrew")
            if elapsed <= self.config.unscrew_duration:
                if not self.tracker.hold_pose(target, yaw):
                    return False
                return True

            exit_target = self.post_unscrew_target(target)
            distance = self.track_or_abort(exit_target, self.config.target_max_velocity, yaw, "UNSCREW_TO_TRANSFER_HEIGHT")
            if distance is None:
                return False
            if distance < self.config.position_tolerance:
                self.state = STATE_TRANSFER_TO_SOCKET
            return True

        if self.state == STATE_TRANSFER_TO_SOCKET:
            # Mirror state1, with only an additional safe-lift waypoint.
            distance = self.track_smooth_task_target(
                "transfer_to_socket", "socket", self.config.socket_center_axis_offset,
                self.config.socket_uav_offset, self.config.smooth_pick_duration,
                self.config.target_max_velocity, yaw, "TRANSFER_TO_SOCKET",
                use_lift_waypoints=True)
            if distance is None:
                return False
            if self.arm.ready_for(STATE_TRANSFER_TO_SOCKET, self.config.require_arm_ready):
                self.state = STATE_FINE_ALIGN_SOCKET
            return True

        if self.state == STATE_FINE_ALIGN_SOCKET:
            # Mirror state2: keep the same socket-side target while the arm
            # finishes the insertion alignment.
            distance = self.track_locked_task_target(
                "transfer_to_socket_target", "socket", self.config.socket_center_axis_offset,
                self.config.socket_uav_offset, self.config.hold_max_velocity, yaw, "FINE_ALIGN_SOCKET")
            if distance is None:
                return False
            if elapsed > self.config.grasp_duration and self.arm.ready_for(STATE_FINE_ALIGN_SOCKET, self.config.require_arm_ready):
                self.transition_to_hold_state(STATE_SCREW_IN_BULB, "screw", yaw)
            return True

        if self.state == STATE_SCREW_IN_BULB:
            # Mirror state3: lock the current UAV hover point while the wrist
            # performs the screw-in action.
            target = self.lock_current_pose("screw")
            if not self.tracker.hold_pose(target, yaw):
                return False
            if elapsed > self.config.screw_duration and self.arm.ready_for(STATE_SCREW_IN_BULB, self.config.require_arm_ready):
                self.state = STATE_FINISH
            return True

        if self.state == STATE_FINISH:
            target = offset_pose(self.communication.default_pose, self.config.retreat_offset)
            self.track_smooth_pose(
                "finish", [target], self.config.smooth_finish_duration,
                self.config.target_max_velocity, yaw,
                "FINISH", use_lift_waypoints=True)
            if elapsed > self.config.finish_duration:
                self.tracker.hold_zero_velocity()
            return True

        if self.state == STATE_ABORT:
            self.state_pub.publish(Int32(data=STATE_ABORT))
            self.tracker.hold_zero_velocity()
            return True

        rospy.logerr("未知 UAV 状态 %s，进入 ABORT", self.state)
        self.state = STATE_ABORT
        return False

    def track_task_target(self, target_name, center_axis_offset, offset, yaw, label):
        source = get_target_pose(self.communication, target_name)
        if source is None:
            rospy.logerr_throttle(1.0, "%s 缺少 %s 位姿", label, target_name)
            return None
        target = target_from_center_pose(source, center_axis_offset, offset, self.axis_local_for_target(target_name))
        return self.track_or_abort(target, self.config.target_max_velocity, yaw, label)

    def track_locked_task_target(self, key, target_name, center_axis_offset, offset, max_velocity, yaw, label):
        if key not in self.locked_targets:
            source = get_target_pose(self.communication, target_name)
            if source is None:
                rospy.logerr_throttle(1.0, "%s 缺少 %s 位姿", label, target_name)
                return None
            self.locked_targets[key] = target_from_center_pose(
                source, center_axis_offset, offset, self.axis_local_for_target(target_name))
            target = self.locked_targets[key]
            rospy.loginfo(
                "%s 锁定目标点: [%.3f %.3f %.3f]",
                label,
                target.position.x,
                target.position.y,
                target.position.z,
            )
        return self.track_or_abort(self.locked_targets[key], max_velocity, yaw, label)

    def track_smooth_task_target(self, key, target_name, center_axis_offset, offset,
                                 duration, max_velocity, yaw, label,
                                 use_lift_waypoints=False):
        source = get_target_pose(self.communication, target_name)
        if source is None:
            return None
        target_key = key + "_target"
        if target_key not in self.locked_targets:
            self.locked_targets[target_key] = target_from_center_pose(
                source, center_axis_offset, offset, self.axis_local_for_target(target_name))
        return self.track_smooth_pose(
            key, [self.locked_targets[target_key]], duration, max_velocity, yaw, label,
            use_lift_waypoints)

    def track_smooth_pose(self, key, goal_poses, duration, max_velocity, yaw, label,
                          use_lift_waypoints=False):
        if not self.config.smooth_trajectory_enabled:
            return self.track_or_abort(goal_poses[-1], max_velocity, yaw, label)
        if key not in self.smooth_trajectories:
            current = self.capture_current_pose()
            waypoints = [current]
            if use_lift_waypoints:
                waypoints.extend(self.lift_waypoints(current, goal_poses[-1]))
            waypoints.extend([copy_pose(pose) for pose in goal_poses])
            self.smooth_trajectories[key] = SmoothTrajectory(
                waypoints, duration, self.config.smooth_min_segment_duration,
            )
            self.smooth_start_times[key] = rospy.Time.now()
        elapsed = (rospy.Time.now() - self.smooth_start_times[key]).to_sec()
        trajectory = self.smooth_trajectories[key]
        reference = trajectory.sample(elapsed)
        reference_velocity = trajectory.velocity(elapsed)
        return self.track_or_abort(
            reference, max_velocity, yaw, label,
            feedforward_velocity=reference_velocity)

    def lift_waypoints(self, start, goal):
        safe_z = max(start.position.z, goal.position.z) + self.config.safe_lift_height
        return [pose_from_xyz(start.position.x, start.position.y, safe_z),
                pose_from_xyz(goal.position.x, goal.position.y, safe_z)]

    def axis_local_for_target(self, target_name):
        return self.config.socket_axis_local if target_name == "socket" else self.config.bulb_axis_local

    def track_or_abort(self, target, max_velocity, yaw, label,
                       feedforward_velocity=None, max_vertical_velocity=None):
        distance = self.tracker.track_pose(
            target, max_velocity, yaw, feedforward_velocity, max_vertical_velocity)
        if distance is None:
            rospy.logerr("%s 控制输入非法，进入 ABORT", label)
        return distance

    def lock_current_pose(self, key):
        if key not in self.locked_targets:
            self.locked_targets[key] = self.capture_current_pose()
            target = self.locked_targets[key]
            rospy.loginfo("%s 锁定 UAV 悬停点: [%.3f %.3f %.3f]", key, target.position.x, target.position.y, target.position.z)
        return self.locked_targets[key]

    def post_unscrew_target(self, hold_target):
        target = Pose()
        target.position.x = hold_target.position.x
        target.position.y = hold_target.position.y
        target.position.z = self.config.default_height
        return target

    def capture_current_pose(self):
        current = self.communication.current_position
        target = Pose()
        target.position.x = current.x
        target.position.y = current.y
        target.position.z = current.z
        return target

    def transition_to_hold_state(self, next_state, lock_key, yaw):
        self.locked_targets[lock_key] = self.capture_current_pose()
        self.tracker.reset_adaptive_state()
        self.tracker.hold_zero_velocity()
        self.state = next_state

    def default_pose(self):
        target = Pose()
        target.position.x = self.communication.default_pose.position.x
        target.position.y = self.communication.default_pose.position.y
        target.position.z = self.communication.default_pose.position.z
        return target

    def elapsed(self):
        return (rospy.Time.now() - self.state_enter_time).to_sec()

    def publish_diagnostics(self):
        velocity = getattr(self.communication, "last_control_velocity", (float("nan"), float("nan"), float("nan")))
        target = getattr(self.communication, "last_control_target", (float("nan"), float("nan"), float("nan")))
        error = getattr(self.communication, "last_control_error", float("nan"))
        rospy.loginfo_throttle(
            1.0,
            "UAV统一控制 state=%d %s elapsed=%.2f err=%.3f target=[%.3f %.3f %.3f] "
            "v=[%.3f %.3f %.3f] arm_ready=%d arm_err=[%.3f %.2f %.3f]",
            self.state,
            state_name(self.state),
            self.elapsed(),
            error,
            target[0],
            target[1],
            target[2],
            velocity[0],
            velocity[1],
            velocity[2],
            self.arm.ready_state,
            self.arm.error[0],
            self.arm.error[1],
            self.arm.error[2],
        )


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def vector_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        value = [float(item) for item in value.strip("[]").split(",")]
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        rospy.logwarn("参数 %s=%s 无效，使用默认值 %s", name, value, default)
        value = default
    return [float(item) for item in value]


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def finite_values(*values):
    return all(math.isfinite(float(value)) for value in values)


def finite_position(position):
    return position is not None and finite_values(position.x, position.y, position.z)


def finite_pose(pose):
    return pose is not None and finite_position(pose.position)


def pose_has_position(pose):
    return any(abs(value) > 1e-6 for value in (pose.position.x, pose.position.y, pose.position.z))


def pose_from_xyz(x, y, z):
    pose = Pose()
    pose.position.x = float(x)
    pose.position.y = float(y)
    pose.position.z = float(z)
    return pose


def copy_pose(pose):
    copied = Pose()
    copied.position = pose.position
    copied.orientation = pose.orientation
    return copied


def interpolate_pose(start, end, blend):
    result = Pose()
    result.position.x = start.position.x + blend * (end.position.x - start.position.x)
    result.position.y = start.position.y + blend * (end.position.y - start.position.y)
    result.position.z = start.position.z + blend * (end.position.z - start.position.z)
    result.orientation = end.orientation
    return result


def pose_distance(first, second):
    return math.sqrt((first.position.x - second.position.x) ** 2
                     + (first.position.y - second.position.y) ** 2
                     + (first.position.z - second.position.z) ** 2)


def stable_local_position(communication):
    return communication.has_local_position and finite_position(communication.current_position) and abs(communication.current_position.z) < 20.0


def limit_horizontal_velocity(vx, vy, max_velocity):
    speed = math.sqrt(vx * vx + vy * vy)
    if speed > max_velocity > 1e-6:
        scale = max_velocity / speed
        return vx * scale, vy * scale
    return vx, vy


def limit_vector_norm(vector, max_norm):
    vector = np.array(vector, dtype=float).reshape(3)
    norm = float(np.linalg.norm(vector))
    if norm > max_norm > 1e-6:
        vector *= float(max_norm) / norm
    return vector


def get_target_pose(communication, target_name):
    pose = communication.target_poses.get(target_name)
    if pose is None or not finite_pose(pose) or not pose_has_position(pose):
        return None
    return pose


def target_from_center_pose(pose, center_axis_offset, offset_xyz, axis_local=None):
    axis_world = pose_axis_world(pose, axis_local or [0.0, 0.0, -1.0])
    target = Pose()
    target.position.x = pose.position.x + axis_world[0] * center_axis_offset + offset_xyz[0]
    target.position.y = pose.position.y + axis_world[1] * center_axis_offset + offset_xyz[1]
    target.position.z = pose.position.z + axis_world[2] * center_axis_offset + offset_xyz[2]
    return target


def pose_axis_world(pose, axis_local):
    axis = np.array(axis_local, dtype=float).reshape(3)
    axis /= max(float(np.linalg.norm(axis)), 1e-9)
    axis_world = quaternion_to_rotation(pose.orientation).dot(axis)
    return axis_world / max(float(np.linalg.norm(axis_world)), 1e-9)


def quaternion_to_rotation(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-9:
        return np.eye(3)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def offset_pose(pose, offset_xyz):
    target = Pose()
    target.position.x = pose.position.x + offset_xyz[0]
    target.position.y = pose.position.y + offset_xyz[1]
    target.position.z = pose.position.z + offset_xyz[2]
    return target


def wait_for_condition(description, predicate, communication, timeout):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if predicate():
            return True
        rate.sleep()
    rospy.logerr(
        "等待%s超时: connected=%s armed=%s mode=%s",
        description,
        communication.mavros_state.connected,
        communication.mavros_state.armed,
        communication.mavros_state.mode,
    )
    return False


def set_offboard_mode(communication, max_attempts=5):
    for attempt in range(1, max_attempts + 1):
        try:
            res = communication.flightModeService(base_mode=0, custom_mode="OFFBOARD")
        except rospy.ServiceException as exc:
            rospy.logwarn("切换 OFFBOARD 服务调用失败: %s", exc)
            res = None
        if communication.mavros_state.mode == "OFFBOARD" or (res is not None and res.mode_sent):
            rospy.loginfo("飞行模式已切换为 OFFBOARD")
            return True
        rospy.logwarn("飞行模式切换失败，第 %d 次重试", attempt)
        rospy.sleep(1.0)
    return False


def arm_vehicle(communication, max_attempts=5):
    for attempt in range(1, max_attempts + 1):
        if communication.mavros_state.armed or communication.arm():
            rospy.loginfo("飞机已解锁")
            return True
        rospy.logwarn("飞机解锁失败，第 %d 次重试", attempt)
        rospy.sleep(2.0)
    return False


def state_name(state):
    return STATE_NAMES.get(state, "UNKNOWN")


def describe_states():
    return ", ".join("%d=%s" % (state, name) for state, name in sorted(STATE_NAMES.items()))


if __name__ == "__main__":
    try:
        BulbUavMission().run()
    except rospy.ROSInterruptException:
        pass
