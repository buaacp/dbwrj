#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import ast
import csv
import math
import os
import threading
import time

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory

from arm_control_bulb import (
    BulbArmController,
    arm_velocity_from_mpc,
    create_serial_port,
    finite_array,
    normalize_vector,
    publish_gripper,
    stow_arm_control,
    zero_arm_control,
)
from utils import ARM


def get_param(name, default):
    private_name = "~" + name
    if rospy.has_param(private_name):
        return rospy.get_param(private_name)
    found_name = rospy.search_param(name)
    if found_name:
        return rospy.get_param(found_name)
    return default


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def parse_vector(value, default, length=3):
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            rospy.logwarn("参数解析失败: %s，使用默认值 %s", value, default)
            value = default
    if not isinstance(value, (list, tuple)) or len(value) != length:
        rospy.logwarn("参数长度错误: %s，使用默认值 %s", value, default)
        value = default
    return [float(item) for item in value]


def parse_waypoints(value):
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            value = []
    if not isinstance(value, (list, tuple)):
        return []
    waypoints = []
    for item in value:
        if isinstance(item, (list, tuple)) and len(item) == 3:
            waypoints.append(np.array(item, dtype=float).reshape(3, 1))
    return waypoints


def axis_from_yaw_pitch(yaw_deg, pitch_deg):
    yaw = math.radians(float(yaw_deg))
    pitch = math.radians(float(pitch_deg))
    axis = np.array([
        [math.cos(pitch) * math.cos(yaw)],
        [math.cos(pitch) * math.sin(yaw)],
        [math.sin(pitch)],
    ])
    return normalize_vector(axis, [0.0, 0.0, -1.0])


def rotation_from_uav(controller):
    phi = controller.uav_arm.phi
    theta = controller.uav_arm.theta
    delta = controller.uav_arm.delta
    rz = np.array([
        [math.cos(delta), -math.sin(delta), 0.0],
        [math.sin(delta), math.cos(delta), 0.0],
        [0.0, 0.0, 1.0],
    ])
    rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, math.cos(theta), -math.sin(theta)],
        [0.0, math.sin(theta), math.cos(theta)],
    ])
    ry = np.array([
        [math.cos(phi), 0.0, math.sin(phi)],
        [0.0, 1.0, 0.0],
        [-math.sin(phi), 0.0, math.cos(phi)],
    ])
    return rx.dot(ry).dot(rz)


def interpolate_waypoints(waypoints, period, elapsed):
    if not waypoints:
        return np.zeros((3, 1))
    if len(waypoints) == 1:
        return waypoints[0]
    segment_count = len(waypoints) - 1
    segment_duration = max(float(period) / segment_count, 1e-3)
    t = min(max(elapsed, 0.0), float(period))
    index = min(int(t / segment_duration), segment_count - 1)
    alpha = (t - index * segment_duration) / segment_duration
    return (1.0 - alpha) * waypoints[index] + alpha * waypoints[index + 1]


def interpolate_waypoint_velocity(waypoints, period, elapsed):
    if not waypoints or len(waypoints) == 1:
        return np.zeros((3, 1))
    segment_count = len(waypoints) - 1
    segment_duration = max(float(period) / segment_count, 1e-3)
    t = min(max(elapsed, 0.0), float(period))
    index = min(int(t / segment_duration), segment_count - 1)
    return (waypoints[index + 1] - waypoints[index]) / segment_duration


def target_local(path_type, elapsed, center, amplitude, period, waypoints):
    if path_type == "line":
        phase = min(max(elapsed / max(period, 1e-3), 0.0), 1.0)
        offset = np.array([
            [amplitude[0] * (2.0 * phase - 1.0)],
            [amplitude[1] * (2.0 * phase - 1.0)],
            [amplitude[2] * (2.0 * phase - 1.0)],
        ])
        return center + offset
    if path_type == "circle":
        angle = 2.0 * math.pi * elapsed / max(period, 1e-3)
        offset = np.array([
            [amplitude[0] * math.cos(angle)],
            [amplitude[1] * math.sin(angle)],
            [amplitude[2] * math.sin(angle)],
        ])
        return center + offset
    if path_type == "waypoints":
        return interpolate_waypoints(waypoints, period, elapsed)
    return center


def target_local_velocity(path_type, elapsed, amplitude, period, waypoints):
    period = max(float(period), 1e-3)
    if path_type == "line":
        if elapsed < 0.0 or elapsed > period:
            return np.zeros((3, 1))
        return np.array([
            [2.0 * amplitude[0] / period],
            [2.0 * amplitude[1] / period],
            [2.0 * amplitude[2] / period],
        ])
    if path_type == "circle":
        omega = 2.0 * math.pi / period
        angle = omega * elapsed
        return np.array([
            [-amplitude[0] * omega * math.sin(angle)],
            [amplitude[1] * omega * math.cos(angle)],
            [amplitude[2] * omega * math.cos(angle)],
        ])
    if path_type == "waypoints":
        return interpolate_waypoint_velocity(waypoints, period, elapsed)
    return np.zeros((3, 1))


def progress_scale_from_error(error, slow_error, stop_error):
    slow_error = max(float(slow_error), 1e-6)
    stop_error = max(float(stop_error), slow_error + 1e-6)
    if error <= slow_error:
        return 1.0
    if error >= stop_error:
        return 0.0
    return 1.0 - (error - slow_error) / (stop_error - slow_error)


class EndpointPathTester:
    def __init__(self):
        rospy.init_node("arm_endpoint_path_test")
        self.servo_ids = [int(item) for item in parse_vector(get_param("servo_ids", [0, 1, 2, 3]), [0, 1, 2, 3], 4)]
        self.if_simulation = rospy.get_param("/if_simulation", True)
        self.servo_port_name = get_param("servo_port_name", "/dev/ttyUSB0")
        self.servo_baudrate = int(get_param("servo_baudrate", 115200))
        self.path_type = str(get_param("path_type", "circle")).strip().lower()
        self.frame = str(get_param("target_frame", "body")).strip().lower()
        self.center = np.array(parse_vector(get_param("path_center", [0.0, 0.0, -0.24]), [0.0, 0.0, -0.24]), dtype=float).reshape(3, 1)
        self.amplitude = parse_vector(get_param("path_amplitude", [0.04, 0.04, 0.0]), [0.04, 0.04, 0.0])
        self.period = float(get_param("path_period", 12.0))
        self.duration = float(get_param("test_duration", 30.0))
        self.rate_hz = float(get_param("rate_hz", 20.0))
        self.target_axis = axis_from_yaw_pitch(
            float(get_param("target_axis_yaw_deg", 0.0)),
            float(get_param("target_axis_pitch_deg", -90.0)),
        )
        self.waypoints = parse_waypoints(get_param("waypoints", []))
        self.use_current_center = parse_bool(get_param("use_current_center", True))
        self.use_current_axis = parse_bool(get_param("use_current_axis", True))
        self.enable_gripper_open = parse_bool(get_param("enable_gripper_open", True))
        self.gripper_open = float(get_param("gripper_open", 0.55))
        self.output_csv = str(get_param("output_csv", ""))
        self.stow_before_start = parse_bool(get_param("stow_before_start", False))
        self.stow_joint_rad = np.deg2rad(parse_vector(get_param("stow_joint_deg", [-45.0, 90.0, 45.0, 0.0]), [-45.0, 90.0, 45.0, 0.0], 4))
        self.enable_uav_workspace_tracking = parse_bool(get_param("enable_uav_workspace_tracking", True))
        self.uav_auto_offboard = parse_bool(get_param("uav_auto_offboard", False))
        self.uav_workspace_desired_local = np.array(
            parse_vector(get_param("uav_workspace_desired_local", [0.0, 0.0, -0.22]), [0.0, 0.0, -0.22]),
            dtype=float,
        ).reshape(3, 1)
        self.use_current_workspace_desired = parse_bool(get_param("use_current_workspace_desired", True))
        self.uav_workspace_gain = float(get_param("uav_workspace_gain", 0.45))
        self.uav_workspace_z_scale = float(get_param("uav_workspace_z_scale", 0.35))
        self.uav_max_xy_velocity = float(get_param("uav_max_xy_velocity", 0.18))
        self.uav_max_z_velocity = float(get_param("uav_max_z_velocity", 0.08))
        self.uav_velocity_feedforward_gain = float(get_param("uav_velocity_feedforward_gain", 0.0))
        self.uav_follow_mode = str(get_param("uav_follow_mode", "workspace")).strip().lower()
        self.uav_position_gain = float(get_param("uav_position_gain", 0.6))
        self.uav_position_z_gain = float(get_param("uav_position_z_gain", 0.25))
        self.uav_feedforward_scale_with_error = parse_bool(get_param("uav_feedforward_scale_with_error", True))
        self.enable_adaptive_progress = parse_bool(get_param("enable_adaptive_progress", True))
        self.continuous_start = parse_bool(get_param("continuous_start", True))
        self.adaptive_min_scale = float(get_param("adaptive_min_scale", 0.0))
        self.adaptive_pos_slow_error = float(get_param("adaptive_pos_slow_error", 0.08))
        self.adaptive_pos_stop_error = float(get_param("adaptive_pos_stop_error", 0.25))
        self.adaptive_axis_slow_deg = float(get_param("adaptive_axis_slow_deg", 8.0))
        self.adaptive_axis_stop_deg = float(get_param("adaptive_axis_stop_deg", 30.0))
        self.adaptive_centerline_slow_error = float(get_param("adaptive_centerline_slow_error", 0.05))
        self.adaptive_centerline_stop_error = float(get_param("adaptive_centerline_stop_error", 0.20))
        self.adaptive_workspace_slow_error = float(get_param("adaptive_workspace_slow_error", 0.20))
        self.adaptive_workspace_stop_error = float(get_param("adaptive_workspace_stop_error", 0.55))
        self.adaptive_uav_saturation_slow = float(get_param("adaptive_uav_saturation_slow", 0.80))
        self.adaptive_uav_saturation_min_scale = float(get_param("adaptive_uav_saturation_min_scale", 0.25))
        self.mavros_state = State()
        if self.enable_uav_workspace_tracking and self.frame != "world":
            rospy.logwarn("enable_uav_workspace_tracking 只对 target_frame=world 有效；当前为 %s，已关闭 UAV 协同", self.frame)
            self.enable_uav_workspace_tracking = False

        uart = None if self.if_simulation else create_serial_port(self.servo_port_name, self.servo_baudrate)
        self.arm = ARM.ARM(uart=uart, SERVO_IDS=self.servo_ids, if_simulation=self.if_simulation)
        self.controller = BulbArmController(self.arm, self.servo_ids)
        self.pub_angular = rospy.Publisher("/le_arm_controller/command", Float64MultiArray, queue_size=10)
        self.pub_gripper = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
        self.pub_uav = rospy.Publisher("mavros/setpoint_raw/local", PositionTarget, queue_size=10)
        self.arm_service = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.set_mode_service = rospy.ServiceProxy("mavros/set_mode", SetMode)
        self.file_obj = None
        self.writer = None
        if self.output_csv:
            output_dir = os.path.dirname(os.path.abspath(self.output_csv))
            if output_dir and not os.path.isdir(output_dir):
                os.makedirs(output_dir)
            self.file_obj = open(self.output_csv, "w", newline="")
            self.writer = csv.writer(self.file_obj)
            self.writer.writerow([
                "t",
                "target_x",
                "target_y",
                "target_z",
                "gripper_x",
                "gripper_y",
                "gripper_z",
                "pos_error",
                "axis_error_deg",
                "centerline_error",
                "target_local_x",
                "target_local_y",
                "target_local_z",
                "uav_cmd_vx",
                "uav_cmd_vy",
                "uav_cmd_vz",
                "uav_ff_vx",
                "uav_ff_vy",
                "uav_ff_vz",
                "uav_x",
                "uav_y",
                "uav_z",
                "path_progress",
                "path_progress_scale",
                "uav_ref_x",
                "uav_ref_y",
                "uav_ref_z",
                "uav_ref_error",
            ])

        self.thread = threading.Thread(target=self.query_state_continuously)
        self.thread.daemon = True
        self.thread.start()

    def query_state_continuously(self):
        rospy.Subscriber("mavros/state", State, self.mavros_state_callback)
        rospy.Subscriber("/joint_states", JointState, self.arm.joint_states_callback)
        rospy.Subscriber("mavros/vision_pose/pose", PoseStamped, self.controller.uav.vision_imu_callback)
        rospy.Subscriber("mavros/vision_pose/pose", PoseStamped, self.controller.uav.vision_pose_callback)
        rospy.Subscriber("mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, self.controller.uav.velocity_callback)
        rospy.Subscriber("/clock", Clock, self.controller.uav_arm.clock_callback)
        rospy.spin()

    def mavros_state_callback(self, msg):
        self.mavros_state = msg

    def local_pose_callback(self, msg):
        self.controller.uav.pose_callback(msg)
        self.controller.uav.vision_pose_callback(msg)

    def target_world(self, local_target):
        if self.frame == "world":
            return local_target, self.target_axis
        self.controller.update_uav_arm_state()
        rb = rotation_from_uav(self.controller)
        world_position = self.controller.uav_arm.p_b + rb.dot(self.controller.uav_arm.p_delta + local_target)
        world_axis = normalize_vector(rb.dot(self.target_axis), [0.0, 0.0, -1.0])
        return world_position, world_axis

    def target_world_velocity(self, local_velocity):
        if self.frame == "world":
            return local_velocity
        self.controller.update_uav_arm_state()
        rb = rotation_from_uav(self.controller)
        return rb.dot(local_velocity)

    def current_local_gripper_pose(self):
        self.controller.update_uav_arm_state()
        gripper_position, gripper_axis = self.controller.current_gripper_pose()
        rb = rotation_from_uav(self.controller)
        local_position = np.linalg.inv(rb).dot(gripper_position - self.controller.uav_arm.p_b) - self.controller.uav_arm.p_delta
        local_axis = normalize_vector(np.linalg.inv(rb).dot(gripper_axis), [0.0, 0.0, -1.0])
        return local_position, local_axis

    def target_local_from_world(self, world_target):
        self.controller.update_uav_arm_state()
        rb = rotation_from_uav(self.controller)
        return np.linalg.inv(rb).dot(world_target - self.controller.uav_arm.p_b) - self.controller.uav_arm.p_delta

    def limited_uav_workspace_velocity(self, world_target, world_target_velocity):
        target_local_now = self.target_local_from_world(world_target)
        if not finite_array(target_local_now):
            rospy.logerr("UAV局部目标含 NaN/Inf，停止UAV速度输出")
            return np.zeros(3), target_local_now, np.zeros(3)
        local_error = target_local_now - self.uav_workspace_desired_local
        local_cmd = self.uav_workspace_gain * local_error
        local_cmd[2, 0] *= self.uav_workspace_z_scale
        rb = rotation_from_uav(self.controller)
        feedforward = self.uav_velocity_feedforward_gain * world_target_velocity.reshape(3)
        velocity = rb.dot(local_cmd).reshape(3) + feedforward
        if not finite_array(velocity):
            rospy.logerr("UAV速度命令含 NaN/Inf，停止UAV速度输出")
            return np.zeros(3), target_local_now, np.zeros(3)
        speed_xy = math.sqrt(velocity[0] ** 2 + velocity[1] ** 2)
        if speed_xy > self.uav_max_xy_velocity > 1e-6:
            scale = self.uav_max_xy_velocity / speed_xy
            velocity[0] *= scale
            velocity[1] *= scale
        velocity[2] = max(min(velocity[2], self.uav_max_z_velocity), -self.uav_max_z_velocity)
        return velocity, target_local_now, feedforward

    def desired_uav_position_for_target(self, world_target):
        self.controller.update_uav_arm_state()
        rb = rotation_from_uav(self.controller)
        return world_target - rb.dot(self.controller.uav_arm.p_delta + self.uav_workspace_desired_local)

    def limited_uav_reference_velocity(self, world_target, world_target_velocity, speed_scale):
        self.controller.update_uav_arm_state()
        rb = rotation_from_uav(self.controller)
        target_local_now = self.target_local_from_world(world_target)
        if not finite_array(target_local_now):
            rospy.logerr("UAV局部目标含 NaN/Inf，停止UAV速度输出")
            return np.zeros(3), target_local_now, np.zeros(3), self.controller.uav_arm.p_b, 0.0
        uav_position = self.controller.uav_arm.p_b
        uav_ref = self.desired_uav_position_for_target(world_target)
        position_error = uav_ref - uav_position
        feedback = np.array([
            self.uav_position_gain * position_error[0, 0],
            self.uav_position_gain * position_error[1, 0],
            self.uav_position_z_gain * position_error[2, 0],
        ])
        ff_scale = speed_scale if self.uav_feedforward_scale_with_error else 1.0
        feedforward = self.uav_velocity_feedforward_gain * ff_scale * world_target_velocity.reshape(3)
        velocity = feedback + feedforward
        if not finite_array(velocity):
            rospy.logerr("UAV参考轨迹速度命令含 NaN/Inf，停止UAV速度输出")
            return np.zeros(3), target_local_now, np.zeros(3), uav_ref, float(np.linalg.norm(position_error))
        speed_xy = math.sqrt(velocity[0] ** 2 + velocity[1] ** 2)
        if speed_xy > self.uav_max_xy_velocity > 1e-6:
            scale = self.uav_max_xy_velocity / speed_xy
            velocity[0] *= scale
            velocity[1] *= scale
        velocity[2] = max(min(velocity[2], self.uav_max_z_velocity), -self.uav_max_z_velocity)
        return velocity, target_local_now, feedforward, uav_ref, float(np.linalg.norm(position_error))

    def publish_uav_velocity(self, velocity):
        msg = PositionTarget()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "map"
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        msg.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW |
            PositionTarget.IGNORE_YAW_RATE
        )
        if finite_array(velocity):
            msg.velocity.x = float(velocity[0])
            msg.velocity.y = float(velocity[1])
            msg.velocity.z = float(velocity[2])
        self.pub_uav.publish(msg)

    def publish_zero_uav_velocity(self):
        self.publish_uav_velocity(np.zeros(3))

    def enter_offboard_if_requested(self):
        if not self.enable_uav_workspace_tracking or not self.uav_auto_offboard:
            return True
        try:
            rospy.wait_for_service("mavros/cmd/arming", timeout=30.0)
            rospy.wait_for_service("mavros/set_mode", timeout=30.0)
        except rospy.ROSException as exc:
            rospy.logerr("等待 MAVROS 服务超时: %s", exc)
            return False
        warmup_end = rospy.Time.now() + rospy.Duration(3.0)
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and rospy.Time.now() < warmup_end:
            self.publish_zero_uav_velocity()
            rate.sleep()
        deadline = rospy.Time.now() + rospy.Duration(8.0)
        last_mode_request = rospy.Time(0)
        last_arm_request = rospy.Time(0)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            self.publish_zero_uav_velocity()
            now = rospy.Time.now()
            try:
                if self.mavros_state.mode != "OFFBOARD" and (now - last_mode_request).to_sec() > 1.0:
                    mode_res = self.set_mode_service(base_mode=0, custom_mode="OFFBOARD")
                    last_mode_request = now
                    rospy.loginfo("UAV切OFFBOARD: mode_sent=%s", getattr(mode_res, "mode_sent", None))
                if not self.mavros_state.armed and (now - last_arm_request).to_sec() > 1.0:
                    arm_res = self.arm_service(True)
                    last_arm_request = now
                    rospy.loginfo("UAV解锁: arm=%s", getattr(arm_res, "success", None))
            except rospy.ServiceException as exc:
                rospy.logerr("UAV自动解锁/OFFBOARD 服务调用失败: %s", exc)
                return False
            if self.mavros_state.mode == "OFFBOARD" and self.mavros_state.armed:
                rospy.loginfo("UAV已进入OFFBOARD并解锁")
                return True
            rate.sleep()
        rospy.logerr(
            "UAV未能进入可测试状态: connected=%s armed=%s mode=%s",
            self.mavros_state.connected,
            self.mavros_state.armed,
            self.mavros_state.mode,
        )
        return False

    def publish_arm_command(self, command):
        msg = Float64MultiArray()
        msg.data = command
        self.pub_angular.publish(msg)

    def adaptive_progress_scale(self, pos_error, axis_error, line_error, target_local_now, uav_velocity, uav_ref_error=0.0):
        if not self.enable_adaptive_progress:
            return 1.0
        workspace_error = float(np.linalg.norm(target_local_now - self.uav_workspace_desired_local))
        pos_scale = progress_scale_from_error(pos_error, self.adaptive_pos_slow_error, self.adaptive_pos_stop_error)
        axis_scale = progress_scale_from_error(axis_error, self.adaptive_axis_slow_deg, self.adaptive_axis_stop_deg)
        centerline_scale = progress_scale_from_error(line_error, self.adaptive_centerline_slow_error, self.adaptive_centerline_stop_error)
        workspace_scale = progress_scale_from_error(workspace_error, self.adaptive_workspace_slow_error, self.adaptive_workspace_stop_error)
        uav_ref_scale = progress_scale_from_error(uav_ref_error, self.adaptive_workspace_slow_error, self.adaptive_workspace_stop_error)
        speed_xy = math.sqrt(uav_velocity[0] ** 2 + uav_velocity[1] ** 2)
        saturation = speed_xy / max(self.uav_max_xy_velocity, 1e-6)
        if saturation <= self.adaptive_uav_saturation_slow:
            saturation_scale = 1.0
        else:
            span = max(1.0 - self.adaptive_uav_saturation_slow, 1e-6)
            saturation_scale = 1.0 - (saturation - self.adaptive_uav_saturation_slow) / span
            saturation_scale = max(self.adaptive_uav_saturation_min_scale, min(1.0, saturation_scale))
        scale = min(pos_scale, axis_scale, centerline_scale, workspace_scale, uav_ref_scale, saturation_scale)
        return max(self.adaptive_min_scale, min(1.0, scale))

    def apply_continuous_start(self, current_center):
        if not self.continuous_start:
            return
        amplitude = np.array(self.amplitude, dtype=float).reshape(3, 1)
        if self.path_type == "circle":
            self.center = current_center - np.array([[amplitude[0, 0]], [0.0], [0.0]])
            rospy.loginfo("连续起步: 圆轨迹中心调整为 %s，使 path_progress=0 对齐当前爪心", self.center.flatten().tolist())
        elif self.path_type == "line":
            self.center = current_center + amplitude
            rospy.loginfo("连续起步: 线轨迹中心调整为 %s，使 path_progress=0 对齐当前爪心", self.center.flatten().tolist())
        elif self.path_type == "waypoints" and self.waypoints:
            offset = current_center - self.waypoints[0]
            self.waypoints = [waypoint + offset for waypoint in self.waypoints]
            rospy.loginfo("连续起步: waypoints 平移 %.3f %.3f %.3f", offset[0, 0], offset[1, 0], offset[2, 0])

    def write_log(
        self,
        elapsed,
        target,
        gripper_position,
        pos_error,
        axis_error,
        line_error,
        target_local_now,
        uav_velocity,
        uav_feedforward,
        path_progress,
        path_progress_scale,
        uav_ref,
        uav_ref_error,
    ):
        if self.writer is None or gripper_position is None:
            return
        self.controller.update_uav_arm_state()
        uav_position = self.controller.uav_arm.p_b
        self.writer.writerow([
            "%.4f" % elapsed,
            "%.6f" % target[0, 0],
            "%.6f" % target[1, 0],
            "%.6f" % target[2, 0],
            "%.6f" % gripper_position[0, 0],
            "%.6f" % gripper_position[1, 0],
            "%.6f" % gripper_position[2, 0],
            "%.6f" % pos_error,
            "%.6f" % axis_error,
            "%.6f" % line_error,
            "%.6f" % target_local_now[0, 0],
            "%.6f" % target_local_now[1, 0],
            "%.6f" % target_local_now[2, 0],
            "%.6f" % uav_velocity[0],
            "%.6f" % uav_velocity[1],
            "%.6f" % uav_velocity[2],
            "%.6f" % uav_feedforward[0],
            "%.6f" % uav_feedforward[1],
            "%.6f" % uav_feedforward[2],
            "%.6f" % uav_position[0, 0],
            "%.6f" % uav_position[1, 0],
            "%.6f" % uav_position[2, 0],
            "%.6f" % path_progress,
            "%.6f" % path_progress_scale,
            "%.6f" % uav_ref[0, 0],
            "%.6f" % uav_ref[1, 0],
            "%.6f" % uav_ref[2, 0],
            "%.6f" % uav_ref_error,
        ])

    def run(self):
        rospy.loginfo(
            "末端路径跟踪测试启动: path=%s frame=%s center=%s amplitude=%s duration=%.1fs",
            self.path_type,
            self.frame,
            self.center.flatten().tolist(),
            self.amplitude,
            self.duration,
        )
        rate = rospy.Rate(self.rate_hz)
        start_time = rospy.Time.now()
        while not rospy.is_shutdown() and not self.controller.ready():
            rospy.logwarn_throttle(1.0, "等待机械臂 joint_states")
            self.publish_arm_command(zero_arm_control(self.servo_ids))
            rate.sleep()

        if self.use_current_center or self.use_current_axis:
            current_local_center, current_local_axis = self.current_local_gripper_pose()
            current_world_center, current_world_axis = self.controller.current_gripper_pose()
            if self.use_current_center:
                self.center = current_world_center if self.frame == "world" else current_local_center
                self.apply_continuous_start(self.center)
                rospy.loginfo("使用当前爪心作为路径中心: %s", self.center.flatten().tolist())
            if self.use_current_axis:
                self.target_axis = current_world_axis if self.frame == "world" else current_local_axis
                rospy.loginfo("使用当前末端方向作为期望方向: %s", self.target_axis.flatten().tolist())

        if not self.enter_offboard_if_requested():
            return

        if self.enable_uav_workspace_tracking and self.use_current_workspace_desired:
            initial_target = target_local(
                self.path_type,
                0.0,
                self.center,
                self.amplitude,
                self.period,
                self.waypoints,
            )
            initial_world_target, _ = self.target_world(initial_target)
            self.uav_workspace_desired_local = self.target_local_from_world(initial_world_target)
            rospy.loginfo("使用当前目标局部位置作为UAV舒适工作点: %s", self.uav_workspace_desired_local.flatten().tolist())

        if self.stow_before_start:
            stow_until = rospy.Time.now() + rospy.Duration(2.0)
            while not rospy.is_shutdown() and rospy.Time.now() < stow_until:
                command = stow_arm_control(self.arm, self.servo_ids, self.stow_joint_rad, 1.2, 0.4)
                self.publish_arm_command(command)
                rate.sleep()
            start_time = rospy.Time.now()

        path_progress = 0.0
        path_progress_scale = 1.0
        last_loop_time = rospy.Time.now()
        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now()
                elapsed = (now - start_time).to_sec()
                if elapsed > self.duration:
                    break
                dt = max(0.0, min((now - last_loop_time).to_sec(), 0.2))
                last_loop_time = now
                if self.enable_gripper_open:
                    publish_gripper(self.pub_gripper, self.gripper_open)

                local_target = target_local(
                    self.path_type,
                    path_progress,
                    self.center,
                    self.amplitude,
                    self.period,
                    self.waypoints,
                )
                local_target_velocity = target_local_velocity(self.path_type, path_progress, self.amplitude, self.period, self.waypoints)
                local_target_velocity *= path_progress_scale
                world_target, world_axis = self.target_world(local_target)
                world_target_velocity = self.target_world_velocity(local_target_velocity)
                uav_velocity = np.zeros(3)
                uav_feedforward = np.zeros(3)
                uav_ref = self.controller.uav_arm.p_b.copy()
                uav_ref_error = 0.0
                target_local_now = self.target_local_from_world(world_target)
                if not finite_array(world_target) or not finite_array(world_axis) or not finite_array(target_local_now):
                    rospy.logerr("路径目标或UAV状态含 NaN/Inf，结束测试")
                    break
                if self.enable_uav_workspace_tracking:
                    if self.uav_follow_mode == "trajectory":
                        uav_velocity, target_local_now, uav_feedforward, uav_ref, uav_ref_error = self.limited_uav_reference_velocity(
                            world_target,
                            world_target_velocity,
                            path_progress_scale,
                        )
                    else:
                        uav_velocity, target_local_now, uav_feedforward = self.limited_uav_workspace_velocity(world_target, world_target_velocity)
                        uav_ref = self.desired_uav_position_for_target(world_target)
                        uav_ref_error = float(np.linalg.norm(uav_ref - self.controller.uav_arm.p_b))
                    if not finite_array(uav_velocity):
                        rospy.logerr("UAV速度命令无效，结束测试")
                        break
                    self.publish_uav_velocity(uav_velocity)
                    rospy.loginfo_throttle(
                        1.0,
                        "UAV跟踪(%s): target_local=[%.3f %.3f %.3f] desired=[%.3f %.3f %.3f] uav_ref_err=%.3f v=[%.3f %.3f %.3f] ff=[%.3f %.3f %.3f]",
                        self.uav_follow_mode,
                        target_local_now[0, 0],
                        target_local_now[1, 0],
                        target_local_now[2, 0],
                        self.uav_workspace_desired_local[0, 0],
                        self.uav_workspace_desired_local[1, 0],
                        self.uav_workspace_desired_local[2, 0],
                        uav_ref_error,
                        uav_velocity[0],
                        uav_velocity[1],
                        uav_velocity[2],
                        uav_feedforward[0],
                        uav_feedforward[1],
                        uav_feedforward[2],
                    )
                command, pos_error, axis_error, line_error = self.controller.track_world_target(world_target, world_axis)
                self.publish_arm_command(command)

                gripper_position = None
                if self.controller.ready():
                    gripper_position, _ = self.controller.current_gripper_pose()
                if finite_array([pos_error, axis_error, line_error]):
                    path_progress_scale = self.adaptive_progress_scale(
                        pos_error,
                        axis_error,
                        line_error,
                        target_local_now,
                        uav_velocity,
                        uav_ref_error,
                    )
                    self.write_log(
                        elapsed,
                        world_target,
                        gripper_position,
                        pos_error,
                        axis_error,
                        line_error,
                        target_local_now,
                        uav_velocity,
                        uav_feedforward,
                        path_progress,
                        path_progress_scale,
                        uav_ref,
                        uav_ref_error,
                    )
                    rospy.loginfo_throttle(
                        1.0,
                        "路径跟踪误差: pos=%.4f m axis=%.2f deg centerline=%.4f m progress=%.2f scale=%.2f",
                        pos_error,
                        axis_error,
                        line_error,
                        path_progress,
                        path_progress_scale,
                    )
                    path_progress += dt * path_progress_scale
                    if self.path_type in ("line", "waypoints"):
                        path_progress = min(path_progress, self.period)
                rate.sleep()
        finally:
            self.publish_arm_command(zero_arm_control(self.servo_ids))
            if self.enable_uav_workspace_tracking:
                self.publish_zero_uav_velocity()
            if self.file_obj is not None:
                self.file_obj.flush()
                self.file_obj.close()
                rospy.loginfo("末端路径跟踪日志已保存: %s", self.output_csv)


if __name__ == "__main__":
    try:
        EndpointPathTester().run()
    except rospy.ROSInterruptException:
        pass
