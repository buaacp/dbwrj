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
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray


ARM_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
]
WRIST_ROLL_JOINT = "wrist_roll_joint"


def get_vector_param(name, default, length):
    value = rospy.get_param("~" + name, default)
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            rospy.logwarn("参数 %s=%s 解析失败，使用默认值 %s", name, value, default)
            value = default
    if not isinstance(value, (list, tuple)) or len(value) != length:
        rospy.logwarn("参数 %s 长度不正确，使用默认值 %s", name, default)
        value = default
    return np.array([float(item) for item in value], dtype=float)


def get_bool_param(name, default):
    value = rospy.get_param("~" + name, default)
    if isinstance(value, str):
        return value.strip().lower() in ("true", "1", "yes", "on")
    return bool(value)


def skew(v):
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])


def rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_z(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def rpy_to_rot(rpy):
    roll, pitch, yaw = rpy
    return rot_z(yaw).dot(rot_y(pitch)).dot(rot_x(roll))


def axis_angle_to_rot(axis, angle):
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm < 1e-9:
        return np.eye(3)
    axis = axis / norm
    K = skew(axis)
    return np.eye(3) + math.sin(angle) * K + (1.0 - math.cos(angle)) * K.dot(K)


def transform_matrix(xyz, rpy):
    mat = np.eye(4)
    mat[:3, :3] = rpy_to_rot(rpy)
    mat[:3, 3] = xyz
    return mat


def joint_transform(xyz, rpy, axis, q):
    mat = transform_matrix(np.array(xyz, dtype=float), np.array(rpy, dtype=float))
    rot = np.eye(4)
    rot[:3, :3] = axis_angle_to_rot(axis, q)
    return mat.dot(rot)


def quaternion_to_rot(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_to_quaternion(rot):
    trace = np.trace(rot)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    elif rot[0, 0] > rot[1, 1] and rot[0, 0] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
        w = (rot[2, 1] - rot[1, 2]) / s
        x = 0.25 * s
        y = (rot[0, 1] + rot[1, 0]) / s
        z = (rot[0, 2] + rot[2, 0]) / s
    elif rot[1, 1] > rot[2, 2]:
        s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
        w = (rot[0, 2] - rot[2, 0]) / s
        x = (rot[0, 1] + rot[1, 0]) / s
        y = 0.25 * s
        z = (rot[1, 2] + rot[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
        w = (rot[1, 0] - rot[0, 1]) / s
        x = (rot[0, 2] + rot[2, 0]) / s
        y = (rot[1, 2] + rot[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w])


def rot_to_rpy(rot):
    pitch = math.asin(max(min(-rot[2, 0], 1.0), -1.0))
    if abs(math.cos(pitch)) > 1e-8:
        roll = math.atan2(rot[2, 1], rot[2, 2])
        yaw = math.atan2(rot[1, 0], rot[0, 0])
    else:
        roll = 0.0
        yaw = math.atan2(-rot[0, 1], rot[1, 1])
    return np.array([roll, pitch, yaw], dtype=float)


def rot_log(rot):
    cos_angle = (np.trace(rot) - 1.0) * 0.5
    cos_angle = max(min(cos_angle, 1.0), -1.0)
    angle = math.acos(cos_angle)
    if angle < 1e-8:
        return np.zeros(3)
    return angle / (2.0 * math.sin(angle)) * np.array([
        rot[2, 1] - rot[1, 2],
        rot[0, 2] - rot[2, 0],
        rot[1, 0] - rot[0, 1],
    ])


def pose_to_arrays(msg):
    p = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z], dtype=float)
    q = np.array([msg.pose.orientation.x, msg.pose.orientation.y,
                  msg.pose.orientation.z, msg.pose.orientation.w], dtype=float)
    return p, quaternion_to_rot(q)


class WholeBodyPoseController:
    def __init__(self):
        self.lock = threading.Lock()
        self.vehicle_ns = rospy.get_param("~vehicle_ns", "/iris_0").rstrip("/")
        self.control_uav = get_bool_param("control_uav", True)
        self.control_arm = get_bool_param("control_arm", True)
        self.control_wrist_roll = get_bool_param("control_wrist_roll", True)
        self.auto_arm_offboard = get_bool_param("auto_arm_offboard", True)

        self.rate_hz = float(rospy.get_param("~rate", 20.0))
        self.setpoint_warmup_time = float(rospy.get_param("~setpoint_warmup_time", 3.0))
        self.arm_attempts = int(rospy.get_param("~arm_attempts", 5))
        self.mode_attempts = int(rospy.get_param("~mode_attempts", 5))
        self.kp_pos = float(rospy.get_param("~kp_pos", 0.3))
        self.kp_rot = float(rospy.get_param("~kp_rot", 0.2))
        self.damping = float(rospy.get_param("~damping", 0.15))
        self.jacobian_eps = float(rospy.get_param("~jacobian_eps", 1e-5))

        self.max_uav_vel = float(rospy.get_param("~max_uav_velocity", 0.12))
        self.max_uav_yaw_rate = float(rospy.get_param("~max_uav_yaw_rate", 0.15))
        self.max_arm_vel = float(rospy.get_param("~max_arm_velocity", 0.12))
        self.max_wrist_roll_vel = float(rospy.get_param("~max_wrist_roll_velocity", 0.2))
        self.wrist_roll_min = float(rospy.get_param("~wrist_roll_min", -math.pi))
        self.wrist_roll_max = float(rospy.get_param("~wrist_roll_max", math.pi))

        self.arm_mount_xyz = get_vector_param("arm_mount_xyz", [-0.1, 0.0, -0.05], 3)
        self.arm_mount_rpy = get_vector_param("arm_mount_rpy", [0.0, math.pi, math.pi], 3)
        self.gripper_center = get_vector_param("gripper_center", [-0.00393, 0.00124, 0.10256], 3)
        self.default_target_position = get_vector_param("default_target_position", [0.0, 0.0, 1.0], 3)
        self.default_target_rpy = get_vector_param("default_target_rpy", [0.0, 0.0, 0.0], 3)
        self.log_enabled = get_bool_param("log_enabled", True)
        self.log_rate_hz = float(rospy.get_param("~log_rate", self.rate_hz))
        self.log_csv = str(rospy.get_param("~log_csv", ""))
        self.log_directory = str(rospy.get_param("~log_directory", ""))

        self.current_uav_p = None
        self.current_uav_R = None
        self.mavros_state = State()
        self.target_p = self.default_target_position.copy()
        self.target_R = rpy_to_rot(self.default_target_rpy)
        self.target_stamp = rospy.Time(0)
        self.joint_positions = {}
        self.integrated_wrist_roll_cmd = None
        self.log_file = None
        self.log_writer = None
        self.log_start_time = None
        self.last_log_time = rospy.Time(0)
        self.log_rows = 0

        self.arm_joint_limits = np.array([
            [-1.57, 1.57],
            [-1.57, 1.57],
            [-1.57, 1.57],
            [-1.57, 1.57],
            [self.wrist_roll_min, self.wrist_roll_max],
        ], dtype=float)

        target_pose_topic = rospy.get_param("~target_pose_topic", "/whole_body_control/target_pose")
        rospy.Subscriber(self.vehicle_ns + "/mavros/state", State, self.mavros_state_callback, queue_size=1)
        rospy.Subscriber(self.vehicle_ns + "/mavros/local_position/pose", PoseStamped, self.uav_pose_callback, queue_size=1)
        rospy.Subscriber("/joint_states", JointState, self.joint_state_callback, queue_size=1)
        rospy.Subscriber(target_pose_topic, PoseStamped, self.target_pose_callback, queue_size=1)

        self.uav_pub = rospy.Publisher(self.vehicle_ns + "/mavros/setpoint_raw/local", PositionTarget, queue_size=1)
        self.arm_pub = rospy.Publisher("/le_arm_controller/command", Float64MultiArray, queue_size=1)
        self.wrist_pub = rospy.Publisher("/wrist_roll_controller/command", Float64, queue_size=1)
        self.current_pose_pub = rospy.Publisher("~current_gripper_pose", PoseStamped, queue_size=1)
        self.error_pub = rospy.Publisher("~task_error", Float64MultiArray, queue_size=1)
        self.arm_service = rospy.ServiceProxy(self.vehicle_ns + "/mavros/cmd/arming", CommandBool)
        self.set_mode_service = rospy.ServiceProxy(self.vehicle_ns + "/mavros/set_mode", SetMode)
        self.setup_logger()

        rospy.loginfo("whole_body_pose_control target topic: %s", target_pose_topic)
        rospy.loginfo("control_uav=%s control_arm=%s control_wrist_roll=%s auto_arm_offboard=%s",
                      self.control_uav, self.control_arm, self.control_wrist_roll, self.auto_arm_offboard)

    def setup_logger(self):
        if not self.log_enabled:
            return
        if not self.log_directory:
            package_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            self.log_directory = os.path.join(package_dir, "data_logs")
        if not self.log_csv:
            stamp = time.strftime("%Y%m%d_%H%M%S")
            self.log_csv = os.path.join(self.log_directory, "whole_body_state_log_%s.csv" % stamp)

        output_dir = os.path.dirname(os.path.abspath(self.log_csv))
        if output_dir and not os.path.isdir(output_dir):
            os.makedirs(output_dir)
        self.log_file = open(self.log_csv, "w", newline="")
        self.log_writer = csv.writer(self.log_file)
        self.log_writer.writerow([
            "sim_time",
            "elapsed_time",
            "target_x",
            "target_y",
            "target_z",
            "target_roll",
            "target_pitch",
            "target_yaw",
            "uav_x",
            "uav_y",
            "uav_z",
            "uav_qx",
            "uav_qy",
            "uav_qz",
            "uav_qw",
            "uav_roll",
            "uav_pitch",
            "uav_yaw",
            "uav_target_error_x",
            "uav_target_error_y",
            "uav_target_error_z",
            "uav_target_pos_error",
            "uav_target_att_error_x",
            "uav_target_att_error_y",
            "uav_target_att_error_z",
            "uav_target_att_error_norm",
            "uav_desired_x",
            "uav_desired_y",
            "uav_desired_z",
            "uav_desired_error_x",
            "uav_desired_error_y",
            "uav_desired_error_z",
            "uav_desired_pos_error",
            "ee_x",
            "ee_y",
            "ee_z",
            "ee_qx",
            "ee_qy",
            "ee_qz",
            "ee_qw",
            "ee_roll",
            "ee_pitch",
            "ee_yaw",
            "ee_error_x",
            "ee_error_y",
            "ee_error_z",
            "ee_pos_error",
            "ee_att_error_x",
            "ee_att_error_y",
            "ee_att_error_z",
            "ee_att_error_norm",
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "wrist_roll",
        ])
        rospy.loginfo("whole-body 状态日志: %s", self.log_csv)

    def mavros_state_callback(self, msg):
        with self.lock:
            self.mavros_state = msg

    def uav_pose_callback(self, msg):
        p, rot = pose_to_arrays(msg)
        with self.lock:
            self.current_uav_p = p
            self.current_uav_R = rot

    def target_pose_callback(self, msg):
        p, rot = pose_to_arrays(msg)
        with self.lock:
            self.target_p = p
            self.target_R = rot
            self.target_stamp = msg.header.stamp

    def joint_state_callback(self, msg):
        with self.lock:
            for index, name in enumerate(msg.name):
                if index < len(msg.position):
                    self.joint_positions[name] = msg.position[index]

    def get_state_snapshot(self):
        with self.lock:
            if self.current_uav_p is None or self.current_uav_R is None:
                return None
            if not np.all(np.isfinite(self.current_uav_p)) or not np.all(np.isfinite(self.current_uav_R)):
                rospy.logerr_throttle(1.0, "UAV local_position 含 NaN/Inf，暂停 whole-body 控制输出")
                return None
            missing = [name for name in ARM_JOINTS + [WRIST_ROLL_JOINT]
                       if name not in self.joint_positions]
            if missing:
                rospy.logwarn_throttle(2.0, "等待 joint_states 中的关节: %s", ", ".join(missing))
                return None
            q = np.array([self.joint_positions[name] for name in ARM_JOINTS + [WRIST_ROLL_JOINT]], dtype=float)
            if not np.all(np.isfinite(q)):
                rospy.logerr_throttle(1.0, "joint_states 含 NaN/Inf，暂停 whole-body 控制输出")
                return None
            return (
                self.current_uav_p.copy(),
                self.current_uav_R.copy(),
                q,
                self.target_p.copy(),
                self.target_R.copy(),
            )

    def arm_fk_in_uav_base(self, q):
        mat = transform_matrix(self.arm_mount_xyz, self.arm_mount_rpy)
        joint_specs = [
            ([0.0005, 0.0, 0.0535], [1.2297e-10, 2.2825e-27, -3.4694e-17], [0, 0, 1], q[0]),
            ([-0.0105, 0.0226, 0.032], [-5.7183e-11, 0.785, 0.0], [0, -1, 0], q[1]),
            ([0.0, -0.0335, 0.104], [-1.6e-10, 0.785, 0.0], [0, -1, 0], q[2]),
            ([-0.0007415, 0.0054, 0.088419], [5.3884e-11, 0.0, 0.0], [0, -1, 0], q[3]),
            ([0.0, 0.009, 0.059], [0.0, 0.0, 0.0], [0, 0, 1], q[4]),
        ]
        for xyz, rpy, axis, qi in joint_specs:
            mat = mat.dot(joint_transform(xyz, rpy, axis, qi))

        center = np.ones(4)
        center[:3] = self.gripper_center
        p = mat.dot(center)[:3]
        return p, mat[:3, :3]

    def task_state(self, uav_p, uav_R, q, target_p, target_R):
        p_local, R_local = self.arm_fk_in_uav_base(q)
        p_world = uav_p + uav_R.dot(p_local)
        R_world = uav_R.dot(R_local)
        e_p = p_world - target_p
        e_R = rot_log(target_R.T.dot(R_world))
        return p_world, R_world, np.concatenate([e_p, e_R])

    def variable_to_state(self, base_uav_p, base_uav_R, base_q, var):
        uav_p = base_uav_p.copy()
        uav_R = base_uav_R.copy()
        q = base_q.copy()
        cursor = 0
        if self.control_uav:
            uav_p = uav_p + var[cursor:cursor + 3]
            cursor += 3
            uav_R = rot_z(var[cursor]).dot(uav_R)
            cursor += 1
        if self.control_arm:
            q[:4] = q[:4] + var[cursor:cursor + 4]
            cursor += 4
        if self.control_wrist_roll:
            q[4] = q[4] + var[cursor]
        return uav_p, uav_R, q

    def build_jacobian(self, uav_p, uav_R, q, target_p, target_R):
        variable_count = 0
        if self.control_uav:
            variable_count += 4
        if self.control_arm:
            variable_count += 4
        if self.control_wrist_roll:
            variable_count += 1

        if variable_count == 0:
            return np.zeros((6, 0))

        _, _, base_error = self.task_state(uav_p, uav_R, q, target_p, target_R)
        jacobian = np.zeros((6, variable_count))
        for column in range(variable_count):
            delta = np.zeros(variable_count)
            delta[column] = self.jacobian_eps
            p2, R2, q2 = self.variable_to_state(uav_p, uav_R, q, delta)
            _, _, error2 = self.task_state(p2, R2, q2, target_p, target_R)
            jacobian[:, column] = (error2 - base_error) / self.jacobian_eps
        return jacobian

    def solve_velocity(self, jacobian, task_velocity):
        if jacobian.shape[1] == 0:
            return np.zeros(0)
        if not np.all(np.isfinite(jacobian)) or not np.all(np.isfinite(task_velocity)):
            rospy.logerr_throttle(1.0, "Jacobian 或任务速度含 NaN/Inf，输出零速度")
            return np.zeros(jacobian.shape[1])
        lhs = jacobian.T.dot(jacobian) + (self.damping ** 2) * np.eye(jacobian.shape[1])
        rhs = jacobian.T.dot(task_velocity)
        command = np.linalg.solve(lhs, rhs)
        if not np.all(np.isfinite(command)):
            rospy.logerr_throttle(1.0, "whole-body 求解结果含 NaN/Inf，输出零速度")
            return np.zeros(jacobian.shape[1])
        return command

    def split_and_limit_command(self, command, q, dt):
        cursor = 0
        uav_vel = np.zeros(3)
        uav_yaw_rate = 0.0
        arm_vel = np.zeros(4)
        wrist_roll_vel = 0.0

        if self.control_uav:
            uav_vel = command[cursor:cursor + 3]
            cursor += 3
            speed = np.linalg.norm(uav_vel)
            if speed > self.max_uav_vel:
                uav_vel *= self.max_uav_vel / speed
            uav_yaw_rate = max(min(command[cursor], self.max_uav_yaw_rate), -self.max_uav_yaw_rate)
            cursor += 1

        if self.control_arm:
            arm_vel = np.clip(command[cursor:cursor + 4], -self.max_arm_vel, self.max_arm_vel)
            for i in range(4):
                if q[i] <= self.arm_joint_limits[i, 0] and arm_vel[i] < 0.0:
                    arm_vel[i] = 0.0
                if q[i] >= self.arm_joint_limits[i, 1] and arm_vel[i] > 0.0:
                    arm_vel[i] = 0.0
            cursor += 4

        if self.control_wrist_roll:
            wrist_roll_vel = max(min(command[cursor], self.max_wrist_roll_vel), -self.max_wrist_roll_vel)
            if q[4] <= self.wrist_roll_min and wrist_roll_vel < 0.0:
                wrist_roll_vel = 0.0
            if q[4] >= self.wrist_roll_max and wrist_roll_vel > 0.0:
                wrist_roll_vel = 0.0

        if self.integrated_wrist_roll_cmd is None:
            self.integrated_wrist_roll_cmd = q[4]
        self.integrated_wrist_roll_cmd = max(
            min(self.integrated_wrist_roll_cmd + wrist_roll_vel * dt, self.wrist_roll_max),
            self.wrist_roll_min,
        )

        return uav_vel, uav_yaw_rate, arm_vel, self.integrated_wrist_roll_cmd

    def publish_uav_command(self, vel, yaw_rate):
        if not np.all(np.isfinite(vel)) or not math.isfinite(yaw_rate):
            rospy.logerr_throttle(1.0, "拒绝发布 NaN/Inf UAV setpoint，改发零速度")
            vel = np.zeros(3)
            yaw_rate = 0.0
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
            PositionTarget.IGNORE_YAW
        )
        msg.velocity.x = vel[0]
        msg.velocity.y = vel[1]
        msg.velocity.z = vel[2]
        msg.yaw_rate = yaw_rate
        self.uav_pub.publish(msg)

    def publish_debug(self, p_world, R_world, error):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = p_world[0]
        pose.pose.position.y = p_world[1]
        pose.pose.position.z = p_world[2]
        quat = rot_to_quaternion(R_world)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        self.current_pose_pub.publish(pose)

        err = Float64MultiArray()
        err.data = error.tolist()
        self.error_pub.publish(err)

    def write_log_row(self, now, uav_p, uav_R, q, target_p, target_R, ee_p, ee_R, error):
        if self.log_writer is None:
            return
        min_period = 1.0 / self.log_rate_hz if self.log_rate_hz > 1e-6 else 0.0
        if self.log_rows > 0 and (now - self.last_log_time).to_sec() < min_period:
            return
        if self.log_start_time is None:
            self.log_start_time = now
        self.last_log_time = now

        p_local, _ = self.arm_fk_in_uav_base(q)
        desired_uav_p = target_p - uav_R.dot(p_local)
        uav_desired_error = uav_p - desired_uav_p
        uav_target_error = uav_p - target_p
        uav_att_error = rot_log(target_R.T.dot(uav_R))
        uav_quat = rot_to_quaternion(uav_R)
        uav_rpy = rot_to_rpy(uav_R)
        target_rpy = rot_to_rpy(target_R)
        ee_quat = rot_to_quaternion(ee_R)
        ee_rpy = rot_to_rpy(ee_R)

        self.log_writer.writerow([
            "%.6f" % now.to_sec(),
            "%.6f" % (now - self.log_start_time).to_sec(),
            "%.6f" % target_p[0],
            "%.6f" % target_p[1],
            "%.6f" % target_p[2],
            "%.6f" % target_rpy[0],
            "%.6f" % target_rpy[1],
            "%.6f" % target_rpy[2],
            "%.6f" % uav_p[0],
            "%.6f" % uav_p[1],
            "%.6f" % uav_p[2],
            "%.9f" % uav_quat[0],
            "%.9f" % uav_quat[1],
            "%.9f" % uav_quat[2],
            "%.9f" % uav_quat[3],
            "%.6f" % uav_rpy[0],
            "%.6f" % uav_rpy[1],
            "%.6f" % uav_rpy[2],
            "%.6f" % uav_target_error[0],
            "%.6f" % uav_target_error[1],
            "%.6f" % uav_target_error[2],
            "%.6f" % np.linalg.norm(uav_target_error),
            "%.6f" % uav_att_error[0],
            "%.6f" % uav_att_error[1],
            "%.6f" % uav_att_error[2],
            "%.6f" % np.linalg.norm(uav_att_error),
            "%.6f" % desired_uav_p[0],
            "%.6f" % desired_uav_p[1],
            "%.6f" % desired_uav_p[2],
            "%.6f" % uav_desired_error[0],
            "%.6f" % uav_desired_error[1],
            "%.6f" % uav_desired_error[2],
            "%.6f" % np.linalg.norm(uav_desired_error),
            "%.6f" % ee_p[0],
            "%.6f" % ee_p[1],
            "%.6f" % ee_p[2],
            "%.9f" % ee_quat[0],
            "%.9f" % ee_quat[1],
            "%.9f" % ee_quat[2],
            "%.9f" % ee_quat[3],
            "%.6f" % ee_rpy[0],
            "%.6f" % ee_rpy[1],
            "%.6f" % ee_rpy[2],
            "%.6f" % error[0],
            "%.6f" % error[1],
            "%.6f" % error[2],
            "%.6f" % np.linalg.norm(error[:3]),
            "%.6f" % error[3],
            "%.6f" % error[4],
            "%.6f" % error[5],
            "%.6f" % np.linalg.norm(error[3:]),
            "%.6f" % q[0],
            "%.6f" % q[1],
            "%.6f" % q[2],
            "%.6f" % q[3],
            "%.6f" % q[4],
        ])
        self.log_rows += 1
        if self.log_rows % 50 == 0:
            self.log_file.flush()

    def close_logger(self):
        if self.log_file is not None:
            self.log_file.flush()
            self.log_file.close()
            self.log_file = None
            rospy.loginfo("whole-body 状态日志已保存: rows=%d output=%s", self.log_rows, self.log_csv)

    def wait_for_mavros_services(self):
        try:
            rospy.wait_for_service(self.vehicle_ns + "/mavros/cmd/arming", timeout=30)
            rospy.wait_for_service(self.vehicle_ns + "/mavros/set_mode", timeout=30)
            return True
        except rospy.ROSException as exc:
            rospy.logerr("等待 MAVROS 服务超时: %s", exc)
            return False

    def wait_for_startup_inputs(self, timeout=30.0):
        deadline = rospy.Time.now() + rospy.Duration(timeout)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            snapshot = self.get_state_snapshot()
            with self.lock:
                connected = self.mavros_state.connected
            if snapshot is not None and connected:
                return True
            rate.sleep()

        with self.lock:
            state = self.mavros_state
        rospy.logerr("等待启动输入超时: connected=%s armed=%s mode=%s",
                     state.connected, state.armed, state.mode)
        return False

    def set_offboard_mode(self):
        for attempt in range(1, self.mode_attempts + 1):
            with self.lock:
                if self.mavros_state.mode == "OFFBOARD":
                    rospy.loginfo("飞行模式已是 OFFBOARD")
                    return True
            try:
                res = self.set_mode_service(base_mode=0, custom_mode="OFFBOARD")
            except rospy.ServiceException as exc:
                rospy.logwarn("切换 OFFBOARD 服务调用失败: %s", exc)
                res = None

            with self.lock:
                reached = self.mavros_state.mode == "OFFBOARD"
            if reached or (res is not None and res.mode_sent):
                rospy.loginfo("飞行模式已切换为 OFFBOARD")
                return True
            rospy.logwarn("切换 OFFBOARD 失败，第 %d 次重试", attempt)
            rospy.sleep(1.0)
        return False

    def arm_vehicle(self):
        for attempt in range(1, self.arm_attempts + 1):
            with self.lock:
                if self.mavros_state.armed:
                    rospy.loginfo("飞机已解锁")
                    return True
            try:
                res = self.arm_service(True)
            except rospy.ServiceException as exc:
                rospy.logwarn("解锁服务调用失败: %s", exc)
                res = None

            if res is not None and res.success:
                rospy.loginfo("飞机已解锁")
                return True
            rospy.logwarn("飞机解锁失败 result=%s，第 %d 次重试", getattr(res, "result", None), attempt)
            rospy.sleep(2.0)
        return False

    def enter_offboard_and_arm(self):
        if not self.control_uav:
            rospy.logwarn("control_uav=false，跳过 OFFBOARD/解锁")
            return True
        if not self.wait_for_mavros_services():
            return False
        if not self.wait_for_startup_inputs():
            return False

        rospy.loginfo("预发布 OFFBOARD setpoint %.1f 秒", self.setpoint_warmup_time)
        end_time = rospy.Time.now() + rospy.Duration(self.setpoint_warmup_time)
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.publish_uav_command(np.zeros(3), 0.0)
            rate.sleep()

        if not self.arm_vehicle():
            rospy.logerr("飞机解锁失败，whole-body 控制退出")
            return False
        if not self.set_offboard_mode():
            rospy.logerr("切换 OFFBOARD 失败，whole-body 控制退出")
            return False
        return True

    def spin(self):
        if self.auto_arm_offboard and not self.enter_offboard_and_arm():
            self.close_logger()
            return

        rate = rospy.Rate(self.rate_hz)
        dt = 1.0 / self.rate_hz
        try:
            while not rospy.is_shutdown():
                snapshot = self.get_state_snapshot()
                if snapshot is None:
                    rate.sleep()
                    continue

                now = rospy.Time.now()
                uav_p, uav_R, q, target_p, target_R = snapshot
                p_world, R_world, error = self.task_state(uav_p, uav_R, q, target_p, target_R)
                jacobian = self.build_jacobian(uav_p, uav_R, q, target_p, target_R)
                task_velocity = -np.array([
                    self.kp_pos * error[0],
                    self.kp_pos * error[1],
                    self.kp_pos * error[2],
                    self.kp_rot * error[3],
                    self.kp_rot * error[4],
                    self.kp_rot * error[5],
                ])
                raw_command = self.solve_velocity(jacobian, task_velocity)
                uav_vel, uav_yaw_rate, arm_vel, wrist_roll_cmd = self.split_and_limit_command(raw_command, q, dt)

                if self.control_uav:
                    self.publish_uav_command(uav_vel, uav_yaw_rate)
                if self.control_arm:
                    arm_msg = Float64MultiArray()
                    arm_msg.data = arm_vel.tolist()
                    self.arm_pub.publish(arm_msg)
                if self.control_wrist_roll:
                    wrist_msg = Float64()
                    wrist_msg.data = wrist_roll_cmd
                    self.wrist_pub.publish(wrist_msg)

                self.publish_debug(p_world, R_world, error)
                self.write_log_row(now, uav_p, uav_R, q, target_p, target_R, p_world, R_world, error)
                rospy.loginfo_throttle(
                    1.0,
                    "whole-body error pos=%.4f rot=%.4f",
                    np.linalg.norm(error[:3]),
                    np.linalg.norm(error[3:]),
                )
                rate.sleep()
        finally:
            self.close_logger()


def main():
    rospy.init_node("whole_body_pose_control")
    controller = WholeBodyPoseController()
    controller.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
