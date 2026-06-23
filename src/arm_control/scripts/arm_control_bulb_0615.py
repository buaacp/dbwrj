#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import ast
import math
import os
import sys
import threading
import time

import casadi as ca
import numpy as np
import rospy
import serial
from geometry_msgs.msg import PoseStamped, TwistStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, Float64MultiArray, Int32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

current_dir = os.path.dirname(__file__)
utils_path = os.path.join(current_dir, "utils")
sys.path.append(utils_path)

from utils import ARM, MPC, UAV, UAV_ARM  # noqa: E402


STATE_NAMES = {
    0: "IDLE",
    1: "PICK_ALIGN",
    2: "GRASP_BULB",
    3: "UNSCREW_BULB",
    4: "TRANSFER_TO_SOCKET",
    5: "FINE_ALIGN_SOCKET",
    6: "SCREW_IN_BULB",
    7: "FINISH",
    99: "ABORT",
}

mission_state = 0
target_poses = {}


def get_param(name, default):
    private_name = "~" + name
    if rospy.has_param(private_name):
        return rospy.get_param(private_name)
    found_name = rospy.search_param(name)
    if found_name:
        return rospy.get_param(found_name)
    return default


def get_vector_param(name, default, length):
    value = get_param(name, default)
    if isinstance(value, str):
        try:
            value = ast.literal_eval(value)
        except (ValueError, SyntaxError):
            rospy.logwarn("参数 %s=%s 解析失败，使用默认值 %s", name, value, default)
            value = default
    if not isinstance(value, (list, tuple)) or len(value) != length:
        rospy.logwarn("参数 %s 长度不正确，使用默认值 %s", name, default)
        value = default
    return [float(item) for item in value]


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def normalize_vector(vector, fallback):
    vector = np.array(vector, dtype=float).reshape(3, 1)
    norm = np.linalg.norm(vector)
    if norm < 1e-6:
        return np.array(fallback, dtype=float).reshape(3, 1)
    return vector / norm


def finite_array(values):
    return np.all(np.isfinite(np.array(values, dtype=float)))


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


def yaw_to_quaternion(yaw):
    half = 0.5 * yaw
    return 0.0, 0.0, math.sin(half), math.cos(half)


def pose_axis_world(pose_stamped, local_axis):
    return normalize_vector(
        quaternion_to_rotation(pose_stamped.pose.orientation).dot(local_axis),
        [0.0, 0.0, -1.0],
    )


def target_pose_callback(msg, target_name):
    target_poses[target_name] = msg


def mission_state_callback(msg):
    global mission_state
    mission_state = msg.data


def create_serial_port(port_name, baudrate=115200):
    try:
        return serial.Serial(
            port=port_name,
            baudrate=baudrate,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            timeout=0,
        )
    except serial.SerialException:
        rospy.logwarn("无法打开串口 %s", port_name)
        return None


def state_name(state):
    return STATE_NAMES.get(state, "UNKNOWN")


def pose_has_position(pose_stamped):
    if pose_stamped is None:
        return False
    position = pose_stamped.pose.position
    values = (position.x, position.y, position.z)
    return finite_array(values) and any(abs(value) > 1e-6 for value in values)


def arm_velocity_from_mpc(d_q):
    return [-float(d_q[0, 0]), -float(d_q[1, 0]), -float(d_q[2, 0]), -float(d_q[3, 0])]


def zero_arm_control(servo_ids):
    return [0.0 for _ in servo_ids]


def stow_arm_control(arm, servo_ids, stow_joint_rad, stow_kp, stow_max_velocity):
    angular = []
    for servo_id in servo_ids:
        angle_now = arm.angle.get(servo_id)
        if angle_now is None:
            rospy.logwarn_throttle(2.0, "收起机械臂等待关节 %d 状态", servo_id)
            angular.append(0.0)
            continue
        velocity = stow_kp * (stow_joint_rad[servo_id] - angle_now)
        velocity = max(min(velocity, stow_max_velocity), -stow_max_velocity)
        angular.append(-velocity)
    return angular


def publish_gripper(pub, position, duration=0.4):
    msg = JointTrajectory()
    msg.joint_names = ["left_knuckle_joint"]
    point = JointTrajectoryPoint()
    point.positions = [float(position)]
    point.time_from_start = rospy.Duration(duration)
    msg.points.append(point)
    pub.publish(msg)


def target_reached(pos_error, axis_error, pos_tolerance, axis_tolerance_deg, line_error=None, line_tolerance=None):
    if pos_error is None or axis_error is None:
        return False
    if not (pos_error < pos_tolerance and axis_error < axis_tolerance_deg):
        return False
    if line_tolerance is not None:
        return line_error is not None and line_error < line_tolerance
    return True


def display_error(value):
    return float(value) if value is not None else float("nan")


def stable_for(start_time, duration):
    return start_time is not None and (rospy.Time.now() - start_time).to_sec() >= duration


class BulbArmController:
    def __init__(self, arm, servo_ids):
        self.arm = arm
        self.servo_ids = servo_ids
        self.uav = UAV.Myuav()
        self.mpc = MPC.MPC()
        self.uav_arm = UAV_ARM.UAV_ARM()
        self.mpc.L1 = self.uav_arm.L1
        self.mpc.L2 = self.uav_arm.L2
        self.mpc.L3 = self.uav_arm.L3
        self.mpc.step_horizon = self.uav_arm.dt
        self.mpc.u0 = ca.DM.zeros((self.mpc.n_controls, self.mpc.N))
        self.wrist_roll_cmd = 0.0
        self.last_target_local = None
        self.last_workspace_correction = np.zeros((3, 1))

    def ready(self):
        return all(self.arm.angle.get(servo_id) is not None for servo_id in self.servo_ids)

    def update_uav_arm_state(self):
        self.uav_arm.p_b = np.array([
            [self.uav.vision_pose.x],
            [self.uav.vision_pose.y],
            [self.uav.vision_pose.z],
        ], dtype=np.float64)
        self.uav_arm.phi = self.uav.roll * math.pi / 180
        self.uav_arm.theta = self.uav.pitch * math.pi / 180
        self.uav_arm.delta = self.uav.yaw * math.pi / 180
        self.uav_arm.d_phi = self.uav.angular.y * math.pi / 180
        self.uav_arm.d_theta = self.uav.angular.x * math.pi / 180
        self.uav_arm.d_delta = self.uav.angular.z * math.pi / 180
        self.uav_arm.d_xb = np.array([
            [self.uav.velocity.x],
            [self.uav.velocity.y],
            [self.uav.velocity.z],
            [self.uav_arm.d_theta],
            [self.uav_arm.d_phi],
            [self.uav_arm.d_delta],
        ])
        self.uav_arm.q = np.array([
            self.arm.angle[0],
            self.arm.angle[1],
            self.arm.angle[2],
            self.arm.angle[3],
        ]).reshape(-1, 1)

    def current_gripper_pose(self):
        q0, q1, q2, q3 = [float(v) for v in self.uav_arm.q.flatten()]
        total = q1 + q2 + q3
        reach = (
            self.uav_arm.L1 * math.sin(q1)
            + self.uav_arm.L2 * math.sin(q1 + q2)
            + self.uav_arm.L3 * math.sin(total)
        )
        p_eb = np.array([
            [-math.sin(q0) * reach],
            [math.cos(q0) * reach],
            [-(self.uav_arm.L1 * math.cos(q1) + self.uav_arm.L2 * math.cos(q1 + q2) + self.uav_arm.L3 * math.cos(total))],
        ])
        axis_local = np.array([
            [-math.sin(q0) * math.sin(total)],
            [math.cos(q0) * math.sin(total)],
            [-math.cos(total)],
        ])
        axis_local = normalize_vector(axis_local, [0.0, 0.0, -1.0])
        R_z = np.array([
            [math.cos(self.uav_arm.delta), -math.sin(self.uav_arm.delta), 0],
            [math.sin(self.uav_arm.delta), math.cos(self.uav_arm.delta), 0],
            [0, 0, 1],
        ])
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(self.uav_arm.theta), -math.sin(self.uav_arm.theta)],
            [0, math.sin(self.uav_arm.theta), math.cos(self.uav_arm.theta)],
        ])
        R_y = np.array([
            [math.cos(self.uav_arm.phi), 0, math.sin(self.uav_arm.phi)],
            [0, 1, 0],
            [-math.sin(self.uav_arm.phi), 0, math.cos(self.uav_arm.phi)],
        ])
        R_b = R_x.dot(R_y).dot(R_z)
        p_e = self.uav_arm.p_b + R_b.dot(self.uav_arm.p_delta + p_eb)
        axis_world = normalize_vector(R_b.dot(axis_local), [0.0, 0.0, -1.0])
        return p_e, axis_world

    def body_rotation(self):
        R_z = np.array([
            [math.cos(self.uav_arm.delta), -math.sin(self.uav_arm.delta), 0],
            [math.sin(self.uav_arm.delta), math.cos(self.uav_arm.delta), 0],
            [0, 0, 1],
        ])
        R_x = np.array([
            [1, 0, 0],
            [0, math.cos(self.uav_arm.theta), -math.sin(self.uav_arm.theta)],
            [0, math.sin(self.uav_arm.theta), math.cos(self.uav_arm.theta)],
        ])
        R_y = np.array([
            [math.cos(self.uav_arm.phi), 0, math.sin(self.uav_arm.phi)],
            [0, 1, 0],
            [-math.sin(self.uav_arm.phi), 0, math.cos(self.uav_arm.phi)],
        ])
        return R_x.dot(R_y).dot(R_z)

    def update_workspace_assist(self, target_position, desired_local, gain, max_norm, z_scale):
        target = np.array(target_position, dtype=float).reshape(3, 1)
        desired = np.array(desired_local, dtype=float).reshape(3, 1)
        R_b = self.body_rotation()
        target_local = np.linalg.inv(R_b).dot(target - self.uav_arm.p_b) - self.uav_arm.p_delta
        local_error = target_local - desired
        correction_world = R_b.dot(local_error) * float(gain)
        correction_world[2, 0] *= float(z_scale)
        norm = float(np.linalg.norm(correction_world))
        if norm > float(max_norm) > 1e-6:
            correction_world *= float(max_norm) / norm
        self.last_target_local = target_local
        self.last_workspace_correction = correction_world

    def track_world_target(
            self,
            target_position,
            target_axis_world,
            workspace_desired_local=None,
            workspace_gain=0.0,
            workspace_max_correction=0.0,
            workspace_z_scale=0.0):
        if not finite_array(target_position) or not finite_array(target_axis_world):
            rospy.logerr_throttle(1.0, "MPC 目标含 NaN/Inf，机械臂停止")
            return zero_arm_control(self.servo_ids), None, None, None
        if not self.ready():
            rospy.logwarn_throttle(2.0, "MPC 等待机械臂关节状态")
            return zero_arm_control(self.servo_ids), None, None, None

        self.update_uav_arm_state()
        if not finite_array(self.uav_arm.p_b) or not finite_array(self.uav_arm.q):
            rospy.logerr_throttle(1.0, "MPC 当前状态含 NaN/Inf，机械臂停止")
            return zero_arm_control(self.servo_ids), None, None, None
        if workspace_desired_local is not None:
            self.update_workspace_assist(
                target_position,
                workspace_desired_local,
                workspace_gain,
                workspace_max_correction,
                workspace_z_scale,
            )
        target_axis_world = normalize_vector(target_axis_world, [0.0, 0.0, -1.0])
        self.uav_arm.pos_target = np.array(target_position, dtype=float).reshape(3, 1)
        self.uav_arm.use_grasp_axis = True
        self.uav_arm.grasp_axis_world = target_axis_world
        self.mpc.state_now = ca.DM([
            self.uav_arm.q[0],
            self.uav_arm.q[1],
            self.uav_arm.q[2],
            self.uav_arm.q[3],
        ])
        x0 = ca.repmat(self.mpc.state_now, 1, self.mpc.N + 1)
        arg_p_arm = self.mpc.get_target_path(self.uav_arm)
        p = ca.vertcat(self.mpc.state_now, self.mpc.state_target, arg_p_arm)
        self.mpc.set_reference(p)
        self.mpc.set_x0(x0, self.mpc.u0)
        _, self.mpc.u0 = self.mpc.get_states_and_control()
        p_e, axis_world = self.current_gripper_pose()
        position_error = p_e - self.uav_arm.pos_target
        pos_error = float(np.linalg.norm(position_error))
        line_error = float(np.linalg.norm(np.cross(position_error.flatten(), target_axis_world.flatten())))
        axis_dot = float(np.clip(axis_world.flatten().dot(target_axis_world.flatten()), -1.0, 1.0))
        axis_error = math.degrees(math.acos(axis_dot))
        rospy.loginfo_throttle(
            1.0,
            "机械臂 MPC 误差: pos=%.4f m axis=%.2f deg centerline=%.4f m",
            pos_error,
            axis_error,
            line_error,
        )
        return arm_velocity_from_mpc(self.mpc.u0[:, 0]), pos_error, axis_error, line_error


class KinematicBulbInteraction:
    def __init__(self, enabled, model_name, bulb_center_axis_offset, socket_center_axis_offset):
        self.enabled = enabled
        self.model_name = model_name
        self.bulb_center_axis_offset = bulb_center_axis_offset
        self.socket_center_axis_offset = socket_center_axis_offset
        self.attached = False
        self.roll_yaw = 0.0
        self.service = None
        if self.enabled:
            try:
                rospy.wait_for_service("/gazebo/set_model_state", timeout=10.0)
                self.service = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
            except rospy.ROSException as exc:
                rospy.logerr("等待 /gazebo/set_model_state 超时，灯泡交互关闭: %s", exc)
                self.enabled = False

    def attach(self):
        if self.enabled and not self.attached:
            rospy.loginfo("灯泡交互: attached，灯泡开始跟随爪心")
        self.attached = self.enabled

    def release_to_socket(self, socket_pose, socket_axis):
        if not self.enabled or socket_pose is None or socket_axis is None:
            self.attached = False
            return False
        base = socket_pose.pose.position
        socket_center = (
            np.array([[base.x], [base.y], [base.z]], dtype=float)
            + socket_axis * float(self.socket_center_axis_offset)
        )
        target = socket_center - socket_axis * float(self.bulb_center_axis_offset)
        self._set_model_pose(target, self.roll_yaw)
        self.attached = False
        rospy.loginfo("灯泡交互: released，灯泡放置到灯座插入位")
        return True

    def follow_gripper(self, gripper_position, axis_world):
        if not self.enabled or not self.attached:
            return
        if not finite_array(gripper_position) or not finite_array(axis_world):
            return
        bulb_origin = (
            np.array(gripper_position, dtype=float).reshape(3, 1)
            - axis_world * float(self.bulb_center_axis_offset)
        )
        self._set_model_pose(bulb_origin, self.roll_yaw)

    def set_roll(self, roll_yaw):
        if self.enabled and self.attached:
            self.roll_yaw = float(roll_yaw)
            rospy.loginfo_throttle(1.0, "灯泡交互: follow_wrist_roll=%.2f rad", self.roll_yaw)

    def _set_model_pose(self, position, yaw):
        if self.service is None or not finite_array(position):
            return
        msg = ModelState()
        msg.model_name = self.model_name
        msg.reference_frame = "world"
        msg.pose.position.x = float(position[0, 0])
        msg.pose.position.y = float(position[1, 0])
        msg.pose.position.z = float(position[2, 0])
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        msg.twist.linear.x = 0.0
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = 0.0
        try:
            self.service(msg)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(1.0, "设置灯泡模型状态失败: %s", exc)


def query_state_continuously(arm, controller, topics):
    rospy.Subscriber("/joint_states", JointState, arm.joint_states_callback)
    rospy.Subscriber(topics["mission"], Int32, mission_state_callback)
    rospy.Subscriber(topics["bulb"], PoseStamped, target_pose_callback, callback_args="bulb")
    rospy.Subscriber(topics["socket"], PoseStamped, target_pose_callback, callback_args="socket")
    rospy.Subscriber("mavros/vision_pose/pose", PoseStamped, controller.uav.vision_imu_callback)
    rospy.Subscriber("mavros/vision_pose/pose", PoseStamped, controller.uav.vision_pose_callback)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, controller.uav.pose_callback)
    rospy.Subscriber("mavros/local_position/velocity_local", TwistStamped, controller.uav.velocity_callback)
    rospy.Subscriber("/clock", Clock, controller.uav_arm.clock_callback)
    rospy.spin()


def task_target(target_name, axis_local, offset_along_axis):
    pose_stamped = target_poses.get(target_name)
    if not pose_has_position(pose_stamped):
        return None, None
    axis_world = pose_axis_world(pose_stamped, axis_local)
    base = pose_stamped.pose.position
    target = np.array([[base.x], [base.y], [base.z]]) + axis_world * float(offset_along_axis)
    return target, axis_world


def task_center_target(target_name, axis_local, center_axis_offset, offset_along_axis=0.0):
    pose_stamped = target_poses.get(target_name)
    if not pose_has_position(pose_stamped):
        return None, None
    axis_world = pose_axis_world(pose_stamped, axis_local)
    target = (
        pose_position_array(pose_stamped)
        + axis_world * float(center_axis_offset)
        + axis_world * float(offset_along_axis)
    )
    return target, axis_world


def bulb_axis_target_from_center(bulb_pose, axis_local, center_axis_offset, offset_along_axis=0.0):
    if not pose_has_position(bulb_pose):
        return None, None, None
    axis_world = pose_axis_world(bulb_pose, axis_local)
    center = pose_position_array(bulb_pose) + axis_world * float(center_axis_offset)
    target = center + axis_world * float(offset_along_axis)
    return target, axis_world, center


def track_bulb_center(
        controller,
        bulb_axis_local,
        bulb_center_axis_offset,
        workspace_desired_local,
        workspace_assist_gain,
        workspace_assist_max_correction,
        workspace_assist_z_scale):
    bulb_pose = target_poses.get("bulb")
    target, axis, center = bulb_axis_target_from_center(
        bulb_pose,
        bulb_axis_local,
        bulb_center_axis_offset,
        0.0,
    )
    if target is None:
        rospy.logwarn_throttle(1.0, "等待灯泡球心位姿")
        return None, None, None, None, None, None, None

    angular, pos_error, axis_error, line_error = controller.track_world_target(
        target,
        axis,
        workspace_desired_local,
        workspace_assist_gain,
        workspace_assist_max_correction,
        workspace_assist_z_scale,
    )
    gripper_position = None
    gripper_axis = None
    if controller.ready():
        gripper_position, gripper_axis = controller.current_gripper_pose()
    return angular, pos_error, axis_error, line_error, gripper_position, gripper_axis, center


def pose_position_array(pose_stamped):
    position = pose_stamped.pose.position
    return np.array([[position.x], [position.y], [position.z]], dtype=float)


def project_point_to_axis(point, axis_point, axis_direction):
    axis = normalize_vector(axis_direction, [0.0, 0.0, -1.0])
    point = np.array(point, dtype=float).reshape(3, 1)
    axis_point = np.array(axis_point, dtype=float).reshape(3, 1)
    return axis_point + axis * float(axis.T.dot(point - axis_point))


def interpolate_axis_target(target_name, axis_local, start_offset, end_offset, alpha):
    alpha = max(0.0, min(1.0, float(alpha)))
    offset = (1.0 - alpha) * float(start_offset) + alpha * float(end_offset)
    target, axis = task_target(target_name, axis_local, offset)
    return target, axis, offset


def main():
    global mission_state

    rospy.init_node("arm_control_bulb_0615", anonymous=True)

    servo_port_name = get_param("servo_port_name", "/dev/ttyUSB0")
    servo_baudrate = int(get_param("servo_baudrate", 115200))
    servo_ids = [int(item) for item in get_vector_param("servo_ids", [0, 1, 2, 3], 4)]
    if_simulation = rospy.get_param("/if_simulation", False)
    mission_topic = get_param("mission_topic", "/mission_state")
    bulb_target_topic = get_param("bulb_target_topic", "/light_bulb/body_pose")
    socket_target_topic = get_param("socket_target_topic", "/light_bulb_fixture/socket_pose")
    stow_joint_rad = np.deg2rad(get_vector_param("stow_joint_deg", [-45.0, 90.0, 45.0, 0.0], 4))
    stow_kp = float(get_param("stow_kp", 1.2))
    stow_max_velocity = float(get_param("stow_max_velocity", 0.4))
    bulb_axis_local = normalize_vector(get_vector_param("bulb_axis_local", [0.0, 0.0, -1.0], 3), [0.0, 0.0, -1.0])
    socket_axis_local = normalize_vector(get_vector_param("socket_axis_local", [0.0, 0.0, -1.0], 3), [0.0, 0.0, -1.0])
    bulb_center_axis_offset = float(get_param("bulb_center_axis_offset", -0.07))
    socket_center_axis_offset = float(get_param("socket_center_axis_offset", -0.15))
    upper_align_offset = float(get_param("bulb_upper_axis_offset", get_param("pregrasp_axis_offset", -0.12)))
    grasp_offset = float(get_param("grasp_axis_offset", -0.02))
    socket_preinsert_offset = float(get_param("socket_preinsert_axis_offset", -0.09))
    socket_insert_offset = float(get_param("socket_insert_axis_offset", -0.02))
    gripper_open = float(get_param("gripper_open", 0.55))
    gripper_closed = float(get_param("gripper_closed", 0.12))
    unscrew_turns = float(get_param("unscrew_turns", -1.0))
    screw_turns = float(get_param("screw_turns", 1.0))
    wrist_roll_rate = float(get_param("wrist_roll_rate", 0.8))
    socket_screw_duration = float(get_param("socket_screw_duration", get_param("socket_release_delay", 6.0)))
    align_pos_tolerance = float(get_param("align_pos_tolerance", 0.04))
    align_axis_tolerance_deg = float(get_param("align_axis_tolerance_deg", 3.0))
    centerline_tolerance = float(get_param("centerline_tolerance", 0.015))
    align_ready_duration = float(get_param("align_ready_duration", 0.5))
    grasp_close_duration = float(get_param("grasp_close_duration", 1.0))
    workspace_desired_local = np.array(
        get_vector_param("workspace_desired_local", [0.0, 0.0, -0.22], 3),
        dtype=float,
    ).reshape(3, 1)
    workspace_assist_gain = float(get_param("workspace_assist_gain", 0.35))
    workspace_assist_max_correction = float(get_param("workspace_assist_max_correction", 0.12))
    workspace_assist_z_scale = float(get_param("workspace_assist_z_scale", 0.25))
    enable_wrist_roll = parse_bool(get_param("enable_wrist_roll", True))
    enable_gripper_action = parse_bool(get_param("enable_gripper_action", True))
    enable_bulb_visual_roll = parse_bool(get_param("enable_bulb_visual_roll", False))
    enable_kinematic_bulb_interaction = parse_bool(get_param("enable_kinematic_bulb_interaction", True))
    safe_demo_mode = parse_bool(get_param("safe_demo_mode", False))
    bulb_model_name = str(get_param("bulb_model_name", "light_bulb"))

    if if_simulation:
        uart = None
    else:
        uart = create_serial_port(servo_port_name, servo_baudrate)
        if uart is None and servo_port_name != "/dev/ttyUSB1":
            uart = create_serial_port("/dev/ttyUSB1", servo_baudrate)
        if uart is None:
            rospy.logerr("机械臂串口打开失败，进入 ABORT。")
            mission_state = 99

    arm = ARM.ARM(uart=uart, SERVO_IDS=servo_ids, if_simulation=if_simulation)
    controller = BulbArmController(arm, servo_ids)
    bulb_interaction = KinematicBulbInteraction(
        if_simulation and enable_kinematic_bulb_interaction,
        bulb_model_name,
        bulb_center_axis_offset,
        socket_center_axis_offset,
    )
    topics = {"mission": mission_topic, "bulb": bulb_target_topic, "socket": socket_target_topic}
    state_thread = threading.Thread(target=query_state_continuously, args=(arm, controller, topics))
    state_thread.daemon = True
    state_thread.start()

    pub_angular = rospy.Publisher("/le_arm_controller/command", Float64MultiArray, queue_size=10)
    pub_wrist = rospy.Publisher("/wrist_roll_controller/command", Float64, queue_size=10)
    pub_gripper = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
    pub_ready_state = rospy.Publisher("/arm_control_bulb/ready_state", Int32, queue_size=10, latch=True)
    pub_error = rospy.Publisher("/arm_control_bulb/error", Float64MultiArray, queue_size=10)
    pub_workspace_assist = rospy.Publisher("/arm_control_bulb/workspace_assist", Float64MultiArray, queue_size=10)
    time.sleep(0.5)

    rospy.loginfo("灯泡更换机械臂节点已启动，mission_topic=%s", mission_topic)
    rospy.loginfo("目标话题: bulb=%s socket=%s", bulb_target_topic, socket_target_topic)
    rospy.loginfo("mission_state: 0 IDLE, 1 PICK_ALIGN, 2 GRASP_BULB, 3 UNSCREW_BULB, 4 TRANSFER_TO_SOCKET, 5 FINE_ALIGN_SOCKET, 6 SCREW_IN_BULB, 7 FINISH, 99 ABORT")

    last_state = None
    state_enter_time = rospy.Time.now()
    ready_candidate_since = None
    grasp_descend_start_time = None
    socket_contact_ready_since = None
    socket_screw_start_time = None
    bulb_released_to_socket = False
    interaction_hold_target = None
    interaction_hold_axis = None
    control_dt = 1.0 / 20.0
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        state = mission_state
        if state != last_state:
            rospy.loginfo("机械臂任务状态: %s %s", state, state_name(state))
            state_enter_time = rospy.Time.now()
            ready_candidate_since = None
            grasp_descend_start_time = None
            socket_contact_ready_since = None
            socket_screw_start_time = None
            interaction_hold_target = None
            interaction_hold_axis = None
            if state in (0, 1, 2, 3, 4, 5):
                bulb_released_to_socket = False
            last_state = state

        angular = zero_arm_control(servo_ids)
        pos_error = float("nan")
        axis_error = float("nan")
        line_error = float("nan")
        gripper_position = None
        gripper_axis = None
        ready_state = 0
        elapsed = (rospy.Time.now() - state_enter_time).to_sec()
        open_cmd = gripper_open
        closed_cmd = gripper_closed if enable_gripper_action else gripper_open

        if state == 0:
            angular = stow_arm_control(arm, servo_ids, stow_joint_rad, stow_kp, stow_max_velocity)
            publish_gripper(pub_gripper, open_cmd)
        elif state == 1:
            publish_gripper(pub_gripper, open_cmd)
            if safe_demo_mode:
                angular = zero_arm_control(servo_ids)
                if controller.ready():
                    gripper_position, gripper_axis = controller.current_gripper_pose()
                    pos_error = 0.0
                    axis_error = 0.0
                if ready_candidate_since is None:
                    ready_candidate_since = rospy.Time.now()
                if stable_for(ready_candidate_since, align_ready_duration):
                    ready_state = 1
            else:
                result = track_bulb_center(
                    controller,
                    bulb_axis_local,
                    bulb_center_axis_offset,
                    workspace_desired_local,
                    workspace_assist_gain,
                    workspace_assist_max_correction,
                    workspace_assist_z_scale,
                )
                angular_cmd, pos_error_new, axis_error_new, line_error_new, gripper_position_new, gripper_axis_new, _ = result
                if angular_cmd is not None:
                    rospy.loginfo_throttle(
                        1.0,
                        "PICK_ALIGN 对准灯泡球心",
                    )
                    angular = angular_cmd
                    pos_error = pos_error_new
                    axis_error = axis_error_new
                    line_error = line_error_new
                    gripper_position = gripper_position_new
                    gripper_axis = gripper_axis_new
                    if target_reached(
                            pos_error,
                            axis_error,
                            align_pos_tolerance,
                            align_axis_tolerance_deg,
                            line_error,
                            centerline_tolerance):
                        if ready_candidate_since is None:
                            ready_candidate_since = rospy.Time.now()
                        if stable_for(ready_candidate_since, align_ready_duration):
                            ready_state = 1
                    else:
                        ready_candidate_since = None
        elif state == 2:
            if safe_demo_mode:
                angular = zero_arm_control(servo_ids)
                if controller.ready():
                    gripper_position, gripper_axis = controller.current_gripper_pose()
                    pos_error = 0.0
                    axis_error = 0.0
                if ready_candidate_since is None:
                    ready_candidate_since = rospy.Time.now()
                if stable_for(ready_candidate_since, align_ready_duration):
                    publish_gripper(pub_gripper, closed_cmd)
                else:
                    publish_gripper(pub_gripper, open_cmd)
                if stable_for(ready_candidate_since, align_ready_duration + grasp_close_duration):
                    ready_state = 2
                    bulb_interaction.attach()
            else:
                result = track_bulb_center(
                    controller,
                    bulb_axis_local,
                    bulb_center_axis_offset,
                    workspace_desired_local,
                    workspace_assist_gain,
                    workspace_assist_max_correction,
                    workspace_assist_z_scale,
                )
                angular_cmd, pos_error_new, axis_error_new, line_error_new, gripper_position_new, gripper_axis_new, bulb_center = result
                if angular_cmd is None:
                    publish_gripper(pub_gripper, open_cmd)
                else:
                    if grasp_descend_start_time is None:
                        grasp_descend_start_time = rospy.Time.now()
                        ready_candidate_since = None
                        rospy.loginfo(
                            "GRASP_BULB 直接跟踪灯泡球心并约束灯泡轴线: center=[%.3f %.3f %.3f]",
                            bulb_center[0, 0],
                            bulb_center[1, 0],
                            bulb_center[2, 0],
                        )
                    rospy.loginfo_throttle(
                        1.0,
                        "GRASP_BULB 轴线约束抓取灯泡球心: centerline_tol=%.3f m",
                        centerline_tolerance,
                    )
                    angular = angular_cmd
                    pos_error = pos_error_new
                    axis_error = axis_error_new
                    line_error = line_error_new
                    gripper_position = gripper_position_new
                    gripper_axis = gripper_axis_new
                    if target_reached(
                            pos_error,
                            axis_error,
                            align_pos_tolerance,
                            align_axis_tolerance_deg,
                            line_error,
                            centerline_tolerance):
                        if ready_candidate_since is None:
                            ready_candidate_since = rospy.Time.now()
                            publish_gripper(pub_gripper, closed_cmd)
                        else:
                            publish_gripper(pub_gripper, closed_cmd)
                        if stable_for(
                                ready_candidate_since,
                                grasp_close_duration):
                            ready_state = 2
                            bulb_interaction.attach()
                    else:
                        ready_candidate_since = None
                        publish_gripper(pub_gripper, open_cmd)
        elif state == 3:
            publish_gripper(pub_gripper, closed_cmd)
            if safe_demo_mode:
                angular = zero_arm_control(servo_ids)
                if controller.ready():
                    gripper_position, gripper_axis = controller.current_gripper_pose()
                    pos_error = 0.0
                    axis_error = 0.0
                ready_state = 3
            else:
                if interaction_hold_target is None:
                    if controller.ready():
                        controller.update_uav_arm_state()
                        interaction_hold_target, interaction_hold_axis = controller.current_gripper_pose()
                        rospy.loginfo(
                            "UNSCREW_BULB 锁定机械臂末端保持点: [%.3f %.3f %.3f]",
                            interaction_hold_target[0, 0],
                            interaction_hold_target[1, 0],
                            interaction_hold_target[2, 0],
                        )
                    else:
                        rospy.logwarn_throttle(1.0, "UNSCREW_BULB 等待机械臂关节状态后锁定保持点")
                if interaction_hold_target is not None:
                    angular, pos_error, axis_error, line_error = controller.track_world_target(
                        interaction_hold_target,
                        interaction_hold_axis,
                        workspace_desired_local,
                        workspace_assist_gain,
                        workspace_assist_max_correction,
                        workspace_assist_z_scale,
                    )
                    if controller.ready():
                        gripper_position, gripper_axis = controller.current_gripper_pose()
            if target_reached(
                    pos_error,
                    axis_error,
                    align_pos_tolerance,
                    align_axis_tolerance_deg,
                    line_error,
                    centerline_tolerance):
                ready_state = 3
            if enable_wrist_roll:
                delta_roll = wrist_roll_rate * unscrew_turns * control_dt
                controller.wrist_roll_cmd += delta_roll
                pub_wrist.publish(Float64(data=controller.wrist_roll_cmd))
                bulb_interaction.set_roll(controller.wrist_roll_cmd)
            elif enable_bulb_visual_roll:
                rospy.logwarn_throttle(2.0, "enable_bulb_visual_roll 已禁用为独立转灯泡；请启用 enable_wrist_roll 让爪旋转")
        elif state == 4:
            publish_gripper(pub_gripper, closed_cmd)
            target, axis = task_center_target(
                "socket",
                socket_axis_local,
                socket_center_axis_offset,
                0.0,
            )
            if target is None:
                rospy.logwarn_throttle(1.0, "TRANSFER_TO_SOCKET 等待灯座预插入目标")
            else:
                rospy.loginfo_throttle(
                    1.0,
                    "TRANSFER_TO_SOCKET 跟踪灯座假想灯心轴线点",
                )
                angular, pos_error, axis_error, line_error = controller.track_world_target(
                    target,
                    axis,
                    workspace_desired_local,
                    workspace_assist_gain,
                    workspace_assist_max_correction,
                    workspace_assist_z_scale,
                )
                if controller.ready():
                    gripper_position, gripper_axis = controller.current_gripper_pose()
                if target_reached(
                        pos_error,
                        axis_error,
                        align_pos_tolerance,
                        align_axis_tolerance_deg,
                        line_error,
                        centerline_tolerance):
                    ready_state = 4
        elif state == 5:
            publish_gripper(pub_gripper, closed_cmd)
            if safe_demo_mode:
                angular = zero_arm_control(servo_ids)
                if controller.ready():
                    gripper_position, gripper_axis = controller.current_gripper_pose()
                    pos_error = 0.0
                    axis_error = 0.0
                if ready_candidate_since is None:
                    ready_candidate_since = rospy.Time.now()
                if stable_for(ready_candidate_since, align_ready_duration):
                    ready_state = 5
            else:
                target, axis = task_center_target(
                    "socket",
                    socket_axis_local,
                    socket_center_axis_offset,
                    0.0,
                )
                if target is None:
                    rospy.logwarn_throttle(1.0, "FINE_ALIGN_SOCKET 等待灯座位姿")
                else:
                    angular, pos_error, axis_error, line_error = controller.track_world_target(
                        target,
                        axis,
                        workspace_desired_local,
                        workspace_assist_gain,
                        workspace_assist_max_correction,
                        workspace_assist_z_scale,
                    )
                    if controller.ready():
                        gripper_position, gripper_axis = controller.current_gripper_pose()
                    if target_reached(
                            pos_error,
                            axis_error,
                            align_pos_tolerance,
                            align_axis_tolerance_deg,
                            line_error,
                            centerline_tolerance):
                        if ready_candidate_since is None:
                            ready_candidate_since = rospy.Time.now()
                        if stable_for(ready_candidate_since, align_ready_duration):
                            ready_state = 5
                    else:
                        ready_candidate_since = None
        elif state == 6:
            publish_gripper(pub_gripper, open_cmd if bulb_released_to_socket else closed_cmd)
            if bulb_released_to_socket:
                ready_state = 6
            if safe_demo_mode:
                angular = zero_arm_control(servo_ids)
                if controller.ready():
                    gripper_position, gripper_axis = controller.current_gripper_pose()
                    pos_error = 0.0
                    axis_error = 0.0
                if ready_candidate_since is None:
                    ready_candidate_since = rospy.Time.now()
                if stable_for(ready_candidate_since, align_ready_duration):
                    ready_state = 6
            else:
                if interaction_hold_target is None:
                    if controller.ready():
                        controller.update_uav_arm_state()
                        interaction_hold_target, interaction_hold_axis = controller.current_gripper_pose()
                        socket_screw_start_time = rospy.Time.now()
                        rospy.loginfo(
                            "SCREW_IN_BULB 锁定机械臂末端保持点并开始腕部旋拧: [%.3f %.3f %.3f]",
                            interaction_hold_target[0, 0],
                            interaction_hold_target[1, 0],
                            interaction_hold_target[2, 0],
                        )
                    else:
                        rospy.logwarn_throttle(1.0, "SCREW_IN_BULB 等待机械臂关节状态后锁定保持点")
                if interaction_hold_target is not None:
                    angular, pos_error, axis_error, line_error = controller.track_world_target(
                        interaction_hold_target,
                        interaction_hold_axis,
                        workspace_desired_local,
                        workspace_assist_gain,
                        workspace_assist_max_correction,
                        workspace_assist_z_scale,
                    )
                    if controller.ready():
                        gripper_position, gripper_axis = controller.current_gripper_pose()
                    screw_elapsed = (rospy.Time.now() - socket_screw_start_time).to_sec()
                    rospy.loginfo_throttle(
                        1.0,
                        "SCREW_IN_BULB 保持末端不动并旋拧中: elapsed=%.2f/%.2f",
                        screw_elapsed,
                        socket_screw_duration,
                    )
                    if not bulb_released_to_socket and screw_elapsed >= socket_screw_duration:
                        socket_pose = target_poses.get("socket")
                        socket_axis = pose_axis_world(socket_pose, socket_axis_local) if pose_has_position(socket_pose) else None
                        if bulb_interaction.release_to_socket(socket_pose, socket_axis):
                            bulb_released_to_socket = True
                            publish_gripper(pub_gripper, open_cmd)
                            ready_state = 6
                            rospy.loginfo("SCREW_IN_BULB 旋拧完成，灯泡已与机械臂直接脱离")

            if enable_wrist_roll and not bulb_released_to_socket and socket_screw_start_time is not None:
                delta_roll = wrist_roll_rate * screw_turns * control_dt
                controller.wrist_roll_cmd += delta_roll
                pub_wrist.publish(Float64(data=controller.wrist_roll_cmd))
                bulb_interaction.set_roll(controller.wrist_roll_cmd)
            elif enable_bulb_visual_roll:
                rospy.logwarn_throttle(2.0, "enable_bulb_visual_roll 已禁用为独立转灯泡；请启用 enable_wrist_roll 让爪旋转")
        elif state == 7:
            if bulb_interaction.attached:
                socket_pose = target_poses.get("socket")
                socket_axis = pose_axis_world(socket_pose, socket_axis_local) if pose_has_position(socket_pose) else None
                bulb_released_to_socket = bulb_interaction.release_to_socket(socket_pose, socket_axis) or bulb_released_to_socket
            publish_gripper(pub_gripper, open_cmd)
            angular = stow_arm_control(arm, servo_ids, stow_joint_rad, stow_kp, stow_max_velocity)
            if elapsed > 1.0:
                controller.wrist_roll_cmd = 0.0
                pub_wrist.publish(Float64(data=0.0))
        elif state == 99:
            publish_gripper(pub_gripper, open_cmd)
            angular = zero_arm_control(servo_ids)
        else:
            rospy.logerr_throttle(1.0, "未知 mission_state=%s，机械臂停止", state)
            angular = zero_arm_control(servo_ids)

        msg_angular = Float64MultiArray()
        msg_angular.data = angular
        pub_angular.publish(msg_angular)
        if gripper_position is not None and gripper_axis is not None:
            bulb_interaction.follow_gripper(gripper_position, gripper_axis)
        pub_ready_state.publish(Int32(data=ready_state))
        msg_error = Float64MultiArray()
        display_pos_error = display_error(pos_error)
        display_axis_error = display_error(axis_error)
        display_line_error = display_error(line_error)
        msg_error.data = [display_pos_error, display_axis_error, display_line_error]
        pub_error.publish(msg_error)
        msg_workspace = Float64MultiArray()
        target_local = controller.last_target_local
        correction = controller.last_workspace_correction
        if target_local is not None and finite_array(target_local) and finite_array(correction):
            msg_workspace.data = [
                float(correction[0, 0]),
                float(correction[1, 0]),
                float(correction[2, 0]),
                float(target_local[0, 0]),
                float(target_local[1, 0]),
                float(target_local[2, 0]),
            ]
        else:
            msg_workspace.data = [0.0, 0.0, 0.0, float("nan"), float("nan"), float("nan")]
        pub_workspace_assist.publish(msg_workspace)
        rospy.loginfo_throttle(
            1.0,
            "机械臂控制诊断 state=%d %s elapsed=%.2f ready=%d "
            "err_pos=%.4f err_axis=%.2f err_line=%.4f "
            "qdot_cmd=[%.4f %.4f %.4f %.4f] wrist_cmd=%.4f "
            "workspace_corr=[%.4f %.4f %.4f]",
            state,
            state_name(state),
            elapsed,
            ready_state,
            display_pos_error,
            display_axis_error,
            display_line_error,
            angular[0] if len(angular) > 0 else float("nan"),
            angular[1] if len(angular) > 1 else float("nan"),
            angular[2] if len(angular) > 2 else float("nan"),
            angular[3] if len(angular) > 3 else float("nan"),
            controller.wrist_roll_cmd,
            msg_workspace.data[0],
            msg_workspace.data[1],
            msg_workspace.data[2],
        )
        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
