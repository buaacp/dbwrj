#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

"""Unified arm controller for the bulb replacement task.

This script is the current cleaned-up bulb arm mission.  It keeps the
same mission states and output topics.  Every tracking state builds an
ArmTarget and sends it through one MPC tracking function.  State-specific code
is limited to target selection, gripper action, wrist rotation, and ready-state
timing.
"""

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
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from geometry_msgs.msg import PoseStamped, TwistStamped
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


class ArmTarget:
    """World-frame end-effector target used by the MPC tracker."""

    def __init__(self, position, axis, label):
        self.position = np.array(position, dtype=float).reshape(3, 1)
        self.axis = normalize_vector(axis, [0.0, 0.0, -1.0])
        self.label = label


class MissionConfig:
    """All tunable parameters used by the arm mission."""

    def __init__(self):
        self.servo_port_name = get_param("servo_port_name", "/dev/ttyUSB0")
        self.servo_baudrate = int(get_param("servo_baudrate", 115200))
        self.servo_ids = [int(item) for item in get_vector_param("servo_ids", [0, 1, 2, 3], 4)]
        self.if_simulation = rospy.get_param("/if_simulation", False)
        self.mission_topic = get_param("mission_topic", "/mission_state")
        self.bulb_target_topic = get_param("bulb_target_topic", "/light_bulb/body_pose")
        self.socket_target_topic = get_param("socket_target_topic", "/light_bulb_fixture/socket_pose")
        self.stow_joint_rad = np.deg2rad(get_vector_param("stow_joint_deg", [-45.0, 90.0, 45.0, 0.0], 4))
        self.stow_kp = float(get_param("stow_kp", 1.2))
        self.stow_max_velocity = float(get_param("stow_max_velocity", 0.4))
        self.bulb_axis_local = normalize_vector(get_vector_param("bulb_axis_local", [0.0, 0.0, -1.0], 3), [0.0, 0.0, -1.0])
        self.socket_axis_local = normalize_vector(get_vector_param("socket_axis_local", [0.0, 0.0, -1.0], 3), [0.0, 0.0, -1.0])
        self.bulb_center_axis_offset = float(get_param("bulb_center_axis_offset", -0.07))
        self.socket_center_axis_offset = float(get_param("socket_center_axis_offset", -0.15))
        self.gripper_open = float(get_param("gripper_open", 0.55))
        self.gripper_closed = float(get_param("gripper_closed", 0.12))
        self.unscrew_turns = float(get_param("unscrew_turns", -1.0))
        self.screw_turns = float(get_param("screw_turns", 1.0))
        self.wrist_roll_rate = float(get_param("wrist_roll_rate", 0.8))
        self.socket_screw_duration = float(get_param("socket_screw_duration", get_param("socket_release_delay", 6.0)))
        self.align_pos_tolerance = float(get_param("align_pos_tolerance", 0.04))
        self.align_axis_tolerance_deg = float(get_param("align_axis_tolerance_deg", 3.0))
        self.centerline_tolerance = float(get_param("centerline_tolerance", 0.015))
        self.align_ready_duration = float(get_param("align_ready_duration", 0.5))
        self.grasp_close_duration = float(get_param("grasp_close_duration", 1.0))
        self.enable_wrist_roll = parse_bool(get_param("enable_wrist_roll", True))
        self.enable_gripper_action = parse_bool(get_param("enable_gripper_action", True))
        self.enable_kinematic_bulb_interaction = parse_bool(get_param("enable_kinematic_bulb_interaction", True))
        self.safe_demo_mode = parse_bool(get_param("safe_demo_mode", False))
        self.bulb_model_name = str(get_param("bulb_model_name", "light_bulb"))


class BulbArmController:
    """MPC wrapper for end-effector position and axis tracking."""

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

    def ready(self):
        return all(self.arm.angle.get(servo_id) is not None for servo_id in self.servo_ids)

    def update_uav_arm_state(self):
        self.uav_arm.p_b = np.array([[self.uav.vision_pose.x], [self.uav.vision_pose.y], [self.uav.vision_pose.z]], dtype=np.float64)
        self.uav_arm.phi = self.uav.roll * math.pi / 180.0
        self.uav_arm.theta = self.uav.pitch * math.pi / 180.0
        self.uav_arm.delta = self.uav.yaw * math.pi / 180.0
        self.uav_arm.d_phi = self.uav.angular.y * math.pi / 180.0
        self.uav_arm.d_theta = self.uav.angular.x * math.pi / 180.0
        self.uav_arm.d_delta = self.uav.angular.z * math.pi / 180.0
        self.uav_arm.d_xb = np.array([
            [self.uav.velocity.x],
            [self.uav.velocity.y],
            [self.uav.velocity.z],
            [self.uav_arm.d_theta],
            [self.uav_arm.d_phi],
            [self.uav_arm.d_delta],
        ])
        self.uav_arm.q = np.array([self.arm.angle[0], self.arm.angle[1], self.arm.angle[2], self.arm.angle[3]]).reshape(-1, 1)

    def current_gripper_pose(self):
        q0, q1, q2, q3 = [float(value) for value in self.uav_arm.q.flatten()]
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
        axis_local = normalize_vector([
            [-math.sin(q0) * math.sin(total)],
            [math.cos(q0) * math.sin(total)],
            [-math.cos(total)],
        ], [0.0, 0.0, -1.0])
        rot = self.body_rotation()
        position = self.uav_arm.p_b + rot.dot(self.uav_arm.p_delta + p_eb)
        axis_world = normalize_vector(rot.dot(axis_local), [0.0, 0.0, -1.0])
        return position, axis_world

    def body_rotation(self):
        rz = np.array([
            [math.cos(self.uav_arm.delta), -math.sin(self.uav_arm.delta), 0.0],
            [math.sin(self.uav_arm.delta), math.cos(self.uav_arm.delta), 0.0],
            [0.0, 0.0, 1.0],
        ])
        rx = np.array([
            [1.0, 0.0, 0.0],
            [0.0, math.cos(self.uav_arm.theta), -math.sin(self.uav_arm.theta)],
            [0.0, math.sin(self.uav_arm.theta), math.cos(self.uav_arm.theta)],
        ])
        ry = np.array([
            [math.cos(self.uav_arm.phi), 0.0, math.sin(self.uav_arm.phi)],
            [0.0, 1.0, 0.0],
            [-math.sin(self.uav_arm.phi), 0.0, math.cos(self.uav_arm.phi)],
        ])
        return rx.dot(ry).dot(rz)

    def track(self, target):
        """Run the single MPC tracking path for all target-tracking states."""
        if target is None or not finite_array(target.position) or not finite_array(target.axis):
            rospy.logwarn_throttle(1.0, "等待有效机械臂目标")
            return zero_arm_control(self.servo_ids), None, None, None, None, None
        if not self.ready():
            rospy.logwarn_throttle(2.0, "MPC 等待机械臂关节状态")
            return zero_arm_control(self.servo_ids), None, None, None, None, None

        self.update_uav_arm_state()
        if not finite_array(self.uav_arm.p_b) or not finite_array(self.uav_arm.q):
            rospy.logerr_throttle(1.0, "MPC 当前状态含 NaN/Inf，机械臂停止")
            return zero_arm_control(self.servo_ids), None, None, None, None, None

        self.uav_arm.pos_target = target.position
        self.uav_arm.use_grasp_axis = True
        self.uav_arm.grasp_axis_world = target.axis
        self.mpc.state_now = ca.DM([self.uav_arm.q[0], self.uav_arm.q[1], self.uav_arm.q[2], self.uav_arm.q[3]])
        x0 = ca.repmat(self.mpc.state_now, 1, self.mpc.N + 1)
        reference_path = self.mpc.get_target_path(self.uav_arm)
        self.mpc.set_reference(ca.vertcat(self.mpc.state_now, self.mpc.state_target, reference_path))
        self.mpc.set_x0(x0, self.mpc.u0)
        _, self.mpc.u0 = self.mpc.get_states_and_control()

        gripper_position, gripper_axis = self.current_gripper_pose()
        position_error = gripper_position - target.position
        pos_error = float(np.linalg.norm(position_error))
        line_error = float(np.linalg.norm(np.cross(position_error.flatten(), target.axis.flatten())))
        axis_dot = float(np.clip(gripper_axis.flatten().dot(target.axis.flatten()), -1.0, 1.0))
        axis_error = math.degrees(math.acos(axis_dot))
        rospy.loginfo_throttle(
            1.0,
            "%s MPC误差: pos=%.4f m axis=%.2f deg centerline=%.4f m",
            target.label,
            pos_error,
            axis_error,
            line_error,
        )
        return arm_velocity_from_mpc(self.mpc.u0[:, 0]), pos_error, axis_error, line_error, gripper_position, gripper_axis


class KinematicBulbInteraction:
    """Optional Gazebo-only bulb attachment model used for visual task feedback."""

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
        socket_center = np.array([[base.x], [base.y], [base.z]], dtype=float) + socket_axis * float(self.socket_center_axis_offset)
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
        bulb_origin = np.array(gripper_position, dtype=float).reshape(3, 1) - axis_world * float(self.bulb_center_axis_offset)
        self._set_model_pose(bulb_origin, self.roll_yaw)

    def set_roll(self, roll_yaw):
        if self.enabled and self.attached:
            self.roll_yaw = float(roll_yaw)

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
        try:
            self.service(msg)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(1.0, "设置灯泡模型状态失败: %s", exc)


class BulbArmMission:
    """Mechanical-arm state machine with a uniform target tracking path."""

    def __init__(self):
        global mission_state
        rospy.init_node("arm_control_bulb_0618", anonymous=True)
        self.config = MissionConfig()
        uart = None if self.config.if_simulation else create_serial_port_with_fallback(self.config.servo_port_name, self.config.servo_baudrate)
        if not self.config.if_simulation and uart is None:
            rospy.logerr("机械臂串口打开失败，进入 ABORT")
            mission_state = 99

        self.arm = ARM.ARM(uart=uart, SERVO_IDS=self.config.servo_ids, if_simulation=self.config.if_simulation)
        self.controller = BulbArmController(self.arm, self.config.servo_ids)
        self.interaction = KinematicBulbInteraction(
            self.config.if_simulation and self.config.enable_kinematic_bulb_interaction,
            self.config.bulb_model_name,
            self.config.bulb_center_axis_offset,
            self.config.socket_center_axis_offset,
        )
        self.pub_angular = rospy.Publisher("/le_arm_controller/command", Float64MultiArray, queue_size=10)
        self.pub_wrist = rospy.Publisher("/wrist_roll_controller/command", Float64, queue_size=10)
        self.pub_gripper = rospy.Publisher("/gripper_controller/command", JointTrajectory, queue_size=10)
        self.pub_ready = rospy.Publisher("/arm_control_bulb/ready_state", Int32, queue_size=10, latch=True)
        self.pub_error = rospy.Publisher("/arm_control_bulb/error", Float64MultiArray, queue_size=10)
        self.last_state = None
        self.state_enter_time = rospy.Time.now()
        self.ready_since = None
        self.hold_target = None
        self.hold_axis = None
        self.socket_screw_start_time = None
        self.bulb_released_to_socket = False

    def run(self):
        self.start_subscribers()
        time.sleep(0.5)
        rospy.loginfo("统一版机械臂灯泡任务启动: %s", describe_states())
        rate = rospy.Rate(20)
        control_dt = 1.0 / 20.0
        while not rospy.is_shutdown():
            state = mission_state
            self.on_state_entry(state)
            result = self.step_state(state, control_dt)
            self.publish_outputs(state, result)
            rate.sleep()

    def start_subscribers(self):
        topics = {
            "mission": self.config.mission_topic,
            "bulb": self.config.bulb_target_topic,
            "socket": self.config.socket_target_topic,
        }
        thread = threading.Thread(target=query_state_continuously, args=(self.arm, self.controller, topics))
        thread.daemon = True
        thread.start()

    def on_state_entry(self, state):
        if state == self.last_state:
            return
        rospy.loginfo("机械臂状态: %d %s", state, state_name(state))
        self.state_enter_time = rospy.Time.now()
        self.ready_since = None
        self.hold_target = None
        self.hold_axis = None
        self.socket_screw_start_time = None
        if state in (0, 1, 2, 3, 4, 5):
            self.bulb_released_to_socket = False
        self.last_state = state

    def step_state(self, state, control_dt):
        result = ArmStepResult()
        open_cmd = self.config.gripper_open
        closed_cmd = self.config.gripper_closed if self.config.enable_gripper_action else self.config.gripper_open

        if state == 0:
            result.angular = stow_arm_control(self.arm, self.config.servo_ids, self.config.stow_joint_rad, self.config.stow_kp, self.config.stow_max_velocity)
            result.gripper = open_cmd
            return result

        if state == 1:
            return self.track_alignment_state(state, self.bulb_target("PICK_ALIGN"), open_cmd)

        if state == 2:
            return self.track_grasp_bulb_state(open_cmd, closed_cmd)

        if state == 3:
            result = self.track_locked_target("UNSCREW_BULB", closed_cmd)
            result.ready_state = 3 if self.target_reached(result) else 0
            self.rotate_wrist(self.config.unscrew_turns, control_dt)
            return result

        if state == 4:
            # Mirror state1: track the bulb center that would result if the
            # bulb were seated in the socket, then report ready=4 when stable.
            return self.track_alignment_state(state, self.socket_bulb_center_target("TRANSFER_TO_SOCKET"), closed_cmd)

        if state == 5:
            # Mirror state2 without the grasp-close delay: keep the same
            # socket-side bulb-center target while preparing for screw-in.
            return self.track_alignment_state(state, self.socket_bulb_center_target("FINE_ALIGN_SOCKET"), closed_cmd)

        if state == 6:
            result = self.track_locked_target("SCREW_IN_BULB", open_cmd if self.bulb_released_to_socket else closed_cmd)
            if self.bulb_released_to_socket:
                result.ready_state = 6
            elif self.socket_screw_start_time is not None:
                screw_elapsed = (rospy.Time.now() - self.socket_screw_start_time).to_sec()
                self.rotate_wrist(self.config.screw_turns, control_dt)
                if screw_elapsed >= self.config.socket_screw_duration:
                    socket_pose = target_poses.get("socket")
                    socket_axis = pose_axis_world(socket_pose, self.config.socket_axis_local) if pose_has_position(socket_pose) else None
                    if self.interaction.release_to_socket(socket_pose, socket_axis):
                        self.bulb_released_to_socket = True
                    result.gripper = open_cmd
                    result.ready_state = 6
            return result

        if state == 7:
            if self.interaction.attached:
                socket_pose = target_poses.get("socket")
                socket_axis = pose_axis_world(socket_pose, self.config.socket_axis_local) if pose_has_position(socket_pose) else None
                self.bulb_released_to_socket = self.interaction.release_to_socket(socket_pose, socket_axis) or self.bulb_released_to_socket
            result.angular = stow_arm_control(self.arm, self.config.servo_ids, self.config.stow_joint_rad, self.config.stow_kp, self.config.stow_max_velocity)
            result.gripper = open_cmd
            if self.elapsed() > 1.0:
                self.controller.wrist_roll_cmd = 0.0
                self.pub_wrist.publish(Float64(data=0.0))
            return result

        if state == 99:
            result.angular = zero_arm_control(self.config.servo_ids)
            result.gripper = open_cmd
            return result

        rospy.logerr_throttle(1.0, "未知 mission_state=%s，机械臂停止", state)
        result.angular = zero_arm_control(self.config.servo_ids)
        result.gripper = open_cmd
        return result

    def track_alignment_state(self, state, target, gripper):
        result = self.track_target(target, gripper)
        if self.config.safe_demo_mode:
            result.angular = zero_arm_control(self.config.servo_ids)
            result.ready_state = state
            return result
        if self.target_stable(result):
            result.ready_state = state
        return result

    def track_grasp_bulb_state(self, open_cmd, closed_cmd):
        """Match original state2 timing: close first, then publish ready=2."""
        result = self.track_target(self.bulb_target("GRASP_BULB"), open_cmd)
        if self.config.safe_demo_mode:
            result.angular = zero_arm_control(self.config.servo_ids)
            if self.ready_since is None:
                self.ready_since = rospy.Time.now()
            result.gripper = closed_cmd if stable_for(self.ready_since, self.config.align_ready_duration) else open_cmd
            if stable_for(self.ready_since, self.config.align_ready_duration + self.config.grasp_close_duration):
                result.ready_state = 2
                self.interaction.attach()
            return result

        if self.target_reached(result):
            if self.ready_since is None:
                self.ready_since = rospy.Time.now()
            result.gripper = closed_cmd
            if stable_for(self.ready_since, self.config.grasp_close_duration):
                result.ready_state = 2
                self.interaction.attach()
        else:
            self.ready_since = None
            result.gripper = open_cmd
        return result

    def track_locked_target(self, label, gripper):
        if self.config.safe_demo_mode:
            result = ArmStepResult()
            result.angular = zero_arm_control(self.config.servo_ids)
            result.gripper = gripper
            result.ready_state = int(mission_state)
            return result
        if self.hold_target is None:
            if not self.controller.ready():
                rospy.logwarn_throttle(1.0, "%s 等待关节状态后锁定末端保持点", label)
                result = ArmStepResult()
                result.gripper = gripper
                return result
            self.controller.update_uav_arm_state()
            self.hold_target, self.hold_axis = self.controller.current_gripper_pose()
            if label == "SCREW_IN_BULB":
                self.socket_screw_start_time = rospy.Time.now()
            rospy.loginfo("%s 锁定末端保持点: [%.3f %.3f %.3f]", label, self.hold_target[0, 0], self.hold_target[1, 0], self.hold_target[2, 0])
        return self.track_target(ArmTarget(self.hold_target, self.hold_axis, label), gripper)

    def track_target(self, target, gripper):
        result = ArmStepResult()
        result.gripper = gripper
        result.angular, result.pos_error, result.axis_error, result.line_error, result.gripper_position, result.gripper_axis = self.controller.track(target)
        return result

    def bulb_target(self, label):
        return center_axis_target("bulb", self.config.bulb_axis_local, self.config.bulb_center_axis_offset, label)

    def socket_bulb_center_target(self, label):
        """Target the bulb center when the bulb is hypothetically seated."""
        return center_axis_target("socket", self.config.socket_axis_local, self.config.socket_center_axis_offset, label)

    def target_reached(self, result):
        return target_reached(
            result.pos_error,
            result.axis_error,
            self.config.align_pos_tolerance,
            self.config.align_axis_tolerance_deg,
            result.line_error,
            self.config.centerline_tolerance,
        )

    def target_stable(self, result):
        if self.target_reached(result):
            if self.ready_since is None:
                self.ready_since = rospy.Time.now()
            return stable_for(self.ready_since, self.config.align_ready_duration)
        self.ready_since = None
        return False

    def rotate_wrist(self, turns, control_dt):
        if not self.config.enable_wrist_roll:
            return
        self.controller.wrist_roll_cmd += self.config.wrist_roll_rate * float(turns) * control_dt
        self.pub_wrist.publish(Float64(data=self.controller.wrist_roll_cmd))
        self.interaction.set_roll(self.controller.wrist_roll_cmd)

    def publish_outputs(self, state, result):
        publish_gripper(self.pub_gripper, result.gripper)
        msg = Float64MultiArray()
        msg.data = result.angular
        self.pub_angular.publish(msg)
        if result.gripper_position is not None and result.gripper_axis is not None:
            self.interaction.follow_gripper(result.gripper_position, result.gripper_axis)
        self.pub_ready.publish(Int32(data=result.ready_state))
        err = Float64MultiArray()
        err.data = [display_error(result.pos_error), display_error(result.axis_error), display_error(result.line_error)]
        self.pub_error.publish(err)
        rospy.loginfo_throttle(
            1.0,
            "机械臂统一控制 state=%d %s elapsed=%.2f ready=%d err=[%.4f %.2f %.4f] qdot=[%.4f %.4f %.4f %.4f] wrist=%.4f",
            state,
            state_name(state),
            self.elapsed(),
            result.ready_state,
            err.data[0],
            err.data[1],
            err.data[2],
            result.angular[0] if len(result.angular) > 0 else float("nan"),
            result.angular[1] if len(result.angular) > 1 else float("nan"),
            result.angular[2] if len(result.angular) > 2 else float("nan"),
            result.angular[3] if len(result.angular) > 3 else float("nan"),
            self.controller.wrist_roll_cmd,
        )

    def elapsed(self):
        return (rospy.Time.now() - self.state_enter_time).to_sec()


class ArmStepResult:
    def __init__(self):
        self.angular = []
        self.pos_error = float("nan")
        self.axis_error = float("nan")
        self.line_error = float("nan")
        self.gripper_position = None
        self.gripper_axis = None
        self.ready_state = 0
        self.gripper = 0.55


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
    return normalize_vector(quaternion_to_rotation(pose_stamped.pose.orientation).dot(local_axis), [0.0, 0.0, -1.0])


def pose_position_array(pose_stamped):
    position = pose_stamped.pose.position
    return np.array([[position.x], [position.y], [position.z]], dtype=float)


def pose_has_position(pose_stamped):
    if pose_stamped is None:
        return False
    position = pose_stamped.pose.position
    values = (position.x, position.y, position.z)
    return finite_array(values) and any(abs(value) > 1e-6 for value in values)


def center_axis_target(target_name, axis_local, center_axis_offset, label):
    pose = target_poses.get(target_name)
    if not pose_has_position(pose):
        rospy.logwarn_throttle(1.0, "%s 等待 %s 位姿", label, target_name)
        return None
    axis = pose_axis_world(pose, axis_local)
    position = pose_position_array(pose) + axis * float(center_axis_offset)
    return ArmTarget(position, axis, label)


def target_pose_callback(msg, target_name):
    target_poses[target_name] = msg


def mission_state_callback(msg):
    global mission_state
    mission_state = msg.data


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


def create_serial_port_with_fallback(port_name, baudrate):
    uart = create_serial_port(port_name, baudrate)
    if uart is None and port_name != "/dev/ttyUSB1":
        uart = create_serial_port("/dev/ttyUSB1", baudrate)
    return uart


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


def stable_for(start_time, duration):
    return start_time is not None and (rospy.Time.now() - start_time).to_sec() >= duration


def display_error(value):
    return float(value) if value is not None else float("nan")


def state_name(state):
    return STATE_NAMES.get(state, "UNKNOWN")


def describe_states():
    return ", ".join("%d=%s" % (state, name) for state, name in sorted(STATE_NAMES.items()))


if __name__ == "__main__":
    try:
        BulbArmMission().run()
    except rospy.ROSInterruptException:
        pass
