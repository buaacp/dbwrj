#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import math
import threading

import rospy

from geometry_msgs.msg import Pose
from multirotor_communication import Communication
from std_msgs.msg import Float64MultiArray, Int32


MAX_VELOCITY = 1.0
PD = 2.0

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

arm_ready_state = 0
arm_error = [float("nan"), float("nan")]
workspace_assist = {
    "correction": [0.0, 0.0, 0.0],
    "target_local": [float("nan"), float("nan"), float("nan")],
    "stamp": None,
}


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in ("1", "true", "yes", "on")
    return bool(value)


def arm_ready_callback(msg):
    global arm_ready_state
    arm_ready_state = int(msg.data)


def arm_error_callback(msg):
    global arm_error
    if len(msg.data) >= 2:
        arm_error = [float(item) for item in msg.data[:3]]


def workspace_assist_callback(msg):
    global workspace_assist
    if len(msg.data) >= 6 and finite_values(*msg.data[:3]):
        workspace_assist = {
            "correction": [float(item) for item in msg.data[:3]],
            "target_local": [float(item) for item in msg.data[3:6]],
            "stamp": rospy.Time.now(),
        }


def arm_ready_for(state, require_arm_ready):
    if not require_arm_ready:
        return True
    return arm_ready_state == state


def communication_state_value(predicate, field):
    communication = getattr(predicate, "communication", None)
    if communication is None:
        return None
    return getattr(communication.mavros_state, field, None)


def bind_condition(communication, condition):
    condition.communication = communication
    return condition


def wait_for_condition(description, predicate, timeout):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if predicate():
            return True
        rate.sleep()
    rospy.logerr(
        "等待%s超时，当前 mavros_state: connected=%s armed=%s mode=%s",
        description,
        communication_state_value(predicate, "connected"),
        communication_state_value(predicate, "armed"),
        communication_state_value(predicate, "mode"),
    )
    return False


def stable_local_position(communication):
    if not communication.has_local_position or not finite_position(communication.current_position):
        return False
    # Reject obviously invalid EKF transients before enabling OFFBOARD.
    return abs(communication.current_position.z) < 20.0


def publish_startup_setpoints(communication, duration, rate_hz=20):
    rate = rospy.Rate(rate_hz)
    deadline = rospy.Time.now() + rospy.Duration(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if not stable_local_position(communication):
            rospy.logwarn_throttle(1.0, "等待稳定本地位置后再预发布 OFFBOARD setpoint")
            return False
        hold_current_velocity_setpoint(communication)
        rate.sleep()
    return True


def set_offboard_mode(communication, max_attempts=5):
    for attempt_count in range(1, max_attempts + 1):
        try:
            res = communication.flightModeService(base_mode=0, custom_mode="OFFBOARD")
        except rospy.ServiceException as exc:
            rospy.logwarn("切换 OFFBOARD 服务调用失败: %s", exc)
            res = None

        if res is not None and res.mode_sent:
            if wait_for_condition(
                    "实际进入 OFFBOARD",
                    bind_condition(communication, lambda: communication.mavros_state.mode == "OFFBOARD"),
                    2.0):
                rospy.loginfo("飞行模式已切换为 OFFBOARD")
                return True

        if communication.mavros_state.mode == "OFFBOARD":
            rospy.loginfo("飞行模式已切换为 OFFBOARD")
            return True

        rospy.logwarn("飞行模式切换失败，正在尝试第%d次...", attempt_count)
        rospy.sleep(1.0)
    return False


def arm_vehicle(communication, max_attempts=5):
    for attempt_count in range(1, max_attempts + 1):
        if communication.mavros_state.armed:
            rospy.loginfo("飞机已解锁")
            return True

        if communication.arm():
            rospy.loginfo("飞机已解锁")
            return True

        rospy.logwarn("飞机解锁失败，正在尝试第%d次...", attempt_count)
        rospy.sleep(2.0)
    return False


def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)


def pose_has_position(pose):
    return any(abs(value) > 1e-6 for value in (
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ))


def finite_values(*values):
    return all(math.isfinite(float(value)) for value in values)


def finite_position(position):
    return position is not None and finite_values(position.x, position.y, position.z)


def finite_velocity(velocity_msg):
    if velocity_msg is None:
        return False
    linear = velocity_msg.twist.linear
    return finite_values(linear.x, linear.y, linear.z)


def finite_pose(pose):
    return pose is not None and finite_position(pose.position)


def safe_yaw(communication, fallback=0.0):
    if math.isfinite(float(communication.current_yaw)):
        return communication.current_yaw
    return fallback


def get_target_pose(communication, target_name):
    pose = communication.target_poses.get(target_name)
    if pose is None or not finite_pose(pose) or not pose_has_position(pose):
        return None
    return pose


def target_from_pose(pose, offset_xyz, correction_xyz=None):
    correction_xyz = correction_xyz or [0.0, 0.0, 0.0]
    target = Pose()
    target.position.x = pose.position.x + offset_xyz[0] + correction_xyz[0]
    target.position.y = pose.position.y + offset_xyz[1] + correction_xyz[1]
    target.position.z = pose.position.z + offset_xyz[2] + correction_xyz[2]
    return target


def target_from_center_pose(pose, center_axis_offset, offset_xyz, correction_xyz=None):
    center = Pose()
    center.position.x = pose.position.x
    center.position.y = pose.position.y
    center.position.z = pose.position.z - float(center_axis_offset)
    return target_from_pose(center, offset_xyz, correction_xyz)


def vector_param(name, default):
    value = rospy.get_param(name, default)
    if isinstance(value, str):
        value = [float(item) for item in value.strip("[]").split(",")]
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        rospy.logwarn("参数 %s=%s 无效，使用默认值 %s", name, value, default)
        value = default
    return [float(item) for item in value]


def clamp_vector_norm(vector, max_norm):
    norm = math.sqrt(sum(item * item for item in vector))
    if norm > max_norm > 1e-6:
        scale = max_norm / norm
        return [item * scale for item in vector]
    return vector


def current_workspace_correction(enable_assist, state, filtered, max_norm, alpha, timeout):
    if not enable_assist or state not in (1, 2, 3, 5):
        return [item * alpha for item in filtered]
    stamp = workspace_assist.get("stamp")
    if stamp is None or (rospy.Time.now() - stamp).to_sec() > timeout:
        return [item * alpha for item in filtered]
    raw = clamp_vector_norm(workspace_assist["correction"], max_norm)
    updated = [
        alpha * filtered[i] + (1.0 - alpha) * raw[i]
        for i in range(3)
    ]
    return clamp_vector_norm(updated, max_norm)


def set_velocity_target(communication, vx, vy, vz, yaw):
    if not finite_values(vx, vy, vz, yaw):
        rospy.logerr("拒绝发布非法速度 setpoint: vx=%s vy=%s vz=%s yaw=%s", vx, vy, vz, yaw)
        return False
    communication.last_control_velocity = (float(vx), float(vy), float(vz))
    communication.last_control_yaw = float(normalize_angle(yaw))
    communication.coordinate_frame = 1
    communication.motion_type = 1
    communication.target_motion = communication.construct_target(
        vx=vx,
        vy=vy,
        vz=vz,
        yaw=normalize_angle(yaw),
    )
    return True


def limit_horizontal_velocity(vx, vy, max_velocity):
    speed_xy = math.sqrt(vx ** 2 + vy ** 2)
    if speed_xy > max_velocity:
        scale = max_velocity / speed_xy
        return vx * scale, vy * scale
    return vx, vy


def apply_anti_orbit_control(communication, target_pose, vx, vy, config):
    if not config.get("enabled", False):
        return vx, vy
    current = communication.current_position
    if not finite_position(current) or not finite_pose(target_pose) or not finite_velocity(communication.current_velocity):
        return vx, vy

    delta_x = target_pose.position.x - current.x
    delta_y = target_pose.position.y - current.y
    distance_xy = math.sqrt(delta_x ** 2 + delta_y ** 2)
    min_radius = float(config.get("min_radius", 0.03))
    if distance_xy < min_radius:
        return vx, vy

    radial_x = delta_x / distance_xy
    radial_y = delta_y / distance_xy
    current_vx = communication.current_velocity.twist.linear.x
    current_vy = communication.current_velocity.twist.linear.y
    radial_speed = current_vx * radial_x + current_vy * radial_y
    tangential_vx = current_vx - radial_speed * radial_x
    tangential_vy = current_vy - radial_speed * radial_y
    damping = float(config.get("damping", 0.6))
    vx -= damping * tangential_vx
    vy -= damping * tangential_vy

    rospy.loginfo_throttle(
        1.0,
        "抗绕控制: distance_xy=%.3f tangential_v=[%.3f %.3f] damping=%.2f",
        distance_xy,
        tangential_vx,
        tangential_vy,
        damping,
    )
    return vx, vy


def apply_obs_compensation(communication, target_pose, vx, vy, config):
    if not config.get("enabled", False):
        return vx, vy
    current = communication.current_position
    if not finite_position(current) or not finite_pose(target_pose) or not finite_velocity(communication.current_velocity):
        return vx, vy

    delta_x = target_pose.position.x - current.x
    delta_y = target_pose.position.y - current.y
    distance_xy = math.sqrt(delta_x ** 2 + delta_y ** 2)
    activation_radius = float(config.get("activation_radius", 0.5))
    state = config.setdefault("state", {
        "last_time": None,
        "dx_hat": 0.0,
        "dy_hat": 0.0,
    })
    if activation_radius > 1e-6 and distance_xy > activation_radius:
        state["last_time"] = None
        state["dx_hat"] *= 0.9
        state["dy_hat"] *= 0.9
        return vx, vy

    now = rospy.Time.now()
    if state["last_time"] is None:
        dt = 0.05
    else:
        dt = max((now - state["last_time"]).to_sec(), 1e-3)
    state["last_time"] = now

    measured_vx = communication.current_velocity.twist.linear.x
    measured_vy = communication.current_velocity.twist.linear.y
    innovation_x = measured_vx - vx
    innovation_y = measured_vy - vy
    tau = max(float(config.get("time_constant", 0.8)), 1e-3)
    alpha = clamp(dt / (tau + dt), 0.0, 1.0)
    state["dx_hat"] += alpha * (innovation_x - state["dx_hat"])
    state["dy_hat"] += alpha * (innovation_y - state["dy_hat"])

    max_estimate = float(config.get("max_estimate", 0.25))
    estimate_norm = math.sqrt(state["dx_hat"] ** 2 + state["dy_hat"] ** 2)
    if estimate_norm > max_estimate > 1e-6:
        scale = max_estimate / estimate_norm
        state["dx_hat"] *= scale
        state["dy_hat"] *= scale

    gain = float(config.get("gain", 0.5))
    vx -= gain * state["dx_hat"]
    vy -= gain * state["dy_hat"]

    rospy.loginfo_throttle(
        1.0,
        "OBS补偿: distance_xy=%.3f d_hat=[%.3f %.3f] gain=%.2f",
        distance_xy,
        state["dx_hat"],
        state["dy_hat"],
        gain,
    )
    return vx, vy


def hold_default_pose(communication, pd_gain, max_velocity, max_vz):
    target_x = communication.default_pose.position.x
    target_y = communication.default_pose.position.y
    target_z = communication.default_pose.position.z
    current = communication.current_position
    if not finite_position(current):
        return False

    control_vx = (target_x - current.x) * pd_gain
    control_vy = (target_y - current.y) * pd_gain
    control_vz = clamp((target_z - current.z) * pd_gain, -max_vz, max_vz)

    control_vx, control_vy = limit_horizontal_velocity(control_vx, control_vy, max_velocity)

    target_yaw = safe_yaw(communication)
    if not set_velocity_target(communication, control_vx, control_vy, control_vz, target_yaw):
        return False
    communication.last_control_error = math.sqrt(
        (target_x - current.x) ** 2
        + (target_y - current.y) ** 2
        + (target_z - current.z) ** 2
    )
    communication.last_control_target = (target_x, target_y, target_z)
    return True


def fly_to_pose(
        communication,
        target_pose,
        pd_gain,
        max_velocity,
        max_vz,
        target_yaw,
        anti_orbit_config=None,
        obs_config=None):
    current = communication.current_position
    if not finite_position(current) or not finite_pose(target_pose):
        return None
    delta_x = target_pose.position.x - current.x
    delta_y = target_pose.position.y - current.y
    delta_z = target_pose.position.z - current.z

    control_vx = delta_x * pd_gain
    control_vy = delta_y * pd_gain
    control_vz = clamp(delta_z * pd_gain, -max_vz, max_vz)

    control_vx, control_vy = apply_anti_orbit_control(
        communication,
        target_pose,
        control_vx,
        control_vy,
        anti_orbit_config or {},
    )
    control_vx, control_vy = apply_obs_compensation(
        communication,
        target_pose,
        control_vx,
        control_vy,
        obs_config or {},
    )
    control_vx, control_vy = limit_horizontal_velocity(control_vx, control_vy, max_velocity)

    if not finite_values(control_vx, control_vy, control_vz, target_yaw):
        return None
    if not set_velocity_target(communication, control_vx, control_vy, control_vz, target_yaw):
        return None
    distance = math.sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2)
    communication.last_control_error = distance
    communication.last_control_target = (
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z,
    )
    return distance


def hold_current_velocity_setpoint(communication):
    ok = set_velocity_target(communication, 0.0, 0.0, 0.0, safe_yaw(communication))
    if ok:
        communication.last_control_error = 0.0
    return ok


def hold_target_pose(
        communication,
        target_pose_name,
        offset_xyz,
        pd_gain,
        max_velocity,
        max_vz,
        yaw,
        state_label,
        correction_xyz=None,
        anti_orbit_config=None,
        obs_config=None):
    target_pose = get_target_pose(communication, target_pose_name)
    if target_pose is None:
        rospy.logerr_throttle(1.0, "%s 缺少目标位姿，进入 ABORT", state_label)
        return None
    target = target_from_pose(target_pose, offset_xyz, correction_xyz)
    distance = fly_to_pose(communication, target, pd_gain, max_velocity, max_vz, yaw, anti_orbit_config, obs_config)
    if distance is None:
        rospy.logerr("%s 控制输入含 NaN/Inf，进入 ABORT", state_label)
    return distance


def fly_to_bulb_grasp_pose(
        communication,
        bulb_center_axis_offset,
        pick_uav_offset,
        filtered_workspace_correction,
        pd_gain,
        target_max_velocity,
        max_vz,
        yaw,
        state_label):
    bulb_pose = get_target_pose(communication, "bulb")
    if bulb_pose is None:
        rospy.logerr_throttle(1.0, "%s 缺少灯泡位姿，进入 ABORT", state_label)
        return None
    target = target_from_center_pose(
        bulb_pose,
        bulb_center_axis_offset,
        pick_uav_offset,
        filtered_workspace_correction,
    )
    distance = fly_to_pose(
        communication,
        target,
        pd_gain,
        target_max_velocity,
        max_vz,
        yaw,
        {},
        {},
    )
    if distance is None:
        rospy.logerr("%s 控制输入含 NaN/Inf，进入 ABORT", state_label)
    return distance


def state_name(state):
    return STATE_NAMES.get(state, "UNKNOWN")


def publish_state(pub, state):
    pub.publish(Int32(data=int(state)))


def main():
    rospy.init_node("test_fly_bulb_0615")

    vehicle_type = rospy.get_param("~vehicle_type", "iris")
    vehicle_id = str(rospy.get_param("~vehicle_id", "0"))
    communication = Communication(vehicle_type, vehicle_id)

    pd_gain = float(rospy.get_param("~pd_gain", PD))
    default_height = float(rospy.get_param("~default_height", 1.0))
    default_max_velocity = float(rospy.get_param("~default_max_velocity", MAX_VELOCITY))
    target_max_velocity = float(rospy.get_param("~target_max_velocity", 0.25))
    fine_max_velocity = float(rospy.get_param("~fine_max_velocity", 0.08))
    max_vz = float(rospy.get_param("~max_vertical_velocity", 0.5))
    position_tolerance = float(rospy.get_param("~position_tolerance", 0.07))
    fine_position_tolerance = float(rospy.get_param("~fine_position_tolerance", 0.04))
    auto_start = parse_bool(rospy.get_param("~auto_start", True))
    require_arm_ready = parse_bool(rospy.get_param("~require_arm_ready", True))
    enable_workspace_assist = parse_bool(rospy.get_param("~enable_workspace_assist", True))
    workspace_assist_max_offset = float(rospy.get_param("~workspace_assist_max_offset", 0.12))
    workspace_assist_alpha = float(rospy.get_param("~workspace_assist_filter_alpha", 0.85))
    workspace_assist_timeout = float(rospy.get_param("~workspace_assist_timeout", 0.5))
    anti_orbit_config = {
        "enabled": parse_bool(rospy.get_param("~enable_anti_orbit_control", True)),
        "damping": float(rospy.get_param("~anti_orbit_damping", 0.6)),
        "min_radius": float(rospy.get_param("~anti_orbit_min_radius", 0.03)),
    }
    obs_config = {
        "enabled": parse_bool(rospy.get_param("~enable_obs_compensation", True)),
        "gain": float(rospy.get_param("~obs_compensation_gain", 0.5)),
        "time_constant": float(rospy.get_param("~obs_time_constant", 0.8)),
        "activation_radius": float(rospy.get_param("~obs_activation_radius", 0.5)),
        "max_estimate": float(rospy.get_param("~obs_max_estimate", 0.25)),
    }
    grasp_duration = float(rospy.get_param("~grasp_duration", 3.0))
    unscrew_duration = float(rospy.get_param("~unscrew_duration", 6.0))
    screw_duration = float(rospy.get_param("~screw_duration", 6.0))
    finish_duration = float(rospy.get_param("~finish_duration", 4.0))
    bulb_center_axis_offset = float(rospy.get_param("~bulb_center_axis_offset", -0.07))
    socket_center_axis_offset = float(rospy.get_param("~socket_center_axis_offset", -0.15))
    pick_uav_offset = vector_param("~pick_uav_offset", [0.0, 0.096, 0.150])
    socket_uav_offset = vector_param("~socket_uav_offset", [0.0, 0.096, 0.150])
    retreat_offset = vector_param("~retreat_offset", [-0.35, 0.0, 0.25])
    fixed_yaw = rospy.get_param("~target_yaw_deg", "")
    fixed_yaw = None if str(fixed_yaw).strip() == "" else math.radians(float(fixed_yaw))
    communication.default_pose.position.z = default_height
    mission_pub = rospy.Publisher("/mission_state", Int32, queue_size=10, latch=True)
    rospy.Subscriber("/arm_control_bulb/ready_state", Int32, arm_ready_callback)
    rospy.Subscriber("/arm_control_bulb/error", Float64MultiArray, arm_error_callback)
    rospy.Subscriber("/arm_control_bulb/workspace_assist", Float64MultiArray, workspace_assist_callback)

    communication_thread = threading.Thread(target=communication.start)
    communication_thread.daemon = True
    communication_thread.start()

    namespace = "/%s_%s" % (vehicle_type, vehicle_id)
    try:
        rospy.wait_for_service(namespace + "/mavros/cmd/arming", timeout=30)
        rospy.wait_for_service(namespace + "/mavros/set_mode", timeout=30)
    except rospy.ROSException as exc:
        rospy.logerr("等待 MAVROS 服务超时: %s", exc)
        return

    if not wait_for_condition(
            "MAVROS连接",
            bind_condition(communication, lambda: communication.mavros_state.connected),
            30.0):
        return
    if not wait_for_condition(
            "PX4稳定本地位置反馈",
            bind_condition(communication, lambda: stable_local_position(communication)),
            30.0):
        return

    rospy.loginfo("PX4本地位置反馈已稳定，开始预发布 OFFBOARD setpoint。")
    if not publish_startup_setpoints(communication, 5.0):
        rospy.logerr("预发布 OFFBOARD setpoint 失败，进入 ABORT。")
        hold_current_velocity_setpoint(communication)
        return

    if not set_offboard_mode(communication):
        rospy.logerr("飞行模式切换失败，进入 ABORT。")
        hold_current_velocity_setpoint(communication)
        return

    if not publish_startup_setpoints(communication, 1.0):
        rospy.logerr("OFFBOARD 后 setpoint 不稳定，进入 ABORT。")
        hold_current_velocity_setpoint(communication)
        return

    if not arm_vehicle(communication):
        rospy.logerr("飞机解锁失败，进入 ABORT。")
        hold_current_velocity_setpoint(communication)
        return

    rospy.loginfo("灯泡更换任务状态机已启动")
    rospy.loginfo("mission_state: 0 IDLE, 1 PICK_ALIGN, 2 GRASP_BULB, 3 UNSCREW_BULB, 4 TRANSFER_TO_SOCKET, 5 FINE_ALIGN_SOCKET, 6 SCREW_IN_BULB, 7 FINISH, 99 ABORT")
    if anti_orbit_config["enabled"]:
        rospy.loginfo(
            "无人机抗绕控制已启用: damping=%.2f min_radius=%.3f",
            anti_orbit_config["damping"],
            anti_orbit_config["min_radius"],
        )
    if obs_config["enabled"]:
        rospy.loginfo(
            "无人机OBS扰动补偿已启用: gain=%.2f tau=%.2f radius=%.2f max_est=%.2f",
            obs_config["gain"],
            obs_config["time_constant"],
            obs_config["activation_radius"],
            obs_config["max_estimate"],
        )

    current_state = 0
    publish_state(mission_pub, current_state)
    last_state = None
    state_start_time = rospy.Time.now()
    aborted = False
    filtered_workspace_correction = [0.0, 0.0, 0.0]
    unscrew_uav_target = None
    screw_uav_target = None
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        if aborted:
            current_state = 99
        state = current_state

        if state != last_state:
            rospy.loginfo("无人机任务状态: %s %s", state, state_name(state))
            publish_state(mission_pub, state)
            state_start_time = rospy.Time.now()
            if state != 3:
                unscrew_uav_target = None
            if state != 6:
                screw_uav_target = None
            last_state = state

        if communication.current_position is None:
            rospy.logwarn_throttle(2.0, "等待 PX4 本地位置反馈，保持当前速度 setpoint")
            hold_current_velocity_setpoint(communication)
            rate.sleep()
            continue

        elapsed = (rospy.Time.now() - state_start_time).to_sec()
        filtered_workspace_correction = current_workspace_correction(
            enable_workspace_assist,
            state,
            filtered_workspace_correction,
            workspace_assist_max_offset,
            workspace_assist_alpha,
            workspace_assist_timeout,
        )
        if not finite_position(communication.current_position):
            rospy.logwarn_throttle(
                1.0,
                "PX4 local_position 含 NaN/Inf，保守保持零速度 setpoint，暂停任务状态推进",
            )
            publish_state(mission_pub, state)
            hold_current_velocity_setpoint(communication)
            rate.sleep()
            continue

        yaw = fixed_yaw if fixed_yaw is not None else safe_yaw(communication)

        if state == 0:
            if not hold_default_pose(communication, pd_gain, default_max_velocity, max_vz):
                aborted = True
            current = communication.current_position
            default_height_error = abs(current.z - communication.default_pose.position.z) if finite_position(current) else float("inf")
            if auto_start and elapsed > 2.0 and default_height_error < position_tolerance:
                current_state = 1
            elif auto_start and elapsed > 2.0:
                rospy.loginfo_throttle(
                    1.0,
                    "IDLE 等待无人机到达默认悬停高度: z=%.3f target=%.3f err=%.3f",
                    current.z if finite_position(current) else float("nan"),
                    communication.default_pose.position.z,
                    default_height_error,
                )
        elif state == 1:
            distance = fly_to_bulb_grasp_pose(
                communication,
                bulb_center_axis_offset,
                pick_uav_offset,
                filtered_workspace_correction,
                pd_gain,
                target_max_velocity,
                max_vz,
                yaw,
                "PICK_ALIGN",
            )
            if distance is None:
                aborted = True
            else:
                rospy.loginfo_throttle(
                    1.0,
                    "PICK_ALIGN 距离目标 %.3f m workspace_corr=[%.3f %.3f %.3f]",
                    distance,
                    filtered_workspace_correction[0],
                    filtered_workspace_correction[1],
                    filtered_workspace_correction[2],
                )
                if arm_ready_for(1, require_arm_ready):
                    current_state = 2
                else:
                    rospy.loginfo_throttle(
                        1.0,
                        "PICK_ALIGN 等待机械臂到达灯泡附近: ready=%s uav_dist=%.3f pos=%.3f axis=%.2f centerline=%.3f",
                        arm_ready_state,
                        distance if distance is not None else float("nan"),
                        arm_error[0],
                        arm_error[1],
                        arm_error[2] if len(arm_error) > 2 else float("nan"),
                    )
        elif state == 2:
            distance = fly_to_bulb_grasp_pose(
                communication,
                bulb_center_axis_offset,
                pick_uav_offset,
                filtered_workspace_correction,
                pd_gain,
                target_max_velocity,
                max_vz,
                yaw,
                "GRASP_BULB",
            )
            if distance is None:
                aborted = True
                rate.sleep()
                continue
            if elapsed > grasp_duration and arm_ready_for(2, require_arm_ready):
                current_state = 3
            elif elapsed > grasp_duration:
                rospy.loginfo_throttle(
                    1.0,
                    "GRASP_BULB 等待机械臂抓取位到位: ready=%s pos=%.3f axis=%.2f centerline=%.3f",
                    arm_ready_state,
                    arm_error[0],
                    arm_error[1],
                    arm_error[2] if len(arm_error) > 2 else float("nan"),
                )
        elif state == 3:
            if unscrew_uav_target is None:
                current = communication.current_position
                unscrew_uav_target = Pose()
                unscrew_uav_target.position.x = current.x
                unscrew_uav_target.position.y = current.y
                unscrew_uav_target.position.z = current.z
                rospy.loginfo(
                    "UNSCREW_BULB 锁定当前无人机悬停点: [%.3f %.3f %.3f]",
                    unscrew_uav_target.position.x,
                    unscrew_uav_target.position.y,
                    unscrew_uav_target.position.z,
                )
            distance = fly_to_pose(
                communication,
                unscrew_uav_target,
                pd_gain,
                target_max_velocity,
                max_vz,
                yaw,
                anti_orbit_config,
                obs_config,
            )
            if distance is None:
                rospy.logerr("UNSCREW_BULB 控制输入含 NaN/Inf，进入 ABORT")
                aborted = True
                rate.sleep()
                continue
            if elapsed > unscrew_duration:
                current_state = 4
        elif state == 4:
            socket_pose = get_target_pose(communication, "socket")
            if socket_pose is None:
                rospy.logerr_throttle(1.0, "TRANSFER_TO_SOCKET 缺少灯座位姿，进入 ABORT")
                aborted = True
            else:
                target = target_from_center_pose(
                    socket_pose,
                    socket_center_axis_offset,
                    socket_uav_offset,
                    filtered_workspace_correction,
                )
                distance = fly_to_pose(
                    communication,
                    target,
                    pd_gain,
                    target_max_velocity,
                    max_vz,
                    yaw,
                    anti_orbit_config,
                    obs_config,
                )
                if distance is None:
                    rospy.logerr("TRANSFER_TO_SOCKET 控制输入含 NaN/Inf，进入 ABORT")
                    aborted = True
                else:
                    rospy.loginfo_throttle(1.0, "TRANSFER_TO_SOCKET 距离目标 %.3f m", distance)
                if distance is not None and distance < position_tolerance:
                    current_state = 5
        elif state == 5:
            socket_pose = get_target_pose(communication, "socket")
            if socket_pose is None:
                rospy.logerr_throttle(1.0, "FINE_ALIGN_SOCKET 缺少灯座位姿，进入 ABORT")
                aborted = True
            else:
                target = target_from_center_pose(
                    socket_pose,
                    socket_center_axis_offset,
                    socket_uav_offset,
                    filtered_workspace_correction,
                )
                distance = fly_to_pose(
                    communication,
                    target,
                    pd_gain,
                    target_max_velocity,
                    max_vz,
                    yaw,
                    {},
                    {},
                )
                if distance is None:
                    rospy.logerr("FINE_ALIGN_SOCKET 控制输入含 NaN/Inf，进入 ABORT")
                    aborted = True
                else:
                    rospy.loginfo_throttle(1.0, "FINE_ALIGN_SOCKET 距离目标 %.3f m", distance)
                if elapsed > grasp_duration and arm_ready_for(5, require_arm_ready):
                    current_state = 6
                elif elapsed > grasp_duration:
                    rospy.loginfo_throttle(
                        1.0,
                        "FINE_ALIGN_SOCKET 等待机械臂对准灯座: ready=%s uav_dist=%.3f pos=%.3f axis=%.2f",
                        arm_ready_state,
                        distance if distance is not None else float("nan"),
                        arm_error[0],
                        arm_error[1],
                    )
        elif state == 6:
            if screw_uav_target is None:
                current = communication.current_position
                screw_uav_target = Pose()
                screw_uav_target.position.x = current.x
                screw_uav_target.position.y = current.y
                screw_uav_target.position.z = current.z
                rospy.loginfo(
                    "SCREW_IN_BULB 锁定当前无人机悬停点: [%.3f %.3f %.3f]",
                    screw_uav_target.position.x,
                    screw_uav_target.position.y,
                    screw_uav_target.position.z,
                )
            distance = fly_to_pose(
                communication,
                screw_uav_target,
                pd_gain,
                fine_max_velocity,
                max_vz,
                yaw,
                {},
                {},
            )
            if distance is None:
                rospy.logerr("SCREW_IN_BULB 控制输入含 NaN/Inf，进入 ABORT")
                aborted = True
                rate.sleep()
                continue
            if elapsed > screw_duration and arm_ready_for(6, require_arm_ready):
                current_state = 7
            elif elapsed > screw_duration:
                rospy.loginfo_throttle(
                    1.0,
                    "SCREW_IN_BULB 等待机械臂拧入并释放: ready=%s pos=%.3f axis=%.2f centerline=%.3f",
                    arm_ready_state,
                    arm_error[0],
                    arm_error[1],
                    arm_error[2] if len(arm_error) > 2 else float("nan"),
                )
        elif state == 7:
            target = Pose()
            target.position.x = communication.default_pose.position.x + retreat_offset[0]
            target.position.y = communication.default_pose.position.y + retreat_offset[1]
            target.position.z = communication.default_pose.position.z + retreat_offset[2]
            fly_to_pose(communication, target, pd_gain, target_max_velocity, max_vz, yaw, {}, {})
            if elapsed > finish_duration:
                hold_current_velocity_setpoint(communication)
        elif state == 99:
            publish_state(mission_pub, 99)
            hold_current_velocity_setpoint(communication)
        else:
            rospy.logerr("未知 mission_state=%s，进入 ABORT", state)
            aborted = True
            hold_current_velocity_setpoint(communication)

        control_velocity = getattr(communication, "last_control_velocity", (float("nan"), float("nan"), float("nan")))
        control_error = getattr(communication, "last_control_error", float("nan"))
        control_target = getattr(communication, "last_control_target", (float("nan"), float("nan"), float("nan")))
        rospy.loginfo_throttle(
            1.0,
            "UAV控制诊断 state=%d %s elapsed=%.2f err=%.3f target=[%.3f %.3f %.3f] "
            "v_cmd=[%.3f %.3f %.3f] yaw_cmd=%.3f workspace_corr=[%.3f %.3f %.3f] arm_ready=%d arm_err=[%.3f %.2f %.3f]",
            state,
            state_name(state),
            elapsed,
            control_error,
            control_target[0],
            control_target[1],
            control_target[2],
            control_velocity[0],
            control_velocity[1],
            control_velocity[2],
            getattr(communication, "last_control_yaw", float("nan")),
            filtered_workspace_correction[0],
            filtered_workspace_correction[1],
            filtered_workspace_correction[2],
            arm_ready_state,
            arm_error[0],
            arm_error[1],
            arm_error[2] if len(arm_error) > 2 else float("nan"),
        )

        rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
