#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import atexit
import csv
import math
import os
import threading
from datetime import datetime

import rospy

from multirotor_communication import Communication


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def wait_for_condition(description, predicate, timeout, communication=None):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if predicate():
            return True
        rate.sleep()

    if communication is not None:
        rospy.logerr(
            "等待%s超时，mavros_state: connected=%s armed=%s mode=%s",
            description,
            communication.mavros_state.connected,
            communication.mavros_state.armed,
            communication.mavros_state.mode,
        )
    else:
        rospy.logerr("等待%s超时", description)
    return False


def set_offboard_mode(communication, max_attempts=5):
    for attempt_count in range(1, max_attempts + 1):
        try:
            res = communication.flightModeService(base_mode=0, custom_mode="OFFBOARD")
        except rospy.ServiceException as exc:
            rospy.logwarn("切换 OFFBOARD 服务调用失败: %s", exc)
            res = None

        if communication.mavros_state.mode == "OFFBOARD" or (res is not None and res.mode_sent):
            rospy.loginfo("飞行模式已切换为 OFFBOARD")
            return True

        rospy.logwarn("飞行模式切换失败，正在尝试第 %d 次...", attempt_count)
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

        rospy.logwarn("飞机解锁失败，正在尝试第 %d 次...", attempt_count)
        rospy.sleep(2.0)
    return False


class ErrorStats:
    def __init__(self):
        self.count = 0
        self.sum_pos = 0.0
        self.sum_pos_sq = 0.0
        self.max_pos = 0.0
        self.sum_vel = 0.0
        self.sum_vel_sq = 0.0
        self.max_vel = 0.0

    def add(self, pos_error, vel_error):
        self.count += 1
        self.sum_pos += pos_error
        self.sum_pos_sq += pos_error * pos_error
        self.max_pos = max(self.max_pos, pos_error)
        self.sum_vel += vel_error
        self.sum_vel_sq += vel_error * vel_error
        self.max_vel = max(self.max_vel, vel_error)

    def summary(self):
        if self.count == 0:
            return {
                "samples": 0,
                "mean_pos": 0.0,
                "rmse_pos": 0.0,
                "max_pos": 0.0,
                "mean_vel": 0.0,
                "rmse_vel": 0.0,
                "max_vel": 0.0,
            }
        return {
            "samples": self.count,
            "mean_pos": self.sum_pos / self.count,
            "rmse_pos": math.sqrt(self.sum_pos_sq / self.count),
            "max_pos": self.max_pos,
            "mean_vel": self.sum_vel / self.count,
            "rmse_vel": math.sqrt(self.sum_vel_sq / self.count),
            "max_vel": self.max_vel,
        }


def vector_norm(x, y, z):
    return math.sqrt(x * x + y * y + z * z)


def make_log_writer(log_directory):
    if not os.path.exists(log_directory):
        os.makedirs(log_directory)

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    csv_path = os.path.join(log_directory, "arm_hover_low_speed_error_{}.csv".format(timestamp))
    csv_file = open(csv_path, "w", newline="")
    fieldnames = [
        "elapsed_time",
        "phase",
        "ref_x",
        "ref_y",
        "ref_z",
        "actual_x",
        "actual_y",
        "actual_z",
        "error_x",
        "error_y",
        "error_z",
        "pos_error",
        "ref_vx",
        "ref_vy",
        "ref_vz",
        "actual_vx",
        "actual_vy",
        "actual_vz",
        "vel_error",
        "cmd_vx",
        "cmd_vy",
        "cmd_vz",
        "ref_yaw",
        "actual_yaw",
    ]
    writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    writer.writeheader()

    def close_csv():
        csv_file.close()
        rospy.loginfo("误差日志已保存: %s", csv_path)

    atexit.register(close_csv)
    return writer, csv_file, csv_path


def main():
    rospy.init_node("arm_hover_low_speed_test")

    vehicle_type = rospy.get_param("~vehicle_type", "iris")
    vehicle_id = str(rospy.get_param("~vehicle_id", "0"))
    communication = Communication(vehicle_type, vehicle_id)

    target_height = float(rospy.get_param("~target_height", 2.0))
    hover_duration = float(rospy.get_param("~hover_duration", 20.0))
    low_speed_duration = float(rospy.get_param("~low_speed_duration", 30.0))
    low_speed = float(rospy.get_param("~low_speed", 0.2))
    low_speed_axis = rospy.get_param("~low_speed_axis", "x").lower()
    settle_duration = float(rospy.get_param("~settle_duration", 8.0))
    pd_gain_xy = float(rospy.get_param("~pd_gain_xy", 1.5))
    pd_gain_z = float(rospy.get_param("~pd_gain_z", 1.8))
    max_xy_speed = float(rospy.get_param("~max_xy_speed", 0.45))
    max_z_speed = float(rospy.get_param("~max_z_speed", 0.35))
    control_rate_hz = float(rospy.get_param("~control_rate", 20.0))
    log_enabled = bool(rospy.get_param("~log_enabled", True))
    hold_after_test = bool(rospy.get_param("~hold_after_test", True))

    current_script_path = os.path.abspath(__file__)
    package_dir = os.path.dirname(os.path.dirname(current_script_path))
    default_log_dir = os.path.join(package_dir, "data_logs")
    log_directory = rospy.get_param("~log_directory", default_log_dir)

    writer = None
    csv_file = None
    if log_enabled:
        writer, csv_file, csv_path = make_log_writer(log_directory)
        rospy.loginfo("误差日志文件: %s", csv_path)

    communication_thread = threading.Thread(target=communication.start)
    communication_thread.daemon = True
    communication_thread.start()

    service_prefix = "/{}_{}".format(vehicle_type, vehicle_id)
    try:
        rospy.wait_for_service(service_prefix + "/mavros/cmd/arming", timeout=30)
        rospy.wait_for_service(service_prefix + "/mavros/set_mode", timeout=30)
    except rospy.ROSException as exc:
        rospy.logerr("等待 MAVROS 服务超时: %s", exc)
        return

    if not wait_for_condition(
        "MAVROS连接",
        lambda: communication.mavros_state.connected,
        30.0,
        communication,
    ):
        return

    if not wait_for_condition(
        "MAVROS本地位置",
        lambda: communication.current_position is not None and communication.current_velocity is not None,
        30.0,
        communication,
    ):
        return

    start_x = communication.current_position.x
    start_y = communication.current_position.y
    start_z = target_height
    ref_yaw = communication.current_yaw

    communication.coordinate_frame = 1
    communication.motion_type = 1
    communication.target_motion = communication.construct_target(
        vx=0.0,
        vy=0.0,
        vz=0.0,
        yaw=ref_yaw,
    )

    rospy.loginfo("预发布 OFFBOARD setpoint，等待 PX4 接收控制目标...")
    rospy.sleep(3.0)

    if not set_offboard_mode(communication):
        rospy.logerr("飞行模式切换失败，退出测试。")
        return

    if not arm_vehicle(communication):
        rospy.logerr("飞机解锁失败，退出测试。")
        return

    axis_sign = 1.0
    if low_speed_axis.startswith("-"):
        axis_sign = -1.0
        low_speed_axis = low_speed_axis[1:]
    if low_speed_axis not in ("x", "y"):
        rospy.logwarn("未知 low_speed_axis=%s，使用 x 方向", low_speed_axis)
        low_speed_axis = "x"

    phases = [
        ("settle", settle_duration, 0.0, 0.0),
        ("hover", hover_duration, 0.0, 0.0),
    ]
    if low_speed_axis == "x":
        phases.append(("low_speed_x", low_speed_duration, axis_sign * low_speed, 0.0))
    else:
        phases.append(("low_speed_y", low_speed_duration, 0.0, axis_sign * low_speed))

    stats = {
        "hover": ErrorStats(),
        "low_speed": ErrorStats(),
    }
    rate = rospy.Rate(control_rate_hz)
    test_start = rospy.Time.now()
    ref_x = start_x
    ref_y = start_y
    ref_z = start_z

    rospy.loginfo(
        "开始带臂无人机误差测试: hover=%.1fs, low_speed=%.2fm/s %.1fs, z=%.2fm",
        hover_duration,
        low_speed,
        low_speed_duration,
        target_height,
    )

    for phase_name, duration, ref_vx, ref_vy in phases:
        phase_start = rospy.Time.now()
        last_time = phase_start
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            phase_elapsed = (now - phase_start).to_sec()
            if phase_elapsed >= duration:
                break

            dt = max(0.0, (now - last_time).to_sec())
            last_time = now
            ref_x += ref_vx * dt
            ref_y += ref_vy * dt

            if communication.current_position is None or communication.current_velocity is None:
                rate.sleep()
                continue

            actual_x = communication.current_position.x
            actual_y = communication.current_position.y
            actual_z = communication.current_position.z
            actual_vx = communication.current_velocity.twist.linear.x
            actual_vy = communication.current_velocity.twist.linear.y
            actual_vz = communication.current_velocity.twist.linear.z

            error_x = ref_x - actual_x
            error_y = ref_y - actual_y
            error_z = ref_z - actual_z

            cmd_vx = ref_vx + pd_gain_xy * error_x
            cmd_vy = ref_vy + pd_gain_xy * error_y
            cmd_vz = pd_gain_z * error_z

            xy_speed = vector_norm(cmd_vx, cmd_vy, 0.0)
            if xy_speed > max_xy_speed:
                scale = max_xy_speed / xy_speed
                cmd_vx *= scale
                cmd_vy *= scale
            cmd_vz = clamp(cmd_vz, -max_z_speed, max_z_speed)

            communication.target_motion = communication.construct_target(
                vx=cmd_vx,
                vy=cmd_vy,
                vz=cmd_vz,
                yaw=ref_yaw,
            )

            pos_error = vector_norm(error_x, error_y, error_z)
            vel_error = vector_norm(ref_vx - actual_vx, ref_vy - actual_vy, 0.0 - actual_vz)
            metric_key = "hover" if phase_name == "hover" else "low_speed" if phase_name.startswith("low_speed") else None
            if metric_key is not None:
                stats[metric_key].add(pos_error, vel_error)

            rospy.loginfo_throttle(
                2.0,
                "%s: pos_err=%.3fm vel_err=%.3fm/s ref=(%.2f, %.2f, %.2f) actual=(%.2f, %.2f, %.2f)",
                phase_name,
                pos_error,
                vel_error,
                ref_x,
                ref_y,
                ref_z,
                actual_x,
                actual_y,
                actual_z,
            )

            if writer is not None:
                writer.writerow(
                    {
                        "elapsed_time": "{:.3f}".format((now - test_start).to_sec()),
                        "phase": phase_name,
                        "ref_x": ref_x,
                        "ref_y": ref_y,
                        "ref_z": ref_z,
                        "actual_x": actual_x,
                        "actual_y": actual_y,
                        "actual_z": actual_z,
                        "error_x": error_x,
                        "error_y": error_y,
                        "error_z": error_z,
                        "pos_error": pos_error,
                        "ref_vx": ref_vx,
                        "ref_vy": ref_vy,
                        "ref_vz": 0.0,
                        "actual_vx": actual_vx,
                        "actual_vy": actual_vy,
                        "actual_vz": actual_vz,
                        "vel_error": vel_error,
                        "cmd_vx": cmd_vx,
                        "cmd_vy": cmd_vy,
                        "cmd_vz": cmd_vz,
                        "ref_yaw": ref_yaw,
                        "actual_yaw": normalize_angle(communication.current_yaw),
                    }
                )
            rate.sleep()

    communication.target_motion = communication.construct_target(
        vx=0.0,
        vy=0.0,
        vz=0.0,
        yaw=ref_yaw,
    )

    for name, phase_stats in stats.items():
        summary = phase_stats.summary()
        rospy.loginfo(
            "%s 误差统计: samples=%d pos_mean=%.4fm pos_rmse=%.4fm pos_max=%.4fm "
            "vel_mean=%.4fm/s vel_rmse=%.4fm/s vel_max=%.4fm/s",
            name,
            summary["samples"],
            summary["mean_pos"],
            summary["rmse_pos"],
            summary["max_pos"],
            summary["mean_vel"],
            summary["rmse_vel"],
            summary["max_vel"],
        )

    if csv_file is not None:
        csv_file.flush()

    if hold_after_test:
        rospy.loginfo("测试结束，保持当前位置附近悬停。Ctrl-C 退出。")
        while not rospy.is_shutdown():
            communication.target_motion = communication.construct_target(
                vx=0.0,
                vy=0.0,
                vz=0.0,
                yaw=ref_yaw,
            )
            rate.sleep()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
