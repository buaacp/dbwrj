#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
from multirotor_communication import Communication  
import threading
import math
import csv
import os
from datetime import datetime
import sys
import atexit

# 定义最大速度和 PD 控制系数
MAX_VELOCITY = 1
PD = 2

def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def optional_degrees_param(name):
    value = rospy.get_param(name, None)
    if value is None:
        return None
    if isinstance(value, str):
        value = value.strip()
        if value == '' or value.lower() in ('none', 'null', 'nan'):
            return None
    return math.radians(float(value))

def mission_target_name(mission_state):
    if mission_state == 1:
        return 'ball'
    if mission_state == 2:
        return 'bulb'
    if mission_state == 3:
        return 'socket'
    return None

def mission_yaw_name(mission_state):
    if mission_state == 0:
        return 'hover'
    return mission_target_name(mission_state)

def select_target_yaw(communication, target_name, target_yaws, fallback_yaw,
                      face_target, distance_xy, delta_x, delta_y):
    yaw_name = mission_yaw_name(communication.mission_state)
    if yaw_name in target_yaws and target_yaws[yaw_name] is not None:
        return normalize_angle(target_yaws[yaw_name])

    if fallback_yaw is not None:
        return normalize_angle(fallback_yaw)

    current_x = communication.current_position.x
    current_y = communication.current_position.y
    if target_name is not None and face_target:
        object_pose = communication.target_poses[target_name]
        return normalize_angle(math.atan2(object_pose.position.y - current_y,
                                          object_pose.position.x - current_x))

    if distance_xy > 0.3:
        return normalize_angle(math.atan2(delta_y, delta_x))

    return None

def wait_for_condition(description, predicate, timeout):
    deadline = rospy.Time.now() + rospy.Duration(timeout)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown() and rospy.Time.now() < deadline:
        if predicate():
            return True
        rate.sleep()
    rospy.logerr("等待%s超时，当前 mavros_state: connected=%s armed=%s mode=%s",
                 description,
                 communication_state_value(predicate, 'connected'),
                 communication_state_value(predicate, 'armed'),
                 communication_state_value(predicate, 'mode'))
    return False

def communication_state_value(predicate, field):
    communication = getattr(predicate, 'communication', None)
    if communication is None:
        return None
    return getattr(communication.mavros_state, field, None)

def bind_condition(communication, condition):
    condition.communication = communication
    return condition

def set_offboard_mode(communication, max_attempts=5):
    for attempt_count in range(1, max_attempts + 1):
        try:
            res = communication.flightModeService(base_mode=0, custom_mode='OFFBOARD')
        except rospy.ServiceException as e:
            rospy.logwarn("切换 OFFBOARD 服务调用失败: %s", e)
            res = None

        if communication.mavros_state.mode == 'OFFBOARD' or (res is not None and res.mode_sent):
            rospy.loginfo("飞行模式已切换为OFFBOARD")
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

def main():
    rospy.init_node('arm_and_set_mode')
    vehicle_type = 'iris'
    vehicle_id = '0'
    communication = Communication(vehicle_type, vehicle_id)

    pd_gain = rospy.get_param('~pd_gain', PD)
    default_height = rospy.get_param('~default_height', 2.0)
    target_height_offset = rospy.get_param('~target_height_offset', 0.36)
    default_max_velocity = rospy.get_param('~default_max_velocity', 1.0)
    target_max_velocity = rospy.get_param('~target_max_velocity', 0.2)
    approach_angle_deg = rospy.get_param('~approach_angle_deg', 0.0)
    approach_distance = rospy.get_param('~approach_distance', 0.0)
    face_target = rospy.get_param('~face_target', True)
    fixed_target_yaw = optional_degrees_param('~target_yaw_deg')
    target_yaws = {
        'hover': optional_degrees_param('~hover_yaw_deg'),
        'ball': optional_degrees_param('~ball_yaw_deg'),
        'bulb': optional_degrees_param('~bulb_yaw_deg'),
        'socket': optional_degrees_param('~socket_yaw_deg'),
    }
    approach_angle = math.radians(approach_angle_deg)
    communication.default_pose.position.z = default_height

    # 获取当前脚本的上一级目录
    current_script_path = os.path.abspath(__file__)
    parent_directory = os.path.dirname(os.path.dirname(current_script_path))
    
    # 定义CSV文件的路径
    LOG_ENABLED = False  # 将此设置为 False 以禁用日志记录
    log_directory = os.path.join(parent_directory, "data_logs")
    if LOG_ENABLED:
        if not os.path.exists(log_directory):
            os.makedirs(log_directory)  # 确保目录存在

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_filename = os.path.join(log_directory, "position_velocity_log_{}.csv".format(timestamp_str))

        # 打开CSV文件并写入表头，并保持文件打开状态以便持续写入
        # 在 Python 2 中，使用 'wb' 模式并处理 Unicode
        try:
            csv_file = open(csv_filename, mode='wb')
        except IOError as e:
            rospy.logerr("无法打开CSV文件进行写入: {}".format(e))
            csv_file = None

        if csv_file:
            fieldnames = [
                'elapsed_time',  # 自实验开始以来的时间（秒，精确到0.001）
                'current_x', 'current_y', 'current_z',
                'current_vx', 'current_vy', 'current_vz',
                'control_vx', 'control_vy', 'control_vz'
            ]
            writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
            writer.writeheader()

            # 注册退出时关闭文件
            def close_csv():
                if csv_file:
                    csv_file.close()
                    rospy.loginfo("CSV文件已关闭。")

            atexit.register(close_csv)
        else:
            LOG_ENABLED = False  # 如果无法打开文件，禁用日志记录
            rospy.logwarn("日志记录已禁用，因为无法打开CSV文件。")
    else:
        writer = None  # 如果日志记录禁用，保持writer为None

    # 启动发送控制指令的线程
    communication_thread = threading.Thread(target=communication.start)
    communication_thread.start()

    # 等待服务可用
    try:
        rospy.wait_for_service('/iris_0/mavros/cmd/arming', timeout=30)
        rospy.wait_for_service('/iris_0/mavros/set_mode', timeout=30)
    except rospy.ROSException as e:
        rospy.logerr("等待服务超时: {}".format(e))
        if LOG_ENABLED and csv_file:
            csv_file.close()
        return

    # Wait for PX4/MAVROS local position. Gazebo ground truth is only an
    # auxiliary target/vision stream and must not be treated as FCU state.
    if not wait_for_condition(
            "MAVROS连接",
            bind_condition(communication, lambda: communication.mavros_state.connected),
            30.0):
        return
    if not wait_for_condition(
            "PX4本地位置反馈",
            bind_condition(communication, lambda: communication.has_local_position),
            30.0):
        return
    rospy.loginfo("PX4本地位置反馈已就绪，继续启动 OFFBOARD 控制。")
    if communication.vision_pose is None:
        rospy.logwarn("尚未收到 Gazebo 视觉位姿桥接；mission_state=0 悬停起飞不受影响。")

    # 设置初始目标为当前位置，防止无人机突然移动
    communication.coordinate_frame = 1  # ENU坐标系
    communication.motion_type = 1  # 速度控制
    communication.target_motion = communication.construct_target(
        vx=0.0,
        vy=0.0,
        vz=0.0,
        yaw=communication.current_yaw
    )

    # 记录实验开始时间
    start_time = rospy.Time.now()

    # PX4 OFFBOARD 需要先持续收到 setpoint，再切 OFFBOARD，最后解锁。
    rospy.loginfo("预发布 OFFBOARD setpoint，等待 PX4 接收控制目标...")
    rospy.sleep(3.0)

    if not arm_vehicle(communication):
        rospy.logerr("飞机解锁失败，超过最大尝试次数，退出程序。")
        if LOG_ENABLED and csv_file:
            csv_file.close()
        return

    if not set_offboard_mode(communication):
        rospy.logerr("飞行模式切换失败，退出程序。")
        if LOG_ENABLED and csv_file:
            csv_file.close()
        return

    # 设置目标位置
    target_yaw = communication.current_yaw  # 保持当前航向

    rospy.loginfo("开始速度控制")
    rospy.loginfo("目标话题: ball=%s, bulb=%s, socket=%s",
                  communication.ball_target_topic,
                  communication.bulb_target_topic,
                  communication.socket_target_topic)
    rospy.loginfo("到达方位角: %.2f deg, 距离目标: %.2f m", approach_angle_deg, approach_distance)
    rospy.loginfo("期望航向 deg: global=%s, hover=%s, ball=%s, bulb=%s, socket=%s",
                  rospy.get_param('~target_yaw_deg', ''),
                  rospy.get_param('~hover_yaw_deg', ''),
                  rospy.get_param('~ball_yaw_deg', ''),
                  rospy.get_param('~bulb_yaw_deg', ''),
                  rospy.get_param('~socket_yaw_deg', ''))
    rospy.loginfo("mission_state: 0 悬停点, 1 小球, 2 灯泡, 3 灯座")

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        if communication.current_position is not None:
            target_name = mission_target_name(communication.mission_state)

            if communication.mission_state == 0:
                target_x = communication.default_pose.position.x
                target_y = communication.default_pose.position.y
                target_z = communication.default_pose.position.z
                MAX_VELOCITY = default_max_velocity
            elif target_name is not None:
                # 计算期望位置
                object_pose = communication.target_poses[target_name]
                object_x = object_pose.position.x
                object_y = object_pose.position.y
                object_z = object_pose.position.z
                target_x = object_x + approach_distance * math.cos(approach_angle)
                target_y = object_y + approach_distance * math.sin(approach_angle)
                target_z = object_z + target_height_offset
                MAX_VELOCITY = target_max_velocity
            else:
                rospy.logwarn_throttle(2.0, "未知 mission_state=%s，保持默认悬停目标", communication.mission_state)
                target_x = communication.default_pose.position.x
                target_y = communication.default_pose.position.y
                target_z = communication.default_pose.position.z
                MAX_VELOCITY = default_max_velocity

            # Use PX4/MAVROS local position for the UAV feedback loop. The
            # Gazebo vision pose is an external measurement stream and can jump
            # when EKF origin/fusion changes.
            current_x = communication.current_position.x
            current_y = communication.current_position.y
            current_z = communication.current_position.z

            delta_x = target_x - current_x
            delta_y = target_y - current_y
            delta_z = target_z - current_z

            distance_xy = math.sqrt(delta_x*delta_x+delta_y*delta_y)
            selected_yaw = select_target_yaw(
                communication,
                target_name,
                target_yaws,
                fixed_target_yaw,
                face_target,
                distance_xy,
                delta_x,
                delta_y
            )
            if selected_yaw is not None:
                target_yaw = selected_yaw
            target_yaw = normalize_angle(target_yaw)


            control_vx = delta_x * pd_gain
            control_vy = delta_y * pd_gain
            control_vz = delta_z * pd_gain

            # 计算速度的大小
            speed = math.sqrt(control_vx**2 + control_vy**2)

            # 如果速度超出最大速度，进行限幅
            if speed > MAX_VELOCITY:
                scale = MAX_VELOCITY / speed
                control_vx *= scale
                control_vy *= scale
            min_speed = -0.5  # 下限
            max_speed = 0.5   # 上限
            control_vz = max(min(control_vz, max_speed), min_speed)

            communication.target_motion = communication.construct_target(
                vx=control_vx,
                vy=control_vy,
                vz=control_vz,
                yaw=target_yaw
            )
            yaw_error = normalize_angle(target_yaw - communication.current_yaw)
            rospy.loginfo_throttle(
                1.0,
                "目标位置=(%.2f, %.2f, %.2f), 位置误差=(%.2f, %.2f, %.2f), 期望航向=%.1f deg, 当前航向=%.1f deg, 航向误差=%.1f deg",
                target_x, target_y, target_z,
                delta_x, delta_y, delta_z,
                math.degrees(target_yaw),
                math.degrees(communication.current_yaw),
                math.degrees(yaw_error)
            )


            if LOG_ENABLED and writer:
                 # 获取当前时间戳（自实验开始以来的时间，秒，精确到0.001）
                current_time = communication.current_position_time  # 假设这是 rospy.Time 对象
                elapsed_duration = current_time - start_time
                elapsed_time = round(elapsed_duration.to_sec(), 3)  # 精确到0.001秒

                # 获取当前速度
                if communication.current_velocity is not None:
                    current_vx = communication.current_velocity.twist.linear.x
                    current_vy = communication.current_velocity.twist.linear.y
                    current_vz = communication.current_velocity.twist.linear.z
                else:
                    current_vx = 0.0
                    current_vy = 0.0
                    current_vz = 0.0
                # 写入CSV
                row = {
                    'elapsed_time': '%.3f' % elapsed_time,  # 格式化为字符串，保留3位小数
                    'current_x': current_x,
                    'current_y': current_y,
                    'current_z': current_z,
                    'current_vx': current_vx,
                    'current_vy': current_vy,
                    'current_vz': current_vz,
                    'control_vx': control_vx,
                    'control_vy': control_vy,
                    'control_vz': control_vz
                }

                # 在 Python 2 中，必须将所有字符串编码为 UTF-8
                row_encoded = {}
                for key, value in row.items():
                    if isinstance(value, unicode):
                        row_encoded[key] = value.encode('utf-8')
                    else:
                        row_encoded[key] = value  # '%.3f' % elapsed_time 已经是字符串

                try:
                    writer.writerow(row_encoded)
                except Exception as e:
                    rospy.logerr("写入CSV时发生错误: {}".format(e))

        rate.sleep()

    # 在程序退出前关闭CSV文件
    if LOG_ENABLED and csv_file:
        csv_file.close()
        rospy.loginfo("CSV文件已关闭。")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
