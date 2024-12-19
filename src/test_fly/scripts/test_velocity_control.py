#!/usr/bin/env python
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
MAX_VELOCITY = 3.0
PD = 2


def main():
    rospy.init_node('arm_and_set_mode')
    vehicle_type = 'iris'
    vehicle_id = '0'
    communication = Communication(vehicle_type, vehicle_id)

    # 获取当前脚本的上一级目录
    current_script_path = os.path.abspath(__file__)
    parent_directory = os.path.dirname(os.path.dirname(current_script_path))
    
    # 定义CSV文件的路径
    LOG_ENABLED = True  # 将此设置为 False 以禁用日志记录
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

    # 等待获取当前位置信息
    while communication.current_position is None and not rospy.is_shutdown():
        rospy.sleep(0.1)

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

    # 开始发送控制指令，等待一段时间确保飞控接收到
    rospy.sleep(1)

    # 解锁飞机，并进行反复尝试直到成功
    arm_successful = False
    attempt_count = 0
    max_attempts = 5  # 最大尝试次数

    while not arm_successful and attempt_count < max_attempts and not rospy.is_shutdown():
        if communication.arm():
            rospy.loginfo("飞机已解锁")
            arm_successful = True
        else:
            attempt_count += 1
            rospy.logwarn("飞机解锁失败，正在尝试第{}次...".format(attempt_count))
            rospy.sleep(2)  # 等待2秒后重试

    if not arm_successful:
        rospy.logerr("飞机解锁失败，超过最大尝试次数，退出程序。")
        if LOG_ENABLED and csv_file:
            csv_file.close()
        return

    # 切换飞行模式为OFFBOARD
    if communication.flightModeService(base_mode=0, custom_mode='OFFBOARD'):
        rospy.loginfo("飞行模式已切换为OFFBOARD")
    else:
        rospy.logerr("飞行模式切换失败")
        if LOG_ENABLED and csv_file:
            csv_file.close()
        return

    # 设置目标位置
    target_x = 0.0
    target_y = 1.0 
    target_z = 2.0 
    target_yaw = communication.current_yaw  # 保持当前航向

    rospy.loginfo("开始速度控制")

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        if communication.current_position is not None and communication.current_velocity is not None:
            current_x = communication.current_position.x
            current_y = communication.current_position.y
            current_z = communication.current_position.z

            control_vx = (target_x - current_x) * PD
            control_vy = (target_y - current_y) * PD
            control_vz = (target_z - current_z) * PD

            # 计算速度的大小
            speed = math.sqrt(control_vx**2 + control_vy**2 + control_vz**2)

            # 如果速度超出最大速度，进行限幅
            if speed > MAX_VELOCITY:
                scale = MAX_VELOCITY / speed
                control_vx *= scale
                control_vy *= scale
                control_vz *= scale

            communication.target_motion = communication.construct_target(
                vx=control_vx,
                vy=control_vy,
                vz=control_vz,
                yaw=target_yaw
            )

            # 获取当前时间戳（自实验开始以来的时间，秒，精确到0.001）
            current_time = communication.current_position_time  # 假设这是 rospy.Time 对象
            elapsed_duration = current_time - start_time
            elapsed_time = round(elapsed_duration.to_sec(), 3)  # 精确到0.001秒

            # 获取当前速度
            current_vx = communication.current_velocity.twist.linear.x
            current_vy = communication.current_velocity.twist.linear.y
            current_vz = communication.current_velocity.twist.linear.z

            if LOG_ENABLED and writer:
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