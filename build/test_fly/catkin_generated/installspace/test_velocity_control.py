#!/usr/bin/env python2
# -*- coding: UTF-8 -*-

import rospy
from multirotor_communication import Communication  
import threading

def main():
    rospy.init_node('arm_and_set_mode')
    vehicle_type = 'iris'
    vehicle_id = '0'
    communication = Communication(vehicle_type, vehicle_id)

    # 启动发送控制指令的线程
    communication_thread = threading.Thread(target=communication.start)
    communication_thread.start()

    # 等待服务可用
    rospy.wait_for_service('/iris_0/mavros/cmd/arming')
    rospy.wait_for_service('/iris_0/mavros/set_mode')

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

    # 开始发送控制指令，等待一段时间确保飞控接收到
    rospy.sleep(1)

    # 解锁飞机，并进行反复尝试直到成功
    arm_successful = False
    attempt_count = 0
    max_attempts = 5  # 最大尝试次数

    while not arm_successful and attempt_count < max_attempts:
        if communication.arm():
            print("飞机已解锁")
            arm_successful = True
        else:
            attempt_count += 1
            print("飞机解锁失败，正在尝试第{}次...".format(attempt_count))
            rospy.sleep(2)  # 等待2秒后重试

    if not arm_successful:
        print("飞机解锁失败，超过最大尝试次数，退出程序。")
        return

    # 切换飞行模式为OFFBOARD
    if communication.flightModeService(base_mode=0, custom_mode='OFFBOARD'):
        print("飞行模式已切换为OFFBOARD")
    else:
        print("飞行模式切换失败")
        return

    # 设置目标速度，例如向前移动1米/秒，保持高度
    target_vx = 1.0  # 前向速度 (m/s)
    target_vy = 0.0  # 左向速度 (m/s)
    target_vz = 0.0  # 上升/下降速度 (m/s)
    target_yaw = communication.current_yaw  # 保持当前航向

    communication.target_motion = communication.construct_target(
        vx=target_vx,
        vy=target_vy,
        vz=target_vz,
        yaw=target_yaw
    )

    print("开始速度控制，目标速度：vx={:.2f} m/s, vy={:.2f} m/s, vz={:.2f} m/s".format(target_vx, target_vy, target_vz))

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        # 打印当前速度
        if communication.current_velocity is not None:
            current_vx = communication.current_velocity.x
            current_vy = communication.current_velocity.y
            current_vz = communication.current_velocity.z
            print("当前速度：vx={:.2f} m/s, vy={:.2f} m/s, vz={:.2f} m/s".format(current_vx, current_vy, current_vz))
            # 判断是否达到目标速度（可以根据具体需求调整阈值）
            if (abs(current_vx - target_vx) < 0.1 and
                abs(current_vy - target_vy) < 0.1 and
                abs(current_vz - target_vz) < 0.1):
                print("已达到目标速度")
                break
        rate.sleep()

    print("速度控制过程结束")

if __name__ == '__main__':
    main()
