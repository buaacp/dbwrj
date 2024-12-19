#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import rospy
from multirotor_communication import Communication  
import threading
import math
# 定义最大速度
MAX_VELOCITY = 3.0
PD = 2

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

    # 设置目标位置
    target_x = 0.0
    target_y = 0.0 
    target_z = 2.0 
    target_yaw = communication.current_yaw  # 保持当前航向


    print("开始速度控制")

    rate = rospy.Rate(10)  # 10Hz
    while not rospy.is_shutdown():
        if communication.current_position is not None:
            current_x = communication.current_position.x
            current_y = communication.current_position.y
            current_z = communication.current_position.z
        # if communication.vision_pose is not None:
        #     current_x = communication.vision_pose.x
        #     current_y = communication.vision_pose.y
        #     current_z = communication.vision_pose.z

            control_vx = (target_x - current_x)*PD
            control_vy = (target_y - current_y)*PD
            control_vz = (target_z - current_z)*PD
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
            # 判断是否达到目标位置
            if (abs(target_x - current_x) < 0.1 and
                abs(target_y - current_y) < 0.1 and
                abs(target_z - current_z) < 0.1):
                print("已达到目标位置")
        rate.sleep()

if __name__ == '__main__':
    main()
