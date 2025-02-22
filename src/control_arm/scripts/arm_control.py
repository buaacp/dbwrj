#!/usr/bin/env python
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import os
import sys
import time
import serial
import threading

# 添加当前脚本所在目录到sys.path
current_dir = os.path.dirname(__file__)
sys.path.append(current_dir)

import ARM
import config

# 参数配置
SERVO_PORT_NAME = '/dev/ttyUSB0'  # 舵机串口号
SERVO_BAUDRATE = 115200  # 舵机的波特率
SERVO_IDS = [0, 1, 2, 3]  # 云台的舵机的ID号列表

GIMBAL_TYPE =1
ARM_TYPE = 0
IF_SIMULATION = 1


def create_serial_port(self,port_name,SERVO_BAUDRATE=115200):
    try:
        # 尝试打开串口
        uart = serial.Serial(port=port_name, baudrate=SERVO_BAUDRATE,
                            parity=serial.PARITY_NONE, stopbits=1,
                            bytesize=8, timeout=0)  # 设置串口超时，避免阻塞
        print(f"成功打开串口: {port_name}")
        return uart
    except serial.SerialException as e:
        print(f"无法打开串口 {port_name}: {e}")
        return None
    

def query_servos_continuously():
    rospy.Subscriber("/joint_states", JointState, arm.joint_states_callback)
    rospy.spin()


def arm_control():
    angel_target = [0,0,0,0]
    angular = [0,0,0,0]
    for i in SERVO_IDS:
        angular[i] = -0.2*(angel_target[i]-arm.angle[i])
    print("关节角度：",arm.angle)
    print("关节控制速度：",angular)
    return angular


if __name__ == '__main__':
    try:
        rospy.init_node('arm_control', anonymous=True)
        rate_control = rospy.Rate(5)
        if not IF_SIMULATION:
            # 尝试创建串口
            uart = create_serial_port(SERVO_PORT_NAME)
            if uart is None:
                uart = create_serial_port('/dev/ttyUSB1')
        else:
            uart = None
        arm = ARM.ARM(uart=uart, SERVO_IDS=SERVO_IDS,if_simulation=IF_SIMULATION)
        # 创建并启动查询舵机角度的线程
        angle_thread = threading.Thread(target=query_servos_continuously, daemon=True)
        angle_thread.start()
        pub_angular = rospy.Publisher('/le_arm_controller/command', Float64MultiArray, queue_size=10)
        time.sleep(1)
        while not rospy.is_shutdown():
            angular = arm_control()
            msg_angular = Float64MultiArray()
            msg_angular.data = angular  # 设置数据部分
            pub_angular.publish(msg_angular)
            rate_control.sleep()

    except rospy.ROSInterruptException:
        pass