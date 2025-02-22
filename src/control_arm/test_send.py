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



# 参数配置
SERVO_PORT_NAME = '/dev/ttyUSB0'  # 舵机串口号
SERVO_BAUDRATE = 115200  # 舵机的波特率
SERVO_IDS = [0, 1, 2, 3]  # 云台的舵机的ID号列表

GIMBAL_TYPE =1
ARM_TYPE = 0
IF_SIMULATION = 1


if __name__ == '__main__':
    try:
        rospy.init_node('arm_control', anonymous=True)
        rate_control = rospy.Rate(5)
        pub_angular = rospy.Publisher('/le_arm_controller/command', Float64MultiArray, queue_size=10)
        time.sleep(1)
        while not rospy.is_shutdown():
            angular = [0.01,0.02,0.03,0.04]
            msg_angular = Float64MultiArray()
            msg_angular.data = angular  # 设置数据部分
            pub_angular.publish(msg_angular)
            rate_control.sleep()

    except rospy.ROSInterruptException:
        pass