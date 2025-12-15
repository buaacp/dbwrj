#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
CAMERA.py
"""
import math

class NeighborCameraInfo:
    def __init__(self):
        self.UAV_OBJ_X = 0
        self.UAV_OBJ_Y = 0
        self.latitude = 0
        self.longitude = 0
        self.altitude = 0

class Mycamera:
    def __init__(self):
        self.CAMERA_ID = 0
        self.pixel_x = 0
        self.pixel_y = 0
        self.UAV_OBJ_X = 0
        self.UAV_OBJ_Y = 0
        self.latitude = 37.7749
        self.longitude = 122.4194
        self.altitude = 1

        # 邻居无人机信息
        self.camera_neighbor = [NeighborCameraInfo() for _ in range(UAV_num)]

        # 云台信息
        self.angle_0 = 0
        self.angle_1 = 0
        self.angle_2 = 0

        # 无人机姿态信息
        self.UAV_x = 0
        self.UAV_y = 0
        self.UAV_z = 0
        self.UAV_w = 1

    def relative_altitude_info_callback(self, msg):
        self.altitude = msg.data

    def camera_detect_callback(self, msg):
        self.pixel_x = msg.pixel_x
        self.pixel_y = msg.pixel_y


    def attitude_info_callback(self,msg):
        quaternion = [msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z]
        roll,pitch,yaw = self.quaternion_to_euler(quaternion)
        roll_new = -pitch
        pitch_new = roll
        yaw_new = yaw
        w,x,y,z = self.euler_to_quaternion(roll_new,pitch_new,yaw_new)
        self.UAV_x = x
        self.UAV_y = y
        self.UAV_z = z
        self.UAV_w = w

    def global_position_callback(self,msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude


    def quaternion_to_euler(self, quaternion):
        """
        将四元数转换为欧拉角

        参数：
        quaternion: 4个元素的四元数 [w, x, y, z]

        返回：
        欧拉角 [roll, pitch, yaw]，单位为弧度
        """
        w, x, y, z = quaternion  # 修正四元数解包顺序
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x**2 + y**2)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = min(max(t2, -1.0), 1.0)  # 钳位以避免数值错误
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y**2 + z**2)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # 返回弧度值

    def euler_to_quaternion(self, roll, pitch, yaw):
        """
        将欧拉角转换为四元数

        参数：
        roll: 绕 x 轴旋转角度，单位为弧度
        pitch: 绕 y 轴旋转角度，单位为弧度
        yaw: 绕 z 轴旋转角度，单位为弧度

        返回：
        四元数 [w, x, y, z]
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        w = cy * cp * cr + sy * sp * sr
        x = cy * cp * sr - sy * sp * cr
        y = sy * cp * sr + cy * sp * cr
        z = sy * cp * cr - cy * sp * sr

        return w, x, y, z  # 返回四元数 [w, x, y, z]
# 无人机数量
UAV_num = 3

