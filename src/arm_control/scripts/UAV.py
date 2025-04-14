#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
"""
CAMERA.py
"""
import math

class Pose:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class velocity:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class angular:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
class Myuav:
    def __init__(self):
        self.UAV_ID = 0
        self.altitude = 10
        self.latitude = 0
        self.longitude = 0

        # local
        self.pose = Pose()
        self.vision_pose = Pose()
        self.velocity = velocity()
        self.angular = angular()

        # 无人机姿态信息
        self.x = 0
        self.y = 0
        self.z = 0
        self.w = 1

        self.pitch = 0
        self.roll = 0
        self.yaw = 0

        self.d_pitch = 0
        self.d_roll = 0
        self.d_yaw = 0

        self.vx = 0
        self.vy = 0
        self.vz = 0

        self.ax = 0
        self.ay = 0
        self.az = 0

        

    def relative_altitude_info_callback(self, msg):
        self.altitude = msg.data

    def pose_callback(self,msg):
        self.pose.x = msg.pose.position.x
        self.pose.y = msg.pose.position.y
        self.pose.z = msg.pose.position.z

    def vision_pose_callback(self,msg):
        self.vision_pose.x = msg.pose.position.x
        self.vision_pose.y = msg.pose.position.y
        self.vision_pose.z = msg.pose.position.z

    def velocity_callback(self,msg):
        self.velocity.x = msg.twist.linear.x
        self.velocity.y = msg.twist.linear.y
        self.velocity.z = msg.twist.linear.z

        self.angular.x = -msg.twist.angular.y
        self.angular.y = msg.twist.angular.x
        self.angular.z = msg.twist.angular.z

    def attitude_info_callback(self,msg):
        quaternion = [msg.orientation.w,msg.orientation.x,msg.orientation.y,msg.orientation.z]
        roll,pitch,yaw = self.quaternion_to_euler(quaternion)
        self.pitch = -pitch
        self.roll = roll
        self.yaw = yaw-90
        if self.yaw<-180:
            self.yaw = self.yaw+360
        eular = [self.roll,self.pitch,self.yaw]
        w,x,y,z = self.euler_to_quaternion(eular)
        self.x = x
        self.y = y
        self.z = z
        self.w = w
    def vision_imu_callback(self,msg):
        quaternion = [msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z]
        roll,pitch,yaw = self.quaternion_to_euler(quaternion)
        self.pitch = -pitch
        self.roll = roll
        self.yaw = yaw-90
        if self.yaw<-180:
            self.yaw = self.yaw+360
        eular = [self.roll,self.pitch,self.yaw]
        w,x,y,z = self.euler_to_quaternion(eular)
        self.x = x
        self.y = y
        self.z = z
        self.w = w






    def global_position_callback(self,msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def quaternion_to_euler(self,quaternion):
        """
        将四元数转换为欧拉角

        参数：
        quaternion: 4个元素的四元数 [w, x, y, z]

        返回：
        欧拉角 [roll, pitch, yaw]，单位为弧度
        """
        w, x, y, z= quaternion
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x**2 + y**2)
        roll = math.atan2(t0, t1)*180/math.pi

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)*180/math.pi

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y**2 + z**2)
        yaw = math.atan2(t3, t4)*180/math.pi

        return roll, pitch, yaw

    def euler_to_quaternion(self,euler):
        """
        将欧拉角转换为四元数

        参数：
        euler: 3个元素的欧拉角 [roll, pitch, yaw]，单位为弧度

        返回：
        四元数 [w, x, y, z]
        """
        roll, pitch, yaw = euler

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

        return x, y, z, w

