#! /usr/bin/env python3
# -*- coding: UTF-8 -*-
'''
    对于uservo.py进行进一步的打包，方便多个舵机的协同控制
    作者：郑立浩琦 
    更新时间：2024.11.27
'''
from uservo import UartServoManager
import config
import math
import time
import numpy as np
import serial

class Pose:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class ARM:
    def __init__(self, uart, SERVO_IDS,if_simulation):
        """
        初始化多个舵机控制器
        SERVO_IDS: 舵机ID列表，用于管理多个舵机
        """
        self.SERVO_IDS = SERVO_IDS
        # 初始化舵机管理器
        if not if_simulation:
            self.uservo = UartServoManager(uart)
        # 初始化存储舵机角度的字典
        self.angle = {servo_id: None for servo_id in SERVO_IDS}
        self.angular = {servo_id: 0 for servo_id in SERVO_IDS}
        self.effort = {servo_id: 0 for servo_id in SERVO_IDS}
        self.if_simulation = if_simulation
        # 控制模式 0：搜索 1：跟踪 2:安全
        self.control_mode = 0
        self.target_pos = [0,0,-20]
        self.last_update_time = None
        self.altitude = 1
        self.target_pose = Pose()
        self.control_rate = 50
        self.L1 = 0.104
        self.L2 = 0.0884
        self.L3 = 0.10

    def boundary_q(self,d_q):
        q0 = self.angle[0]+d_q[0]/self.control_rate
        q1 = self.angle[1]+d_q[1]/self.control_rate
        q2 = self.angle[2]+d_q[2]/self.control_rate
        q3 = self.angle[3]+d_q[3]/self.control_rate
        if q0>=math.pi/4 or q1<=-math.pi/4:
            q0 = self.angle[0]
            d_q[0]=0
        # 第一关节位置
        point1 = np.array([
            np.sin(-q0) * self.L1 * np.sin(q1),
            np.cos(-q0) * self.L1 * np.sin(q1),
            -self.L1 * np.cos(q1)
        ]).reshape(3, 1)
        if point1[2,0]>=0 or abs(q1)>=80*math.pi/180:
            q1 = self.angle[1]
            d_q[1]=0

        # 第二关节位置
        point2 = point1+np.array([
            np.sin(-q0) * self.L2 * np.sin(q1 + q2),
            np.cos(-q0) * self.L2 * np.sin(q1 + q2),
            -self.L2 * np.cos(q1 + q2)
        ]).reshape(3, 1)
        if point2[2,0]>=0 or abs(q2)>=90*math.pi/180:
            q2 = self.angle[2]
            d_q[2]=0

        # 末端执行器位置
        point3 = point2 + np.array([
            np.sin(-q0) * self.L3 * np.sin(q1 + q2 + q3),
            np.cos(-q0) * self.L3 * np.sin(q1 + q2 + q3),
            -self.L3 * np.cos(q1 + q2 + q3)
        ]).reshape(3, 1)
        if point3[2,0]>=0 or abs(q3)>=90*math.pi/180:
            q3 = self.angle[3]
            d_q[3]=0

        return d_q
        
    def query_servo_angle(self, servo_id):
        """
        查询指定舵机的当前角度
        """
        self.angle[servo_id] = self.uservo.query_servo_angle(servo_id)

    def joint_states_callback(self,data):
        # joint 0 
        self.angle[0] = -1*data.position[3]
        self.angular[0] = -1*data.velocity[3]
        self.effort[0] = -1*data.effort[3]
        # joint 1
        self.angle[1] = -1*data.position[2]+math.pi/4
        self.angular[1] = -1*data.velocity[2]
        self.effort[1] = -1*data.effort[2]
        # joint 2
        self.angle[2] = -1*data.position[0]+math.pi/4
        self.angular[2] = -1*data.velocity[0]
        self.effort[2] = -1*data.effort[0]
        # joint 3
        self.angle[3] = -1*data.position[4]
        self.angular[3] = -1*data.velocity[4]
        self.effort[3] = -1*data.effort[4]

        # for i in range(len(data.velocity)):
        #     self.angle[i] = data.position[i]
        #     self.angular[i] = data.velocity[i]
        #     self.effort[i] = data.effort[i]
    def target_pos_callback(self,msg):
        self.target_pose.x = msg.pose.position.x
        self.target_pose.y = msg.pose.position.y
        self.target_pose.z = msg.pose.position.z
    def query_all_servos(self): 
        """
        查询所有舵机的角度
        """
        for servo_id in self.SERVO_IDS:
            self.query_servo_angle(servo_id)

    def velocity_control_all(self,type,velocity):
        """
        速度控制所有舵机的角度
        0:ARM
        1:GIMBAL
        """
        for servo_id in self.SERVO_IDS:
            self.velocity_control_single(type,servo_id,velocity[servo_id])


    def velocity_control_single(self,type,servo_id,velocity):
            if type == 0:
                angle_lowerb = config.ARM_THETA_LOWERB[servo_id]
                angle_upperb = config.ARM_THETA_UPPERB[servo_id]
            elif type == 1:
                angle_lowerb = config.GIMBAL_THETA_LOWERB[servo_id]
                angle_upperb = config.GIMBAL_THETA_UPPERB[servo_id]

            if velocity>0:
                aim_angle = min(angle_upperb,self.angle+velocity*config.CONTROL_T)
            elif velocity<0:
                aim_angle = max(angle_lowerb,self.angle+velocity*config.CONTROL_T)
                velocity = velocity * -1

            self.uservo.set_servo_angle(servo_id, aim_angle=aim_angle, velocity=velocity, t_acc=20, t_dec=20)
    
    def position_control_all(self,type,position,delay):
        """
        位置控制所有舵机的角度
        0:ARM
        1:GIMBAL
        """
        # 计算每个舵机角度的变化
        angle_differences = [abs(position[i] - self.angle[self.SERVO_IDS[i]]) for i in range(len(self.SERVO_IDS))]
        
        # print(angle_differences)
        # 找出最大的角度变化
        interval = max(angle_differences) * delay  # 乘以缩放因子

        # print("interval: ",interval)

        for servo_id in self.SERVO_IDS:
            self.position_control_single(type,servo_id,position[servo_id],interval)

        return interval/1000
            

    def position_control_single(self,type,servo_id,position,interval):
        if type == 0:
            angle_lowerb = config.ARM_THETA_LOWERB[servo_id]
            angle_upperb = config.ARM_THETA_UPPERB[servo_id]
        elif type == 1:
            angle_lowerb = config.GIMBAL_THETA_LOWERB[servo_id]
            angle_upperb = config.GIMBAL_THETA_UPPERB[servo_id]

        position = max(position,angle_lowerb)
        position = min(position,angle_upperb)


        self.uservo.set_servo_angle(servo_id,position, interval=interval) # 设置舵机角度 极速模式

    

    def get_gimbal_pos(self, dx, dy, dz):
        """
        以北为正，以东为正
        x：东西向
        y：南北向
        输入：目标和云台的相对位置
        """
        
        gimbal_aim_pos = config.GIMBAL_THETA_CENTER.copy()  # 云台的目标位置

        # 计算方向角（theta1），将弧度转换为角度
        theta1 = -1 * math.atan2(dx,dy) * 180.0 / math.pi 

        # 计算俯仰角（theta2），假设你还需要用dz（垂直高度差）来计算
        theta2 = 90 + math.atan2(dz, math.sqrt(dx**2 + dy**2)) * 180.0 / math.pi  # 俯仰角转换为角度

        # print(f"方向角 (theta1): {theta1}°")
        # print(f"俯仰角 (theta2): {theta2}°")

        if theta1 + gimbal_aim_pos[0] > config.GIMBAL_THETA_UPPERB[0] or theta1 + gimbal_aim_pos[0] < config.GIMBAL_THETA_LOWERB[0]:
            theta1 = theta1 + 180
            if theta1 >180:
                theta1 = theta1 -360
            theta2 = theta2*-1

        # 更新云台目标位置
        gimbal_aim_pos[0] = gimbal_aim_pos[0] + theta1
        gimbal_aim_pos[1] = gimbal_aim_pos[1] + theta2

        # 打印结果，查看更新后的云台位置
        # print(f"云台目标位置 (gimbal_aim_pos): {gimbal_aim_pos}")

        return gimbal_aim_pos
    
    def change_mode(self,target_pos):
        self.control_mode = 1
        self.last_update_time = time.time()
        self.target_pos = [target_pos.pos_x,target_pos.pos_y,target_pos.height]

    def relative_altitude_callback(self,msg):
        self.altitude = msg.data
        if(self.altitude<=8):
            self.control_mode = 2
        else:
            if(self.control_mode == 2):
                self.control_mode = 0


    def is_stop(self, type, target_pos):
        """
        type 1: 云台
        """
        if type == 1:
            now_pos = [self.angle[0], self.angle[1]]
            # 使用 NumPy 计算欧几里得距离
            norm_pos = np.linalg.norm(np.array(now_pos) - np.array(target_pos))
            if(norm_pos<=4):
                return True
            else:
                return False
            