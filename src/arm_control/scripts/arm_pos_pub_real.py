#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu,NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped, PointStamped
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
import os
import sys
import time
import serial
import threading
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

# 添加当前脚本所在目录到sys.path
current_dir = os.path.dirname(__file__)
utils_path = os.path.join(current_dir, "utils")
sys.path.append(utils_path)

from utils import ARM, config, common, UAV, UAV_ARM

class ArmForwardKinematics:
    def __init__(self, uav_arm):
        self.uav_arm = uav_arm
        # 创建发布器，发布机械臂末端相对位置[3](@ref)
        self.end_effector_pub = rospy.Publisher('/arm_end_effector/relative_position', PointStamped, queue_size=10)
        self.end_effector_pose_pub = rospy.Publisher('/arm_end_effector/relative_pose', PoseStamped, queue_size=10)
        rospy.loginfo("机械臂末端位置发布器已初始化")

    def calculate_forward_kinematics(self, joint_angles):
        """
        计算机械臂正运动学，返回末端执行器相对于基座的位置[1](@ref)
        """
        if len(joint_angles) < 4:
            rospy.logwarn("关节角度数量不足，需要4个关节角度")
            return None
            
        q0 = joint_angles[0]  # 关节0角度
        q1 = joint_angles[1]  # 关节1角度
        q2 = joint_angles[2]  # 关节2角度
        q3 = joint_angles[3]  # 关节3角度
        
        # 正运动学计算（相对于机械臂基座坐标系）[1](@ref)
        arm_z = -(self.uav_arm.L1 * np.cos(q1) + 
                 self.uav_arm.L2 * np.cos(q1 + q2) + 
                 self.uav_arm.L3 * np.cos(q1 + q2 + q3))
        arm_y = np.cos(q0) * (self.uav_arm.L1 * np.sin(q1) + 
                             self.uav_arm.L2 * np.sin(q1 + q2) + 
                             self.uav_arm.L3 * np.sin(q1 + q2 + q3))
        arm_x = -np.sin(q0) * (self.uav_arm.L1 * np.sin(q1) + 
                              self.uav_arm.L2 * np.sin(q1 + q2) + 
                              self.uav_arm.L3 * np.sin(q1 + q2 + q3))
        
        return np.array([arm_x, arm_y, arm_z])

    def publish_end_effector_position(self, position, frame_id="arm_base"):
        """
        发布末端执行器相对位置[3](@ref)
        """
        # 发布PointStamped消息（纯位置信息）
        point_msg = PointStamped()
        point_msg.header.stamp = rospy.Time.now()
        point_msg.header.frame_id = frame_id
        point_msg.point.x = float(position[0])
        point_msg.point.y = float(position[1])
        point_msg.point.z = float(position[2])
        
        self.end_effector_pub.publish(point_msg)
        
        # 同时发布PoseStamped消息（包含姿态信息，姿态默认为单位四元数）
        pose_msg = PoseStamped()
        pose_msg.header = point_msg.header
        pose_msg.pose.position.x = point_msg.point.x
        pose_msg.pose.position.y = point_msg.point.y
        pose_msg.pose.position.z = point_msg.point.z
        pose_msg.pose.orientation.w = 1.0  # 默认姿态
        
        self.end_effector_pose_pub.publish(pose_msg)
        
        # rospy.logdebug("末端执行器相对位置: x=%.3f, y=%.3f, z=%.3f", 
        #               position[0], position[1], position[2])

def create_serial_port(port_name, SERVO_BAUDRATE=115200):
    try:
        # 尝试打开串口
        uart = serial.Serial(port=port_name, baudrate=SERVO_BAUDRATE,
                            parity=serial.PARITY_NONE, stopbits=1,
                            bytesize=8, timeout=0)
        rospy.loginfo("成功打开串口")
        return uart
    except serial.SerialException as e:
        rospy.logwarn("无法打开串口: %s", str(e))
        return None

def query_state_continuously():
    """
    持续查询状态的线程函数[6](@ref)
    """
    rospy.Subscriber("/joint_states", JointState, arm.joint_states_callback)
    rospy.spin()

# 全局变量用于存储当前关节状态
current_joint_angles = [0.0, 0.0, 0.0, 0.0]

def joint_states_callback(msg):
    """
    关节状态回调函数，更新当前关节角度[2](@ref)
    """
    global current_joint_angles
    try:
        if len(msg.position) >= 4:
            current_joint_angles = list(msg.position)[:4]
            rospy.logdebug("更新关节角度: %s", current_joint_angles)
    except Exception as e:
        rospy.logwarn("处理关节状态时出错: %s", str(e))

if __name__ == '__main__':
    # 参数配置
    SERVO_PORT_NAME = '/dev/ttyUSB0'
    SERVO_BAUDRATE = 115200
    SERVO_IDS = [0, 1, 2, 3]
    
    GIMBAL_TYPE = 1
    ARM_TYPE = 0
    PUBLISH_RATE = 10  # 发布频率(Hz)
    
    try:
        rospy.init_node('arm_forward_kinematics', anonymous=True)
        
        # 获取仿真参数
        IF_SIMULATION = rospy.get_param('/if_simulation', False)
        rospy.loginfo("仿真模式: %s", IF_SIMULATION)
        
        # 初始化硬件接口
        if not IF_SIMULATION:
            uart = create_serial_port(SERVO_PORT_NAME)
            if uart is None:
                uart = create_serial_port('/dev/ttyUSB1')
        else:
            uart = None
            
        # 初始化机械臂对象
        arm = ARM.ARM(uart=uart, SERVO_IDS=SERVO_IDS, if_simulation=IF_SIMULATION)
        uav_arm = UAV_ARM.UAV_ARM()
        
        # 设置机械臂参数
        uav_arm.L1 = uav_arm.realL1
        uav_arm.L2 = uav_arm.realL2
        uav_arm.L3 = uav_arm.realL3
        
        # 初始化正运动学计算器
        arm_fk = ArmForwardKinematics(uav_arm)
        
        # 设置关节状态回调[6](@ref)
        rospy.Subscriber("/joint_states", JointState, joint_states_callback)
        
        # 创建并启动查询线程
        angle_thread = threading.Thread(target=query_state_continuously)
        angle_thread.setDaemon(True)
        angle_thread.start()
        
        if not IF_SIMULATION:
            arm_query_thread = threading.Thread(target=arm.keep_query_all_servos, daemon=True)
            arm_query_thread.start()
            rospy.loginfo("查询舵机角度的线程已启动")
        
        time.sleep(0.5)
        
        # 设置发布频率
        rate = rospy.Rate(PUBLISH_RATE)
        
        rospy.loginfo("机械臂正运动学节点已启动，开始发布末端相对位置...")
        
        while not rospy.is_shutdown():
            try:
                # 获取当前关节角度
                if hasattr(arm, 'angle') and arm.angle is not None:
                    joint_angles = arm.angle
                else:
                    joint_angles = current_joint_angles
                
                # 计算正运动学
                end_effector_pos = arm_fk.calculate_forward_kinematics(joint_angles)
                
                if end_effector_pos is not None:
                    # 发布末端执行器相对位置
                    arm_fk.publish_end_effector_position(end_effector_pos)
                    
                    # # 可选：在控制台输出位置信息（调试用）
                    # if rospy.get_time() % 5 < 0.1:  # 每5秒输出一次
                    #     rospy.loginfo("末端执行器相对位置: x=%.3f, y=%.3f, z=%.3f", 
                    #                 end_effector_pos[0], end_effector_pos[1], end_effector_pos[2])
                
                rate.sleep()
                
            except Exception as e:
                rospy.logerr("主循环出错: %s", str(e))
                rate.sleep()
                
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被用户中断")
    except Exception as e:
        rospy.logerr("节点运行出错: %s", str(e))