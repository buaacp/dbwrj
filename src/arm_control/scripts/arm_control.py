#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu,NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64
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

from utils import ARM, config, common, UAV

# 参数配置
SERVO_PORT_NAME = '/dev/ttyUSB0'  # 舵机串口号
SERVO_BAUDRATE = 115200  # 舵机的波特率
SERVO_IDS = [0, 1, 2, 3]  # 云台的舵机的ID号列表

GIMBAL_TYPE =1
ARM_TYPE = 0
IF_SIMULATION = 1
MAX_SPEED = 0.5
MAX_d_q = 2


def create_serial_port(self,port_name,SERVO_BAUDRATE=115200):
    try:
        # 尝试打开串口
        uart = serial.Serial(port=port_name, baudrate=SERVO_BAUDRATE,
                            parity=serial.PARITY_NONE, stopbits=1,
                            bytesize=8, timeout=0)  # 设置串口超时，避免阻塞
        print("成功打开串口")
        return uart
    except serial.SerialException as e:
        print("无法打开串口")
        return None


def query_state_continuously():
    rospy.Subscriber("/joint_states", JointState, arm.joint_states_callback)
    # rospy.Subscriber('mavros/global_position/rel_alt', Float64, uav.relative_altitude_info_callback)
    # rospy.Subscriber('mavros/imu/data', Imu, uav.attitude_info_callback)  # 仿真时姿态获取用vision_pose
    rospy.Subscriber("mavros/vision_pose/pose", PoseStamped, uav.vision_imu_callback)
    rospy.Subscriber('mavros/local_position/pose', PoseStamped, uav.pose_callback)
    rospy.Subscriber('mavros/local_position/velocity_local', TwistStamped, uav.velocity_callback)
    rospy.Subscriber("mavros/vision_pose/pose", PoseStamped, uav.vision_pose_callback)
    rospy.Subscriber("/weightless_ball/pose", PoseStamped, arm.target_pos_callback)
    rospy.spin()

def get_target_pos():
    pos_target_virtual = np.array([[arm.target_pose.x], [arm.target_pose.y], [arm.target_pose.z]])
    pos_target = pos_target_virtual

    return pos_target

def arm_control(d_q):
    # angel_target = [0,0,0,0]
    angular = [-1*d_q[0,0],-1*d_q[1,0],-1*d_q[2,0],-1*d_q[3,0]]
    # print("angular",angular)
    # for i in SERVO_IDS:
    #     angular[i] = -0.2*(angel_target[i]-arm.angle[i])
    return angular

def arm_control_old():
    # angel_target = [0,0,0,30]
    # for i in SERVO_IDS:
    #     angular[i] = -0.2*(angel_target[i]-arm.angle[i])
    # print("关节角度：",arm.angle)
    # print("关节控制速度：",angular)
    angular = [0,0,0,0.02]
    return angular



def arm_velocity_control(pos_target):
    # 基础参数
    I = np.eye(3)
    Z = np.zeros((3, 3))
    # 飞机参数（单位：弧度）
    p_b = np.array([[uav.vision_pose.x], [uav.vision_pose.y], [uav.vision_pose.z]])  # 位置
    phi = uav.roll*math.pi/180  # 滚转角
    theta = uav.pitch*math.pi/180  # 俯仰角
    delta = uav.yaw*math.pi/180  # 偏航角
    d_phi = uav.angular.y*math.pi/180   # 角速度（rad/s）
    d_theta = uav.angular.x*math.pi/180 
    d_delta = uav.angular.z*math.pi/180 
    # 俯仰 滚转 偏航 (广义速度)
    d_xb = np.array([[uav.velocity.x], [uav.velocity.y], [uav.velocity.z], [d_theta], [d_phi], [d_delta]])
    print("d_xb",d_xb)
    #### DEBUG
    # p_b = np.array([[0], [0], [1]])  # 位置
    # d_xb = np.array([[0],[0],[0],[0],[0],[0]])
    d_xb[0,0]=0
    d_xb[1,0]=0
    d_xb[2,0]=0
    # phi = 0
    # theta = 0
    # delta = -math.pi/2
    #### DEBUG
    # 机械臂参数（单位：弧度）
    p_delta = np.array([[0.0], [-0.096], [-0.135]])  # 基座偏移
    L1 = arm.L1
    L2 = arm.L2
    L3_real = arm.L3
    q = np.array([arm.angle[0], arm.angle[1], arm.angle[2], arm.angle[3]]).reshape(-1, 1)  # 关节角度初始化

    # 雅可比矩阵 俯仰 滚转 偏航
    J_eb_omega = np.array([
        [0, 1, 1, 1],
        [0, 0, 0, 0],
        [1, 0, 0, 0]
    ])

    # 计算无人机旋转矩阵
    T_b = np.array([
        [1, 0, -np.sin(theta)],
        [0, np.cos(phi), np.cos(theta) * np.sin(phi)],
        [0, -np.sin(phi), np.cos(theta) * np.cos(phi)]
    ])

    I_b = np.block([[I, Z], [Z, T_b]])

    R_z = np.array([
        [np.cos(delta), -np.sin(delta), 0],
        [np.sin(delta), np.cos(delta), 0],
        [0, 0, 1]
    ])
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(theta), -np.sin(theta)],
        [0, np.sin(theta), np.cos(theta)]
    ])
    R_y = np.array([
        [np.cos(phi), 0, np.sin(phi)],
        [0, 1, 0],
        [-np.sin(phi), 0, np.cos(phi)]
    ])
    R_b = np.dot(R_x, np.dot(R_y, R_z))

    # 计算期望末端执行器状态
    p_b_delta = p_b + np.dot(R_b,p_delta)
    b_tar_dis = pos_target - p_b_delta
    distance_b = np.linalg.norm(b_tar_dis)
    L3 = max(L3_real, distance_b - 0.2)
    pose_target = np.array([
        [np.arcsin(b_tar_dis[2, 0] / distance_b)],
        [0],
        [-np.arctan2(b_tar_dis[0, 0], b_tar_dis[1, 0])]
    ])

    # 机械臂正运动学
    q0 = q[0, 0];
    q1 = q[1, 0];
    q2 = q[2, 0];
    q3 = q[3, 0]
    arm_z = -(L1 * np.cos(q1) + L2 * np.cos(q1 + q2) + L3 * np.cos(q1 + q2 + q3))
    arm_y = np.cos(q0) * (L1 * np.sin(q1) + L2 * np.sin(q1 + q2) + L3 * np.sin(q1 + q2 + q3))
    arm_x = -np.sin(q0) * (L1 * np.sin(q1) + L2 * np.sin(q1 + q2) + L3 * np.sin(q1 + q2 + q3))
    p_eb = np.array([[arm_x], [arm_y], [arm_z]])

    # 雅可比矩阵计算
    S = L1 * np.sin(q1) + L2 * np.sin(q1 + q2) + L3 * np.sin(q1 + q2 + q3)
    C = L1 * np.cos(q1) + L2 * np.cos(q1 + q2) + L3 * np.cos(q1 + q2 + q3)

    J_eb_v = np.array([
        [-np.cos(q0) * S, -np.sin(q0) * C, -np.sin(q0) * (L2 * np.cos(q1 + q2) + L3 * np.cos(q1 + q2 + q3)),
         -np.sin(q0) * L3 * np.cos(q1 + q2 + q3)],
        [-np.sin(q0) * S, np.cos(q0) * C, np.cos(q0) * (L2 * np.cos(q1 + q2) + L3 * np.cos(q1 + q2 + q3)),
         np.cos(q0) * L3 * np.cos(q1 + q2 + q3)],
        [0, S, L2 * np.sin(q1 + q2) + L3 * np.sin(q1 + q2 + q3), L3 * np.sin(q1 + q2 + q3)]
    ])

    J_eb = np.vstack((J_eb_v, J_eb_omega))
    S_jb = -common.skew_symmetric(np.dot(R_b,p_delta)) - common.skew_symmetric(np.dot(R_b,p_eb))
    J_b = np.block([[I, S_jb], [Z, I]])
    J_e = np.dot(np.block([[R_b, Z], [Z, R_b]]),J_eb)

    # 末端执行器状态
    p_e = p_b + np.dot(R_b,(p_delta + p_eb))

    print("p_e",p_e)

    # 四元数计算
    Q_uav = common.angle2quat(delta, phi, theta, 'ZYX')
    Q_arm = common.angle2quat(q0, 0, q1 + q2 + q3 - np.pi / 2, 'ZYX')
    Q_total = common.quatmultiply(Q_uav, Q_arm)
    omega_e = common.quat2euler(Q_total).reshape(3, 1)

    phi_arm_rad = omega_e[0, 0]
    theta_arm_rad = omega_e[1, 0]
    T_e = np.array([
        [1, 0, -np.sin(theta_arm_rad)],
        [0, np.cos(phi_arm_rad), np.cos(theta_arm_rad) * np.sin(phi_arm_rad)],
        [0, -np.sin(phi_arm_rad), np.cos(theta_arm_rad) * np.cos(phi_arm_rad)]
    ])
    I_e = np.block([[I, Z], [Z, T_e]])

    T1 = np.dot(J_b,I_b)
    T2 = J_e


    x_e = np.vstack((p_e, omega_e))

    # 机械臂控制量计算
    delta_x = x_e - np.vstack((pos_target, pose_target))
    W = np.diag([1, 1, 1, 0.1, 0, 0.1])
    delta_x_weighted = np.dot(W,delta_x)
    T2_weighted = np.dot(W,T2)
    T2_pinv_weighted = np.linalg.pinv(T2_weighted)
    T2_pinv = np.linalg.pinv(T2)
    d_q = -0.8 * np.dot(np.dot(T2_pinv_weighted, I_e), delta_x_weighted) - np.dot(np.dot(T2_pinv, T1), d_xb)
    norm_d_q = np.linalg.norm(d_q)
    if norm_d_q > MAX_d_q:
        d_q = d_q * (MAX_d_q / norm_d_q)
    d_xe = np.dot(T1,d_xb) + np.dot(T2,d_q)
    # print("d_xe: ",d_xe)
    return d_q

if __name__ == '__main__':
    try:
        rospy.init_node('arm_control', anonymous=True)
        if not IF_SIMULATION:
            # 尝试创建串口
            uart = create_serial_port(SERVO_PORT_NAME)
            if uart is None:
                uart = create_serial_port('/dev/ttyUSB1')
        else:
            uart = None
        arm = ARM.ARM(uart=uart, SERVO_IDS=SERVO_IDS,if_simulation=IF_SIMULATION)
        uav = UAV.Myuav()
        rate_control = rospy.Rate(arm.control_rate)
        # 创建并启动查询带臂无人机状态的线程
        angle_thread = threading.Thread(target=query_state_continuously)
        angle_thread.setDaemon(True)
        angle_thread.start()
        pub_angular = rospy.Publisher('/le_arm_controller/command', Float64MultiArray, queue_size=10)
        time.sleep(0.5)
        while not rospy.is_shutdown():
            print("----------")
            # 目标位置 x y z
            pos_target = get_target_pos()
            print("pos_target: ",pos_target)
            d_q = arm_velocity_control(pos_target)
            d_q = arm.boundary_q(d_q)
            angular = arm_control(d_q)
            # angular = arm_control_old()
            msg_angular = Float64MultiArray()
            msg_angular.data = angular  # 设置数据部分
            pub_angular.publish(msg_angular)
            # print("roll: ",uav.roll," pitch ",uav.pitch," yaw: ",uav.yaw)
            print("v_roll: ",uav.angular.y," v_pitch: ",uav.angular.x," v_yaw: ",uav.angular.z)
            rate_control.sleep()

    except rospy.ROSInterruptException:
        pass