#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu,NavSatFix
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Float64
from rosgraph_msgs.msg import Clock
import os
import sys
import time
import serial
import threading
import casadi as ca
import numpy as np
from scipy.spatial.transform import Rotation as R
import math

# 添加当前脚本所在目录到sys.path
current_dir = os.path.dirname(__file__)
utils_path = os.path.join(current_dir, "utils")
sys.path.append(utils_path)

from utils import ARM, config, common, UAV, MPC, UAV_ARM


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
    rospy.Subscriber("/clock", Clock, uav_arm.clock_callback)
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


def arm_velocity_control():
    # 飞机参数（单位：弧度）
    uav_arm.p_b = np.array(
    [[uav.vision_pose.x], [uav.vision_pose.y], [uav.vision_pose.z]],
    dtype=np.float64  # 显式声明浮点类型
    )
    uav_arm.phi = uav.roll*math.pi/180  # 滚转角
    uav_arm.theta = uav.pitch*math.pi/180  # 俯仰角
    uav_arm.delta = uav.yaw*math.pi/180  # 偏航角
    uav_arm.d_phi = uav.angular.y*math.pi/180   # 角速度（rad/s）
    uav_arm.d_theta = uav.angular.x*math.pi/180 
    uav_arm.d_delta = uav.angular.z*math.pi/180 
    # 俯仰 滚转 偏航 (广义速度)
    uav_arm.d_xb = np.array([[uav.velocity.x], [uav.velocity.y], [uav.velocity.z], [uav_arm.d_theta], [uav_arm.d_phi], [uav_arm.d_delta]])
    uav_arm.q = np.array([arm.angle[0], arm.angle[1], arm.angle[2], arm.angle[3]]).reshape(-1, 1)  # 关节角度初始化
    #### DEBUG
    # p_b = np.array([[0], [0], [1]])  # 位置
    print("******************")
    # d_xb = np.array([[0],[0],[0],[0],[0],[0]])
    # d_xb[0,0]=0
    # d_xb[1,0]=0
    # d_xb[2,0]=0
    # phi = 0
    # theta = 0
    # delta = -math.pi/2
    print("pos_target",uav_arm.pos_target)
    # print("d_xb",uav_arm.d_xb)
    # print("p_b",uav_arm.p_b)
    #### DEBUG
    # 机械臂参数（单位：弧度）
    # 参数更新
    mpc.state_now = ca.DM([uav_arm.q[0], uav_arm.q[1], uav_arm.q[2], uav_arm.q[3]])
    X0 = ca.repmat(mpc.state_now, 1, mpc.N + 1)  # initial state full
    R_z = np.array([
        [np.cos(uav_arm.delta), -np.sin(uav_arm.delta), 0],
        [np.sin(uav_arm.delta), np.cos(uav_arm.delta), 0],
        [0, 0, 1]
    ])
    R_x = np.array([
        [1, 0, 0],
        [0, np.cos(uav_arm.theta), -np.sin(uav_arm.theta)],
        [0, np.sin(uav_arm.theta), np.cos(uav_arm.theta)]
    ])
    R_y = np.array([
        [np.cos(uav_arm.phi), 0, np.sin(uav_arm.phi)],
        [0, 1, 0],
        [-np.sin(uav_arm.phi), 0, np.cos(uav_arm.phi)]
    ])
    uav_arm.R_b = R_x @ R_y @ R_z
    # 机械臂正运动学
    q0 = uav_arm.q[0, 0];
    q1 = uav_arm.q[1, 0];
    q2 = uav_arm.q[2, 0];
    q3 = uav_arm.q[3, 0]
    arm_z = -(uav_arm.L1 * np.cos(q1) + uav_arm.L2 * np.cos(q1 + q2) + uav_arm.L3 * np.cos(q1 + q2 + q3))
    arm_y = np.cos(q0) * (uav_arm.L1 * np.sin(q1) + uav_arm.L2 * np.sin(q1 + q2) + uav_arm.L3 * np.sin(q1 + q2 + q3))
    arm_x = -np.sin(q0) * (uav_arm.L1 * np.sin(q1) + uav_arm.L2 * np.sin(q1 + q2) + uav_arm.L3 * np.sin(q1 + q2 + q3))
    p_eb = np.array([[arm_x], [arm_y], [arm_z]])
    # 末端执行器状态
    p_e = uav_arm.p_b + np.dot(uav_arm.R_b,(uav_arm.p_delta + p_eb))
    print("p_e",p_e)
    # 计算末端执行器一系列的期望姿态
    arg_p_arm = mpc.get_target_path(uav_arm)
    # 使用mpc进行计算
    p = ca.vertcat(
        mpc.state_now,  # current state
        mpc.state_target,  # target state
        arg_p_arm
    )
    mpc.set_reference(p)
    mpc.set_x0(X0, mpc.u0)
    X0, mpc.u0 = mpc.get_states_and_control()
    uav_arm.d_q = mpc.u0[:,0]
    
    return uav_arm.d_q

if __name__ == '__main__':
    # 参数配置
    SERVO_PORT_NAME = '/dev/ttyUSB0'  # 舵机串口号
    SERVO_BAUDRATE = 115200  # 舵机的波特率
    SERVO_IDS = [0, 1, 2, 3]  # 云台的舵机的ID号列表

    GIMBAL_TYPE =1
    ARM_TYPE = 0
    IF_SIMULATION = rospy.get_param('/if_simulation', False)
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
        mpc = MPC.MPC()
        uav_arm = UAV_ARM.UAV_ARM()
        mpc.L1 = uav_arm.L1
        mpc.L2 = uav_arm.L2
        mpc.L3 = uav_arm.L3
        mpc.step_horizon = uav_arm.dt
        mpc.u0 = ca.DM.zeros((mpc.n_controls, mpc.N)) 

        # 创建并启动查询带臂无人机状态的线程
        angle_thread = threading.Thread(target=query_state_continuously)
        angle_thread.setDaemon(True)
        angle_thread.start()
        pub_angular = rospy.Publisher('/le_arm_controller/command', Float64MultiArray, queue_size=10)
        time.sleep(0.5)
        arg_p_arm = ca.DM.zeros(mpc.n_tarpos * (mpc.N + 1))

        while not rospy.is_shutdown():
            # 目标位置 x y z
            pos_target = get_target_pos()
            uav_arm.pos_target = pos_target
            if uav_arm.gazebo_time>uav_arm.last_control_time+uav_arm.dt:
                uav_arm.last_control_time = uav_arm.gazebo_time

                d_q = arm_velocity_control()
                # d_q = arm.boundary_q(d_q)
                angular = arm_control(d_q)
                print("angular: ",angular)
                # angular = arm_control_old()
                msg_angular = Float64MultiArray()
                msg_angular.data = angular  # 设置数据部分
                pub_angular.publish(msg_angular)
                

    except rospy.ROSInterruptException:
        pass