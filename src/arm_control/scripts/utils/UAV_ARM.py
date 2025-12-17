import numpy as np
from math import pi
class UAV_ARM:
    def __init__(self):
        # 仿真控制参数
        self.MAX_SPEED = 0.2
        self.MAX_SPEED_CONTROL = 0.5
        self.MAX_d_q = 1
        # 指数衰减系数
        self.k_exp = 0.2

        # 基础参数
        self.I = np.eye(3)
        self.Z = np.zeros((3, 3))
        self.T = 20
        self.dt = 0.1
        self.draw_t = 0.1
        # 飞机参数（单位：弧度）
        self.R_b = np.eye(3)
        self.p_b = np.array([[0.0], [0.0], [0.0]])  # 位置
        self.phi = 0.0 * pi / 180  # 滚转角
        self.theta = 0.0 * pi / 180  # 俯仰角
        self.delta = 0.0 * pi / 180  # 偏航角
        d_phi = 0.0  # 角速度（rad/s）
        d_theta = 0.0
        d_delta = 0.0
        # 俯仰 滚转 偏航
        self.d_xb = np.array([[0.0], [0.0], [0.0], [d_theta], [d_phi], [d_delta]])
        # 目标位置 x y z
        self.pos_target = np.array([[0], [3], [0.65]])

        # 机械臂参数（单位：弧度）
        self.p_delta = np.array([[0.0], [-0.096], [-0.135]])  # 基座偏移
        self.L1 = 0.1049
        self.L2 = 0.0884
        self.L3 = 0.12
        self.realL1 = 0.0814
        self.realL2 = 0.07735
        self.realL3 = 0.13865

        self.q = np.deg2rad([0, 45, 45, 0]).reshape(-1, 1)  # 关节角度初始化
        self.d_q = np.zeros((4, 1))  # 关节角速度

        # 雅可比矩阵 俯仰 滚转 偏航
        self.J_eb_omega = np.array([
            [0, 1, 1, 1],
            [0, 0, 0, 0],
            [1, 0, 0, 0]
        ])
        self.gazebo_time = 0
        self.last_control_time = 0
        self.control_step = 3
        self.control_count = 0

    def clock_callback(self,msg):
        # 从消息中提取仿真时间（单位：秒）
        self.gazebo_time = msg.clock.to_sec()