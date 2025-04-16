#!/usr/bin/env python3
# -*- coding: utf-8 -*

import time
import casadi as ca
import numpy as np
from casadi import sin, cos, pi
import sys
import os


current_dir = os.path.dirname(__file__)
utils_path = os.path.join(current_dir, "utils")
sys.path.append(utils_path)
from utils import UAV_ARM,MPC


if __name__ == '__main__':
    mpc = MPC.MPC()
    uav_arm = UAV_ARM.UAV_ARM()
    mpc.L1 = uav_arm.L1
    mpc.L2 = uav_arm.L2
    mpc.L3 = uav_arm.L3_real
    mpc.step_horizon = uav_arm.dt

    u0 = ca.DM.zeros((mpc.n_controls, mpc.N))  # initial control

    arg_p_arm = ca.DM.zeros(mpc.n_tarpos * (mpc.N + 1))

    for t in np.arange(0, uav_arm.T + uav_arm.dt, uav_arm.dt):
        # 无人机控制量计算
        uav_tar_pos = uav_arm.pos_target + np.array([[0], [0], [0.3]])
        uav_tar_dis = uav_tar_pos - uav_arm.p_b
        distance_uav = np.linalg.norm(uav_tar_dis)
        if distance_uav > uav_arm.MAX_SPEED:
            uav_tar_dis = uav_tar_dis / distance_uav * uav_arm.MAX_SPEED
        uav_arm.d_xb[0:3] = uav_tar_dis

        heading = -np.arctan2(uav_tar_dis[0, 0], uav_tar_dis[1, 0])
        if distance_uav > 1:
            uav_arm.d_xb[5] = -0.1 * (uav_arm.delta - heading)

        # uav_arm.d_xb[0:3] = np.array([[0], [uav_arm.MAX_SPEED], [0]])

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
        # 计算末端执行器一系列的期望姿态
        arg_p_arm = mpc.get_target_path(uav_arm)
        # 使用mpc进行计算
        p = ca.vertcat(
            mpc.state_now,  # current state
            mpc.state_target,  # target state
            arg_p_arm
        )
        mpc.set_reference(p)
        mpc.set_x0(X0, u0)
        X0, u0 = mpc.get_states_and_control()
        uav_arm.d_q = u0[:,0]
        # 状态更新
        uav_arm.q += uav_arm.dt * uav_arm.d_q
        uav_arm.p_b += uav_arm.dt * uav_arm.d_xb[0:3]
        uav_arm.phi += uav_arm.dt * uav_arm.d_xb[4, 0]
        uav_arm.theta += uav_arm.dt * uav_arm.d_xb[3, 0]
        uav_arm.delta += uav_arm.dt * uav_arm.d_xb[5, 0]

        print(uav_arm.d_q)






