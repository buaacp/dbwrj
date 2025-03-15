#!/usr/bin/env python
# -*- coding: UTF-8 -*-

import numpy as np

def angle2quat(delta, phi, theta, sequence):
    """
    将欧拉角转换为四元数

    参数:
        delta (float): 绕第一轴的旋转角度（单位：弧度）
        phi (float): 绕第二轴的旋转角度（单位：弧度）
        theta (float): 绕第三轴的旋转角度（单位：弧度）
        sequence (str): 旋转顺序，例如 'ZYX' 表示绕 Z 轴、Y 轴、X 轴的旋转

    返回:
        numpy.ndarray: 四元数，形式为 [w, x, y, z]
    """
    # 检查输入参数
    if len(sequence) != 3:
        raise ValueError("旋转顺序必须是三个字符的字符串，例如 'ZYX'")

    # 根据旋转顺序计算四元数
    if sequence.upper() == 'ZYX':
        # 绕 Z 轴旋转的四元数
        qZ = np.array([np.cos(delta / 2), 0, 0, np.sin(delta / 2)])
        # 绕 Y 轴旋转的四元数
        qY = np.array([np.cos(phi / 2), 0, np.sin(phi / 2), 0])
        # 绕 X 轴旋转的四元数
        qX = np.array([np.cos(theta / 2), np.sin(theta / 2), 0, 0])

        # 四元数相乘（先 Z，再 Y，最后 X）
        Q_uav = quatmultiply(quatmultiply(qZ, qY), qX)

        return Q_uav
    else:
        raise ValueError("不支持的旋转顺序")


def quatmultiply(q1, q2):
    """
    四元数相乘

    参数:
        q1 (numpy.ndarray): 第一个四元数，形式为 [w, x, y, z]
        q2 (numpy.ndarray): 第二个四元数，形式为 [w, x, y, z]

    返回:
        numpy.ndarray: 两个四元数的乘积，形式为 [w, x, y, z]
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.array([w, x, y, z])


def quat2euler(q, sequence='ZYX'):
    """
    四元数转欧拉角

    参数:
        q (numpy.ndarray): 四元数，格式为 [w, x, y, z]
        sequence (str): 旋转顺序，默认为 'ZYX'

    返回:
        numpy.ndarray: 欧拉角，格式为 [roll, pitch, yaw]（单位：弧度）
    """
    w, x, y, z = q

    if sequence.upper() == 'ZYX':
        # 姿态角（pitch）
        sin_pitch = 2 * (w * y - x * z)
        pitch = np.arcsin(sin_pitch)

        # 航向角（yaw）
        sin_yaw = 2 * (w * z + x * y)
        cos_yaw = 1 - 2 * (y ** 2 + z ** 2)
        yaw = np.arctan2(sin_yaw, cos_yaw)

        # 滚转角（roll）
        sin_roll = 2 * (w * x + y * z)
        cos_roll = 1 - 2 * (x ** 2 + y ** 2)
        roll = np.arctan2(sin_roll, cos_roll)

        return np.array([roll, pitch, yaw])
    else:
        raise ValueError("不支持的旋转顺序")
# 反对称矩阵生成函数
def skew_symmetric(v):
    return np.array([
        [0, -v[2, 0], v[1, 0]],
        [v[2, 0], 0, -v[0, 0]],
        [-v[1, 0], v[0, 0], 0]
    ])
